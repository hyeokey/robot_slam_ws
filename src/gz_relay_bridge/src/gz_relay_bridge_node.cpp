#include <memory>
#include <string>
#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <gz/transport/Node.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/odometry.pb.h>

class GzRelayBridge : public rclcpp::Node {
public:
  GzRelayBridge() : Node("gz_relay_bridge") {
    // ---- GZ topic params ----
    gz_scan_topic_   = declare_parameter<std::string>(
      "gz_scan_topic",
      "/scan_omni");

    gz_clock_topic_  = declare_parameter<std::string>(
      "gz_clock_topic",
      "/clock");

    gz_odom_topic_   = declare_parameter<std::string>(
      "gz_odom_topic",
      "/model/tugbot/odometry");

    gz_cmdvel_topic_ = declare_parameter<std::string>(
      "gz_cmdvel_topic",
      "/model/tugbot/cmd_vel");

    // ---- ROS topic params ----
    ros_scan_topic_  = declare_parameter<std::string>("ros_scan_topic", "/scan");
    ros_clock_topic_ = declare_parameter<std::string>("ros_clock_topic", "/clock");
    ros_odom_topic_  = declare_parameter<std::string>("ros_odom_topic", "/odom");
    ros_cmdvel_topic_= declare_parameter<std::string>("ros_cmdvel_topic", "/cmd_vel");

    // ---- Frame params (SLAM-friendly) ----
    odom_frame_id_   = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_   = declare_parameter<std::string>("base_frame_id", "base_link");
    scan_frame_id_   = declare_parameter<std::string>("scan_frame_id", "laser");
    zero_odom_on_start_ = declare_parameter<bool>("zero_odom_on_start", true);
    tf_publish_rate_hz_ = declare_parameter<double>("tf_publish_rate_hz", 50.0);

    // ---- ROS pub/sub ----
    scan_pub_  = create_publisher<sensor_msgs::msg::LaserScan>(ros_scan_topic_, 10);
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(ros_clock_topic_, 10);
    odom_pub_  = create_publisher<nav_msgs::msg::Odometry>(ros_odom_topic_, 10);

    cmdvel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      ros_cmdvel_topic_, 10,
      std::bind(&GzRelayBridge::onRosCmdVel, this, std::placeholders::_1));

    // TF broadcaster (odom -> base_link)
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    const auto tf_period =
      std::chrono::duration<double>(1.0 / std::max(1.0, tf_publish_rate_hz_));
    tf_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(tf_period),
      std::bind(&GzRelayBridge::publishLatestTf, this));

    // ---- GZ subscribe/advertise ----
    const bool ok_clock = gz_node_.Subscribe(gz_clock_topic_, &GzRelayBridge::onGzClock, this);
    const bool ok_scan  = gz_node_.Subscribe(gz_scan_topic_,  &GzRelayBridge::onGzScan,  this);
    const bool ok_odom  = gz_node_.Subscribe(gz_odom_topic_,  &GzRelayBridge::onGzOdom,  this);

    gz_cmdvel_pub_ = gz_node_.Advertise<gz::msgs::Twist>(gz_cmdvel_topic_);

    RCLCPP_INFO(get_logger(), "GZ sub clock: %s (%s)", gz_clock_topic_.c_str(), ok_clock ? "OK" : "FAIL");
    RCLCPP_INFO(get_logger(), "GZ sub scan : %s (%s)", gz_scan_topic_.c_str(),  ok_scan  ? "OK" : "FAIL");
    RCLCPP_INFO(get_logger(), "GZ sub odom : %s (%s)", gz_odom_topic_.c_str(),  ok_odom  ? "OK" : "FAIL");
    RCLCPP_INFO(get_logger(), "GZ pub cmd  : %s", gz_cmdvel_topic_.c_str());

    RCLCPP_INFO(get_logger(), "ROS pub /scan->%s, /clock->%s, /odom->%s",
                ros_scan_topic_.c_str(), ros_clock_topic_.c_str(), ros_odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "ROS sub /cmd_vel<- %s", ros_cmdvel_topic_.c_str());

    RCLCPP_INFO(get_logger(), "Frames: odom=%s base=%s scan=%s",
                odom_frame_id_.c_str(), base_frame_id_.c_str(), scan_frame_id_.c_str());
  }

private:
  static double normalizeAngle(double angle) {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static double yawFromQuaternion(
    double x, double y, double z, double w)
  {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static std::string sanitize_frame(std::string s) {
    // Gazebo가 "tugbot::base_link" 같은 걸 주는 경우가 있어서 ROS TF 프레임에 안전하게
    std::replace(s.begin(), s.end(), ':', '_');
    // "::" -> "__" 로도 바뀜
    return s;
  }

  rclcpp::Time stamp_now() const {
    // use_sim_time:=true 라면 /clock이 들어오면 자동으로 rclcpp::Clock가 sim time을 씀.
    // 그래도 우리는 gz clock을 받아 last_sim_을 저장해두고, 그걸 우선 사용.
    if (have_sim_time_) {
      return rclcpp::Time(static_cast<int64_t>(last_sim_sec_) * 1000000000LL +
                          static_cast<int64_t>(last_sim_nsec_),
                          RCL_ROS_TIME);
    }
    return this->now();
  }

  // ---------- Callbacks ----------
  void onGzClock(const gz::msgs::Clock &msg) {
    last_sim_sec_  = msg.sim().sec();
    last_sim_nsec_ = msg.sim().nsec();
    have_sim_time_ = true;

    rosgraph_msgs::msg::Clock c;
    c.clock.sec     = static_cast<int32_t>(last_sim_sec_);
    c.clock.nanosec = static_cast<uint32_t>(last_sim_nsec_);
    clock_pub_->publish(c);
  }

  void onGzScan(const gz::msgs::LaserScan &msg) {
  sensor_msgs::msg::LaserScan out;
  if (have_latest_tf_) {
    out.header.stamp = latest_tf_.header.stamp;
  } else {
    out.header.stamp = stamp_now();
  }
  out.header.frame_id = scan_frame_id_;

  out.angle_min       = msg.angle_min();
  out.angle_max       = msg.angle_max();
  out.angle_increment = msg.angle_step();

  out.time_increment  = 0.0;
  out.scan_time       = 0.0;

  out.range_min       = msg.range_min();
  out.range_max       = msg.range_max();

  const int horizontal_count = static_cast<int>(msg.count());
  const int vertical_count   = static_cast<int>(msg.vertical_count());
  const int total_ranges     = static_cast<int>(msg.ranges_size());

  if (horizontal_count <= 0 || vertical_count <= 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Invalid scan shape: count=%d, vertical_count=%d",
      horizontal_count, vertical_count);
    return;
  }

  if (total_ranges < horizontal_count * vertical_count) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "ranges_size (%d) < count*vertical_count (%d)",
      total_ranges, horizontal_count * vertical_count);
    return;
  }

  // 가운데 층 선택
  const int selected_layer = vertical_count / 2;
  const int offset = selected_layer * horizontal_count;

  out.ranges.resize(horizontal_count);
  for (int i = 0; i < horizontal_count; ++i) {
    out.ranges[i] = msg.ranges(offset + i);
  }

  scan_pub_->publish(out);
}

  void onGzOdom(const gz::msgs::Odometry &msg) {
    const double raw_x = msg.pose().position().x();
    const double raw_y = msg.pose().position().y();
    const double raw_z = msg.pose().position().z();

    const double raw_qx = msg.pose().orientation().x();
    const double raw_qy = msg.pose().orientation().y();
    const double raw_qz = msg.pose().orientation().z();
    const double raw_qw = msg.pose().orientation().w();

    const double raw_yaw = yawFromQuaternion(raw_qx, raw_qy, raw_qz, raw_qw);

    if (zero_odom_on_start_ && !have_initial_odom_) {
      initial_x_ = raw_x;
      initial_y_ = raw_y;
      initial_z_ = raw_z;
      initial_yaw_ = raw_yaw;
      have_initial_odom_ = true;
    }

    double odom_x = raw_x;
    double odom_y = raw_y;
    double odom_z = raw_z;
    double odom_yaw = raw_yaw;

    if (zero_odom_on_start_) {
      const double dx = raw_x - initial_x_;
      const double dy = raw_y - initial_y_;
      const double cos_yaw = std::cos(initial_yaw_);
      const double sin_yaw = std::sin(initial_yaw_);

      odom_x =  cos_yaw * dx + sin_yaw * dy;
      odom_y = -sin_yaw * dx + cos_yaw * dy;
      odom_z = raw_z - initial_z_;
      odom_yaw = normalizeAngle(raw_yaw - initial_yaw_);
    }

    // 1) ROS /odom publish
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp_now();
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id  = base_frame_id_;

    // pose
    odom.pose.pose.position.x = odom_x;
    odom.pose.pose.position.y = odom_y;
    odom.pose.pose.position.z = odom_z;

    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(odom_yaw * 0.5);
    odom.pose.pose.orientation.w = std::cos(odom_yaw * 0.5);

    // twist
    odom.twist.twist.linear.x  = msg.twist().linear().x();
    odom.twist.twist.linear.y  = msg.twist().linear().y();
    odom.twist.twist.linear.z  = msg.twist().linear().z();

    odom.twist.twist.angular.x = msg.twist().angular().x();
    odom.twist.twist.angular.y = msg.twist().angular().y();
    odom.twist.twist.angular.z = msg.twist().angular().z();

    odom_pub_->publish(odom);

    latest_tf_.header.stamp = odom.header.stamp;
    latest_tf_.header.frame_id = odom_frame_id_;
    latest_tf_.child_frame_id  = base_frame_id_;
    latest_tf_.transform.translation.x = odom.pose.pose.position.x;
    latest_tf_.transform.translation.y = odom.pose.pose.position.y;
    latest_tf_.transform.translation.z = odom.pose.pose.position.z;
    latest_tf_.transform.rotation = odom.pose.pose.orientation;
    have_latest_tf_ = true;

    tf_broadcaster_->sendTransform(latest_tf_);
  }

  void publishLatestTf() {
    if (!have_latest_tf_) {
      return;
    }

    auto tf_msg = latest_tf_;
    tf_msg.header.stamp = stamp_now();
    tf_broadcaster_->sendTransform(tf_msg);
  }

  void onRosCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    gz::msgs::Twist gz_tw;
    gz_tw.mutable_linear()->set_x(msg->linear.x);
    gz_tw.mutable_linear()->set_y(msg->linear.y);
    gz_tw.mutable_linear()->set_z(msg->linear.z);
    gz_tw.mutable_angular()->set_x(msg->angular.x);
    gz_tw.mutable_angular()->set_y(msg->angular.y);
    gz_tw.mutable_angular()->set_z(msg->angular.z);

    gz_cmdvel_pub_.Publish(gz_tw);
  }

  // ---------- Members ----------
  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher gz_cmdvel_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub_;

  std::string gz_scan_topic_, gz_clock_topic_, gz_odom_topic_, gz_cmdvel_topic_;
  std::string ros_scan_topic_, ros_clock_topic_, ros_odom_topic_, ros_cmdvel_topic_;
  std::string odom_frame_id_, base_frame_id_, scan_frame_id_;
  bool zero_odom_on_start_{true};
  double tf_publish_rate_hz_{50.0};

  bool have_sim_time_{false};
  uint64_t last_sim_sec_{0};
  uint64_t last_sim_nsec_{0};
  bool have_initial_odom_{false};
  double initial_x_{0.0};
  double initial_y_{0.0};
  double initial_z_{0.0};
  double initial_yaw_{0.0};
  bool have_latest_tf_{false};
  geometry_msgs::msg::TransformStamped latest_tf_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzRelayBridge>());
  rclcpp::shutdown();
  return 0;
}
