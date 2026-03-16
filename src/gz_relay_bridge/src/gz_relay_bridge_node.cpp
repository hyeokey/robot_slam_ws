#include <memory>
#include <string>
#include <algorithm>

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
      "/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan");

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

    // ---- ROS pub/sub ----
    scan_pub_  = create_publisher<sensor_msgs::msg::LaserScan>(ros_scan_topic_, 10);
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(ros_clock_topic_, 10);
    odom_pub_  = create_publisher<nav_msgs::msg::Odometry>(ros_odom_topic_, 10);

    cmdvel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      ros_cmdvel_topic_, 10,
      std::bind(&GzRelayBridge::onRosCmdVel, this, std::placeholders::_1));

    // TF broadcaster (odom -> base_link)
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

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
  out.header.stamp = stamp_now();
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
    // 1) ROS /odom publish
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp_now();
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id  = base_frame_id_;

    // pose
    odom.pose.pose.position.x = msg.pose().position().x();
    odom.pose.pose.position.y = msg.pose().position().y();
    odom.pose.pose.position.z = msg.pose().position().z();

    odom.pose.pose.orientation.x = msg.pose().orientation().x();
    odom.pose.pose.orientation.y = msg.pose().orientation().y();
    odom.pose.pose.orientation.z = msg.pose().orientation().z();
    odom.pose.pose.orientation.w = msg.pose().orientation().w();

    // twist
    odom.twist.twist.linear.x  = msg.twist().linear().x();
    odom.twist.twist.linear.y  = msg.twist().linear().y();
    odom.twist.twist.linear.z  = msg.twist().linear().z();

    odom.twist.twist.angular.x = msg.twist().angular().x();
    odom.twist.twist.angular.y = msg.twist().angular().y();
    odom.twist.twist.angular.z = msg.twist().angular().z();

    odom_pub_->publish(odom);

    // 2) TF: odom -> base_link broadcast (slam_toolbox가 가장 좋아함)
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = odom.header.stamp;
    t.header.frame_id = odom_frame_id_;
    t.child_frame_id  = base_frame_id_;

    t.transform.translation.x = odom.pose.pose.position.x;
    t.transform.translation.y = odom.pose.pose.position.y;
    t.transform.translation.z = odom.pose.pose.position.z;

    t.transform.rotation = odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
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

  bool have_sim_time_{false};
  uint64_t last_sim_sec_{0};
  uint64_t last_sim_nsec_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzRelayBridge>());
  rclcpp::shutdown();
  return 0;
}