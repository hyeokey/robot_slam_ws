#include <cmath>
#include <fstream>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <gz/msgs/odometry.pb.h>
#include <gz/transport/Node.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class LocalizationErrorMonitor : public rclcpp::Node {
public:
  LocalizationErrorMonitor() : Node("localization_error_monitor") {
    gz_odom_topic_ = declare_parameter<std::string>(
      "gz_odom_topic", "/model/tugbot/odometry");
    amcl_pose_topic_ = declare_parameter<std::string>(
      "amcl_pose_topic", "/amcl_pose");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    map_frame_id_ = declare_parameter<std::string>("map_frame_id", "map");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_footprint");
    use_tf_estimate_ = declare_parameter<bool>("use_tf_estimate", true);
    initial_pose_topic_ = declare_parameter<std::string>(
      "initial_pose_topic", "/initialpose");
    error_topic_ = declare_parameter<std::string>(
      "error_topic", "/localization_error");
    odom_error_topic_ = declare_parameter<std::string>(
      "odom_error_topic", "/odom_error");
    initial_pose_error_topic_ = declare_parameter<std::string>(
      "initial_pose_error_topic", "/initial_pose_error");
    align_on_first_sample_ = declare_parameter<bool>(
      "align_on_first_sample", true);
    use_fixed_world_to_map_ = declare_parameter<bool>(
      "use_fixed_world_to_map", false);
    map_from_world_x_ = declare_parameter<double>("map_from_world_x", 0.0);
    map_from_world_y_ = declare_parameter<double>("map_from_world_y", 0.0);
    map_from_world_yaw_ = declare_parameter<double>("map_from_world_yaw", 0.0);
    log_rate_hz_ = declare_parameter<double>("log_rate_hz", 1.0);
    csv_path_ = declare_parameter<std::string>("csv_path", "");

    amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      amcl_pose_topic_, 10,
      std::bind(&LocalizationErrorMonitor::onAmclPose, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&LocalizationErrorMonitor::onOdom, this, std::placeholders::_1));
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_, 10,
      std::bind(&LocalizationErrorMonitor::onInitialPose, this, std::placeholders::_1));

    error_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(error_topic_, 10);
    odom_error_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(odom_error_topic_, 10);
    initial_pose_error_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(initial_pose_error_topic_, 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    const bool gz_ok = gz_node_.Subscribe(
      gz_odom_topic_, &LocalizationErrorMonitor::onGzOdom, this);

    if (!csv_path_.empty()) {
      csv_.open(csv_path_, std::ios::out | std::ios::trunc);
      if (csv_.is_open()) {
        csv_ << "time_sec,gt_x,gt_y,gt_yaw,amcl_x,amcl_y,amcl_yaw,"
             << "error_x,error_y,error_yaw,error_distance,"
             << "odom_x,odom_y,odom_yaw,odom_error_x,odom_error_y,odom_error_yaw,odom_error_distance,"
             << "sigma_x,sigma_y,sigma_yaw\n";
      } else {
        RCLCPP_WARN(get_logger(), "Failed to open csv_path: %s", csv_path_.c_str());
      }
    }

    const double safe_log_rate = std::max(0.1, log_rate_hz_);
    const auto period = std::chrono::duration<double>(1.0 / safe_log_rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&LocalizationErrorMonitor::logError, this));

    RCLCPP_INFO(
      get_logger(),
      "Monitoring Gazebo GT '%s' (%s) vs AMCL '%s'; align_on_first_sample=%s",
      gz_odom_topic_.c_str(), gz_ok ? "OK" : "FAIL",
      amcl_pose_topic_.c_str(), align_on_first_sample_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(),
      "TF estimate fallback: %s (%s -> %s)",
      use_tf_estimate_ ? "enabled" : "disabled",
      map_frame_id_.c_str(), base_frame_id_.c_str());

    if (use_fixed_world_to_map_) {
      RCLCPP_INFO(
        get_logger(),
        "Using fixed world->map transform: x=%.3f, y=%.3f, yaw=%.3f rad",
        map_from_world_x_, map_from_world_y_, map_from_world_yaw_);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Fixed world->map transform is disabled; /initialpose absolute error will be reported as unavailable.");
    }
  }

private:
  struct Pose2D {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  static double normalizeAngle(double angle) {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static double yawFromQuaternion(double x, double y, double z, double w) {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static double safeSigma(double variance) {
    return variance > 0.0 ? std::sqrt(variance) : 0.0;
  }

  Pose2D transformWorldToMap(const Pose2D &world_pose) const {
    const double cos_yaw = std::cos(map_from_world_yaw_);
    const double sin_yaw = std::sin(map_from_world_yaw_);

    Pose2D out;
    out.x = map_from_world_x_ + cos_yaw * world_pose.x - sin_yaw * world_pose.y;
    out.y = map_from_world_y_ + sin_yaw * world_pose.x + cos_yaw * world_pose.y;
    out.yaw = normalizeAngle(map_from_world_yaw_ + world_pose.yaw);
    out.stamp = world_pose.stamp;
    return out;
  }

  bool lookupTfEstimate(Pose2D &pose) {
    if (!use_tf_estimate_ || !tf_buffer_) {
      return false;
    }

    try {
      const auto tf = tf_buffer_->lookupTransform(
        map_frame_id_, base_frame_id_, tf2::TimePointZero);
      pose.x = tf.transform.translation.x;
      pose.y = tf.transform.translation.y;
      pose.yaw = yawFromQuaternion(
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w);
      pose.stamp = tf.header.stamp;
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  Pose2D transformGroundTruthToAmclFrame(const Pose2D &gt) const {
    if (use_fixed_world_to_map_) {
      return transformWorldToMap(gt);
    }

    if (!align_on_first_sample_ || !aligned_) {
      return gt;
    }

    const double dx = gt.x - initial_gt_.x;
    const double dy = gt.y - initial_gt_.y;
    const double cos_gt = std::cos(initial_gt_.yaw);
    const double sin_gt = std::sin(initial_gt_.yaw);

    const double rel_x = cos_gt * dx + sin_gt * dy;
    const double rel_y = -sin_gt * dx + cos_gt * dy;
    const double rel_yaw = normalizeAngle(gt.yaw - initial_gt_.yaw);

    const double cos_amcl = std::cos(initial_amcl_.yaw);
    const double sin_amcl = std::sin(initial_amcl_.yaw);

    Pose2D out;
    out.x = initial_amcl_.x + cos_amcl * rel_x - sin_amcl * rel_y;
    out.y = initial_amcl_.y + sin_amcl * rel_x + cos_amcl * rel_y;
    out.yaw = normalizeAngle(initial_amcl_.yaw + rel_yaw);
    out.stamp = gt.stamp;
    return out;
  }

  void onGzOdom(const gz::msgs::Odometry &msg) {
    Pose2D pose;
    pose.x = msg.pose().position().x();
    pose.y = msg.pose().position().y();
    pose.yaw = yawFromQuaternion(
      msg.pose().orientation().x(),
      msg.pose().orientation().y(),
      msg.pose().orientation().z(),
      msg.pose().orientation().w());
    pose.stamp = now();

    std::lock_guard<std::mutex> lock(mutex_);
    latest_gt_ = pose;
    have_gt_ = true;
  }

  void onAmclPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    Pose2D pose;
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.yaw = yawFromQuaternion(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    pose.stamp = msg->header.stamp;

    std::lock_guard<std::mutex> lock(mutex_);
    latest_amcl_ = pose;
    sigma_x_ = safeSigma(msg->pose.covariance[0]);
    sigma_y_ = safeSigma(msg->pose.covariance[7]);
    sigma_yaw_ = safeSigma(msg->pose.covariance[35]);
    have_amcl_ = true;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    Pose2D pose;
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.yaw = yawFromQuaternion(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    pose.stamp = msg->header.stamp;

    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_ = pose;
    have_odom_ = true;
  }

  void onInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    Pose2D requested;
    requested.x = msg->pose.pose.position.x;
    requested.y = msg->pose.pose.position.y;
    requested.yaw = yawFromQuaternion(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    requested.stamp = msg->header.stamp;

    Pose2D gt;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_gt_) {
        RCLCPP_WARN(
          get_logger(),
          "Received /initialpose, but Gazebo ground truth is not available yet.");
        return;
      }
      gt = latest_gt_;
    }

    if (!use_fixed_world_to_map_) {
      RCLCPP_WARN(
        get_logger(),
        "Received /initialpose=(%.3f, %.3f, %.3f), raw_gz=(%.3f, %.3f, %.3f), "
        "but fixed world->map transform is disabled. Enable use_fixed_world_to_map to compute initial pose error.",
        requested.x, requested.y, requested.yaw,
        gt.x, gt.y, gt.yaw);
      return;
    }

    const Pose2D gt_map = transformWorldToMap(gt);
    const double error_x = requested.x - gt_map.x;
    const double error_y = requested.y - gt_map.y;
    const double error_yaw = normalizeAngle(requested.yaw - gt_map.yaw);
    const double error_distance = std::hypot(error_x, error_y);

    geometry_msgs::msg::Vector3Stamped error_msg;
    error_msg.header.stamp = now();
    error_msg.header.frame_id = "map";
    error_msg.vector.x = error_x;
    error_msg.vector.y = error_y;
    error_msg.vector.z = error_yaw;
    initial_pose_error_pub_->publish(error_msg);

    RCLCPP_INFO(
      get_logger(),
      "initialpose_check: gz_world=(%.3f, %.3f, %.3f) gt_map=(%.3f, %.3f, %.3f) "
      "requested=(%.3f, %.3f, %.3f) err=(x %.3f, y %.3f, yaw %.3f rad, dist %.3f m)",
      gt.x, gt.y, gt.yaw,
      gt_map.x, gt_map.y, gt_map.yaw,
      requested.x, requested.y, requested.yaw,
      error_x, error_y, error_yaw, error_distance);
  }

  void logError() {
    Pose2D gt;
    Pose2D amcl;
    Pose2D odom;
    double sigma_x;
    double sigma_y;
    double sigma_yaw;
    bool have_odom;
    bool estimate_from_tf = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_gt_) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for data: gz_odom=missing, amcl_pose=%s",
          have_amcl_ ? "OK" : "missing");
        return;
      }

      gt = latest_gt_;
      amcl = latest_amcl_;
      have_odom = have_odom_;
      odom = latest_odom_;

      if (!have_amcl_) {
        estimate_from_tf = lookupTfEstimate(amcl);
        if (!estimate_from_tf) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Waiting for data: gz_odom=OK, amcl_pose=missing, tf_estimate=missing");
          return;
        }
      }

      if (align_on_first_sample_ && !aligned_) {
        initial_gt_ = latest_gt_;
        initial_amcl_ = amcl;
        aligned_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Aligned first samples: gt=(%.3f, %.3f, %.3f), amcl=(%.3f, %.3f, %.3f)",
          initial_gt_.x, initial_gt_.y, initial_gt_.yaw,
          initial_amcl_.x, initial_amcl_.y, initial_amcl_.yaw);
      }

      sigma_x = sigma_x_;
      sigma_y = sigma_y_;
      sigma_yaw = sigma_yaw_;
    }

    const Pose2D gt_aligned = transformGroundTruthToAmclFrame(gt);
    const double error_x = amcl.x - gt_aligned.x;
    const double error_y = amcl.y - gt_aligned.y;
    const double error_yaw = normalizeAngle(amcl.yaw - gt_aligned.yaw);
    const double error_distance = std::hypot(error_x, error_y);

    geometry_msgs::msg::Vector3Stamped error_msg;
    error_msg.header.stamp = now();
    error_msg.header.frame_id = "map";
    error_msg.vector.x = error_x;
    error_msg.vector.y = error_y;
    error_msg.vector.z = error_yaw;
    error_pub_->publish(error_msg);

    double odom_error_x = 0.0;
    double odom_error_y = 0.0;
    double odom_error_yaw = 0.0;
    double odom_error_distance = 0.0;

    if (have_odom) {
      odom_error_x = odom.x - gt_aligned.x;
      odom_error_y = odom.y - gt_aligned.y;
      odom_error_yaw = normalizeAngle(odom.yaw - gt_aligned.yaw);
      odom_error_distance = std::hypot(odom_error_x, odom_error_y);

      geometry_msgs::msg::Vector3Stamped odom_error_msg;
      odom_error_msg.header.stamp = now();
      odom_error_msg.header.frame_id = "map";
      odom_error_msg.vector.x = odom_error_x;
      odom_error_msg.vector.y = odom_error_y;
      odom_error_msg.vector.z = odom_error_yaw;
      odom_error_pub_->publish(odom_error_msg);
    }

    RCLCPP_INFO(
      get_logger(),
      "gt=(%.3f, %.3f, %.3f) amcl=(%.3f, %.3f, %.3f) "
      "source=%s err=(x %.3f, y %.3f, yaw %.3f rad, dist %.3f m) "
      "odom_err=(x %.3f, y %.3f, yaw %.3f rad, dist %.3f m, %s) "
      "sigma=(x %.3f, y %.3f, yaw %.3f)",
      gt_aligned.x, gt_aligned.y, gt_aligned.yaw,
      amcl.x, amcl.y, amcl.yaw,
      estimate_from_tf ? "tf" : "amcl_pose",
      error_x, error_y, error_yaw, error_distance,
      odom_error_x, odom_error_y, odom_error_yaw, odom_error_distance,
      have_odom ? "OK" : "missing",
      sigma_x, sigma_y, sigma_yaw);

    if (csv_.is_open()) {
      csv_ << now().seconds() << ','
           << gt_aligned.x << ',' << gt_aligned.y << ',' << gt_aligned.yaw << ','
           << amcl.x << ',' << amcl.y << ',' << amcl.yaw << ','
           << error_x << ',' << error_y << ',' << error_yaw << ','
           << error_distance << ','
           << odom.x << ',' << odom.y << ',' << odom.yaw << ','
           << odom_error_x << ',' << odom_error_y << ',' << odom_error_yaw << ','
           << odom_error_distance << ','
           << sigma_x << ',' << sigma_y << ',' << sigma_yaw << '\n';
      csv_.flush();
    }
  }

  gz::transport::Node gz_node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr odom_error_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr initial_pose_error_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex mutex_;
  Pose2D latest_gt_;
  Pose2D latest_amcl_;
  Pose2D latest_odom_;
  Pose2D initial_gt_;
  Pose2D initial_amcl_;
  bool have_gt_{false};
  bool have_amcl_{false};
  bool have_odom_{false};
  bool aligned_{false};
  double sigma_x_{0.0};
  double sigma_y_{0.0};
  double sigma_yaw_{0.0};

  std::string gz_odom_topic_;
  std::string amcl_pose_topic_;
  std::string odom_topic_;
  std::string map_frame_id_;
  std::string base_frame_id_;
  std::string initial_pose_topic_;
  std::string error_topic_;
  std::string odom_error_topic_;
  std::string initial_pose_error_topic_;
  std::string csv_path_;
  bool align_on_first_sample_{true};
  bool use_tf_estimate_{true};
  bool use_fixed_world_to_map_{false};
  double map_from_world_x_{0.0};
  double map_from_world_y_{0.0};
  double map_from_world_yaw_{0.0};
  double log_rate_hz_{1.0};
  std::ofstream csv_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationErrorMonitor>());
  rclcpp::shutdown();
  return 0;
}
