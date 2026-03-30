#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <gz/msgs/double.pb.h>
#include <gz/transport/Node.hh>

class IndependentSteerController : public rclcpp::Node {
public:
  IndependentSteerController() : Node("independent_steer_controller") {
    using std::placeholders::_1;

    model_name_ = declare_parameter<std::string>("model_name", "tugbot");
    drive_mode_ = declare_parameter<std::string>("drive_mode", "normal");
    cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.3);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.103);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    min_speed_threshold_ = declare_parameter<double>("min_speed_threshold", 1e-3);
    drive_direction_sign_ = declare_parameter<double>("drive_direction_sign", -1.0);

    // Wheel centers in base_link frame [m], extracted from /home/dong/Desktop/sdf
    wheel_positions_ = {{
      {"front_right",  0.558255699202, -0.500000000000},
      {"front_left",   0.558243305372,  0.500094888084},
      {"rear_right",  -0.248243394788, -0.500000000000},
      {"rear_left",   -0.247531564263,  0.500009437666},
    }};

    steer_joint_names_ = {
      "front_right_steer_joint",
      "front_left_steer_joint",
      "rear_right_steer_joint",
      "rear_left_steer_joint",
    };
    wheel_joint_names_ = {
      "front_right_wheel_joint",
      "front_left_wheel_joint",
      "rear_right_wheel_joint",
      "rear_left_wheel_joint",
    };

    for (std::size_t i = 0; i < steer_joint_names_.size(); ++i) {
      const auto steer_topic =
        "/model/" + model_name_ + "/joint/" + steer_joint_names_[i] + "/0/cmd_pos";
      const auto wheel_topic =
        "/model/" + model_name_ + "/joint/" + wheel_joint_names_[i] + "/cmd_vel";

      steer_pubs_[i] = gz_node_.Advertise<gz::msgs::Double>(steer_topic);
      wheel_pubs_[i] = gz_node_.Advertise<gz::msgs::Double>(wheel_topic);

      RCLCPP_INFO(
        get_logger(), "Joint topics: steer=%s wheel=%s",
        steer_topic.c_str(), wheel_topic.c_str());
    }

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&IndependentSteerController::onCmdVel, this, _1));
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/drive_mode", 10, std::bind(&IndependentSteerController::onDriveMode, this, _1));

    const auto period =
      std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&IndependentSteerController::publishCommands, this));

    last_cmd_time_ = now();
    target_cmd_ = geometry_msgs::msg::Twist();
    normalizeDriveMode();
    RCLCPP_INFO(get_logger(), "Drive mode: %s", drive_mode_.c_str());
  }

private:
  struct WheelPosition {
    std::string name;
    double x;
    double y;
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

  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    target_cmd_ = *msg;
    last_cmd_time_ = now();
  }

  void onDriveMode(const std_msgs::msg::String::SharedPtr msg) {
    drive_mode_ = msg->data;
    normalizeDriveMode();
    RCLCPP_INFO(get_logger(), "Switched drive mode to: %s", drive_mode_.c_str());
  }

  void normalizeDriveMode() {
    if (drive_mode_ != "normal" && drive_mode_ != "crab" && drive_mode_ != "spin") {
      RCLCPP_WARN(
        get_logger(), "Unknown drive mode '%s', falling back to 'normal'",
        drive_mode_.c_str());
      drive_mode_ = "normal";
    }
  }

  void publishCommands() {
    const auto age = (now() - last_cmd_time_).seconds();

    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
    if (age <= cmd_timeout_sec_) {
      vx = target_cmd_.linear.x;
      vy = target_cmd_.linear.y;
      wz = target_cmd_.angular.z;
    }

    if (drive_mode_ == "crab") {
      wz = 0.0;
    } else if (drive_mode_ == "spin") {
      vx = 0.0;
      vy = 0.0;
    }

    for (std::size_t i = 0; i < wheel_positions_.size(); ++i) {
      const auto &wheel = wheel_positions_[i];

      double vix = 0.0;
      double viy = 0.0;

      if (drive_mode_ == "crab") {
        vix = vx;
        viy = vy;
      } else {
        vix = vx - wz * wheel.y;
        viy = vy + wz * wheel.x;
      }

      const double linear_speed = std::hypot(vix, viy);

      double steer_angle = 0.0;
      double wheel_angular_speed = 0.0;

      if (linear_speed > min_speed_threshold_) {
        steer_angle = normalizeAngle(std::atan2(viy, vix));
        wheel_angular_speed = drive_direction_sign_ * (linear_speed / wheel_radius_);

        // Prefer reversing the wheel over large steering excursions.
        if (steer_angle > M_PI_2) {
          steer_angle -= M_PI;
          wheel_angular_speed *= -1.0;
        } else if (steer_angle < -M_PI_2) {
          steer_angle += M_PI;
          wheel_angular_speed *= -1.0;
        }
      }

      gz::msgs::Double steer_msg;
      steer_msg.set_data(steer_angle);
      steer_pubs_[i].Publish(steer_msg);

      gz::msgs::Double wheel_msg;
        wheel_msg.set_data(wheel_angular_speed);
      wheel_pubs_[i].Publish(wheel_msg);
    }
  }

  std::string model_name_;
  std::string drive_mode_{"normal"};
  double cmd_timeout_sec_{0.3};
  double wheel_radius_{0.103};
  double publish_rate_hz_{50.0};
  double min_speed_threshold_{1e-3};
  double drive_direction_sign_{-1.0};

  geometry_msgs::msg::Twist target_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_SYSTEM_TIME};

  gz::transport::Node gz_node_;
  std::array<gz::transport::Node::Publisher, 4> steer_pubs_;
  std::array<gz::transport::Node::Publisher, 4> wheel_pubs_;
  std::array<WheelPosition, 4> wheel_positions_;
  std::array<std::string, 4> steer_joint_names_;
  std::array<std::string, 4> wheel_joint_names_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IndependentSteerController>());
  rclcpp::shutdown();
  return 0;
}
