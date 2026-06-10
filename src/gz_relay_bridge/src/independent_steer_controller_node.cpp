#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <gz/msgs/double.pb.h>
#include <gz/msgs/model.pb.h>
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
    crab_steer_rate_rad_s_ = declare_parameter<double>("crab_steer_rate_rad_s", 1.5);
    crab_steer_tolerance_rad_ = declare_parameter<double>("crab_steer_tolerance_rad", 0.08);
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    feedback_enabled_ = declare_parameter<bool>("feedback_enabled", true);
    feedback_timeout_sec_ = declare_parameter<double>("feedback_timeout_sec", 0.5);
    feedback_cmd_threshold_ = declare_parameter<double>("feedback_cmd_threshold", 1e-3);
    linear_kp_ = declare_parameter<double>("linear_kp", 0.8);
    linear_ki_ = declare_parameter<double>("linear_ki", 0.0);
    linear_kd_ = declare_parameter<double>("linear_kd", 0.02);
    angular_kp_ = declare_parameter<double>("angular_kp", 0.8);
    angular_ki_ = declare_parameter<double>("angular_ki", 0.0);
    angular_kd_ = declare_parameter<double>("angular_kd", 0.02);
    linear_integral_limit_ = declare_parameter<double>("linear_integral_limit", 0.5);
    angular_integral_limit_ = declare_parameter<double>("angular_integral_limit", 1.0);
    max_linear_correction_ = declare_parameter<double>("max_linear_correction", 0.5);
    max_angular_correction_ = declare_parameter<double>("max_angular_correction", 1.0);
    joint_state_topic_ = declare_parameter<std::string>(
      "joint_state_topic", "/world/world_demo/model/tugbot/joint_state");
    steering_feedback_enabled_ = declare_parameter<bool>("steering_feedback_enabled", true);
    steering_feedback_timeout_sec_ = declare_parameter<double>(
      "steering_feedback_timeout_sec", 0.5);
    steering_kp_ = declare_parameter<double>("steering_kp", 0.7);
    steering_ki_ = declare_parameter<double>("steering_ki", 0.0);
    steering_kd_ = declare_parameter<double>("steering_kd", 0.02);
    steering_integral_limit_ = declare_parameter<double>("steering_integral_limit", 0.4);
    max_steering_correction_ = declare_parameter<double>("max_steering_correction", 0.35);
    steering_drive_tolerance_rad_ = declare_parameter<double>(
      "steering_drive_tolerance_rad", 0.08);
    require_steering_ready_for_drive_ = declare_parameter<bool>(
      "require_steering_ready_for_drive", false);
    auto_crab_enabled_ = declare_parameter<bool>("auto_crab_enabled", false);
    auto_spin_angular_threshold_ = declare_parameter<double>(
      "auto_spin_angular_threshold", 0.35);
    auto_spin_release_angular_threshold_ = declare_parameter<double>(
      "auto_spin_release_angular_threshold", 0.03);
    auto_spin_linear_threshold_ = declare_parameter<double>(
      "auto_spin_linear_threshold", 0.04);
    auto_spin_exit_linear_threshold_ = declare_parameter<double>(
      "auto_spin_exit_linear_threshold", 0.10);
    auto_spin_max_angular_speed_ = declare_parameter<double>(
      "auto_spin_max_angular_speed", 0.45);
    auto_spin_entry_duration_sec_ = declare_parameter<double>(
      "auto_spin_entry_duration_sec", 0.20);
    auto_spin_exit_duration_sec_ = declare_parameter<double>(
      "auto_spin_exit_duration_sec", 0.40);
    auto_mode_min_hold_sec_ = declare_parameter<double>(
      "auto_mode_min_hold_sec", 0.80);
    auto_crab_lateral_threshold_ = declare_parameter<double>(
      "auto_crab_lateral_threshold", 0.05);
    auto_crab_angular_threshold_ = declare_parameter<double>(
      "auto_crab_angular_threshold", 0.10);
    debug_enabled_ = declare_parameter<bool>("debug_enabled", true);
    debug_period_sec_ = declare_parameter<double>("debug_period_sec", 1.0);

    // Wheel centers in base_link frame [m], extracted from the vehicle model.
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
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&IndependentSteerController::onOdom, this, _1));
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/drive_mode", 10, std::bind(&IndependentSteerController::onDriveMode, this, _1));
    debug_pub_ = create_publisher<std_msgs::msg::String>(
      "/independent_steer_controller/debug", 10);
    const bool ok_joint_state =
      gz_node_.Subscribe(joint_state_topic_, &IndependentSteerController::onJointState, this);

    const auto period =
      std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&IndependentSteerController::publishCommands, this));

    last_cmd_time_ = now();
    target_cmd_ = geometry_msgs::msg::Twist();
    normalizeDriveMode();
    RCLCPP_INFO(get_logger(), "Drive mode: %s", drive_mode_.c_str());
    RCLCPP_INFO(
      get_logger(), "Velocity feedback: %s (odom=%s)",
      feedback_enabled_ ? "enabled" : "disabled", odom_topic_.c_str());
    RCLCPP_INFO(
      get_logger(), "Steering feedback: %s (joint_state=%s, sub=%s)",
      steering_feedback_enabled_ ? "enabled" : "disabled",
      joint_state_topic_.c_str(), ok_joint_state ? "OK" : "FAIL");
    RCLCPP_INFO(
      get_logger(),
      "Auto mode: crab=%s spin_threshold=%.3f rad/s spin_linear_threshold=%.3f m/s",
      auto_crab_enabled_ ? "enabled" : "disabled",
      auto_spin_angular_threshold_, auto_spin_linear_threshold_);
  }

private:
  struct WheelPosition {
    std::string name;
    double x;
    double y;
  };

  struct PidState {
    double integral{0.0};
    double previous_error{0.0};
    bool has_previous_error{false};
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

  static double crabSteerAngleForWheel(const std::string &wheel_name) {
    // Fixed crab steering:
    // front_right -> +94 deg (CCW from top view)
    // rear_left   -> +95 deg (CCW from top view)
    // front_left / rear_right -> -90 deg (CW from top view)
    if (wheel_name == "front_right") {
      return M_PI_2 + (4.0 * M_PI / 180.0);
    }
    if (wheel_name == "rear_left") {
      return M_PI_2 + (5.0 * M_PI / 180.0);
    }
    return -M_PI_2;
  }

  static double spinSteerAngleForWheel(const std::string &wheel_name) {
    const double angle_45 = M_PI / 4.0;
    if (wheel_name == "front_right" || wheel_name == "rear_left") {
      return angle_45;
    }
    return -angle_45;
  }

  static double crabSteerDirectionForWheel(const std::string &wheel_name) {
    // front_right / rear_left rotate CCW only, front_left / rear_right CW only.
    if (wheel_name == "front_right" || wheel_name == "rear_left") {
      return 1.0;
    }
    return -1.0;
  }

  static double clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(value, upper));
  }

  static void resetPid(PidState &state) {
    state.integral = 0.0;
    state.previous_error = 0.0;
    state.has_previous_error = false;
  }

  static double pidCorrection(
    double target, double actual, PidState &state, double kp, double ki, double kd,
    double integral_limit, double correction_limit, double dt)
  {
    const double error = target - actual;
    state.integral = clamp(
      state.integral + error * dt, -integral_limit, integral_limit);

    double derivative = 0.0;
    if (state.has_previous_error && dt > 1e-6) {
      derivative = (error - state.previous_error) / dt;
    }

    state.previous_error = error;
    state.has_previous_error = true;

    return clamp(
      kp * error + ki * state.integral + kd * derivative,
      -correction_limit, correction_limit);
  }

  static bool jointNameMatches(const std::string &state_name, const std::string &joint_name) {
    return state_name == joint_name ||
      (state_name.size() > joint_name.size() &&
       state_name.compare(state_name.size() - joint_name.size(), joint_name.size(), joint_name) == 0);
  }

  static double anglePidCorrection(
    double target_angle, double actual_angle, PidState &state, double kp, double ki, double kd,
    double integral_limit, double correction_limit, double dt)
  {
    const double error = normalizeAngle(target_angle - actual_angle);
    state.integral = clamp(
      state.integral + error * dt, -integral_limit, integral_limit);

    double derivative = 0.0;
    if (state.has_previous_error && dt > 1e-6) {
      derivative = normalizeAngle(error - state.previous_error) / dt;
    }

    state.previous_error = error;
    state.has_previous_error = true;

    return clamp(
      kp * error + ki * state.integral + kd * derivative,
      -correction_limit, correction_limit);
  }

  double stepCrabSteerAngle(std::size_t wheel_index, double target_angle, double direction_sign) {
    const double max_step =
      crab_steer_rate_rad_s_ / std::max(1.0, publish_rate_hz_);
    const double current_angle = commanded_steer_angles_[wheel_index];

    if (direction_sign > 0.0) {
      if (current_angle >= target_angle) {
        return target_angle;
      }
      return std::min(current_angle + max_step, target_angle);
    }

    if (current_angle <= target_angle) {
      return target_angle;
    }
    return std::max(current_angle - max_step, target_angle);
  }

  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    target_cmd_ = *msg;
    last_cmd_time_ = now();
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odom_twist_ = msg->twist.twist;
    last_odom_time_ = now();
    have_odom_ = true;
  }

  void onJointState(const gz::msgs::Model &msg) {
    for (int joint_index = 0; joint_index < msg.joint_size(); ++joint_index) {
      const auto &joint = msg.joint(joint_index);
      for (std::size_t i = 0; i < steer_joint_names_.size(); ++i) {
        if (jointNameMatches(joint.name(), steer_joint_names_[i]) && joint.has_axis1()) {
          actual_steer_angles_[i] = normalizeAngle(joint.axis1().position());
          have_steer_state_[i] = true;
        }
      }
    }
    last_joint_state_time_ = now();
  }

  void onDriveMode(const std_msgs::msg::String::SharedPtr msg) {
    drive_mode_ = msg->data;
    normalizeDriveMode();
    RCLCPP_INFO(get_logger(), "Switched drive mode to: %s", drive_mode_.c_str());
  }

  void normalizeDriveMode() {
    if (drive_mode_ == "autonomous") {
      drive_mode_ = "auto";
    }

    if (drive_mode_ != "normal" && drive_mode_ != "crab" &&
      drive_mode_ != "spin" && drive_mode_ != "auto")
    {
      RCLCPP_WARN(
        get_logger(), "Unknown drive mode '%s', falling back to 'normal'",
        drive_mode_.c_str());
      drive_mode_ = "normal";
    }
  }

  bool conditionHeld(
    bool condition,
    rclcpp::Time &condition_start_time,
    bool &condition_active,
    const rclcpp::Time &current_time,
    double required_duration_sec)
  {
    if (!condition) {
      condition_active = false;
      return false;
    }

    if (!condition_active) {
      condition_start_time = current_time;
      condition_active = true;
    }

    return (current_time - condition_start_time).seconds() >= required_duration_sec;
  }

  std::string selectAutoDriveMode(
    const rclcpp::Time &current_time,
    double vx,
    double vy,
    double wz,
    bool active_motion_cmd)
  {
    if (!active_motion_cmd) {
      auto_spin_entry_condition_active_ = false;
      auto_spin_exit_condition_active_ = false;
      return "normal";
    }

    const bool auto_mode_initialized = !last_auto_effective_mode_.empty();
    if (!auto_mode_initialized) {
      last_auto_effective_mode_ = "normal";
      last_auto_mode_switch_time_ = current_time;
    }

    const bool hold_active =
      auto_mode_initialized &&
      (current_time - last_auto_mode_switch_time_).seconds() < auto_mode_min_hold_sec_;
    const double linear_speed = std::hypot(vx, vy);
    const bool spin_entry_condition =
      std::fabs(wz) >= auto_spin_angular_threshold_ &&
      linear_speed <= auto_spin_linear_threshold_;
    const bool spin_exit_condition =
      std::fabs(wz) <= auto_spin_release_angular_threshold_ ||
      linear_speed >= auto_spin_exit_linear_threshold_;

    if (last_auto_effective_mode_ == "spin") {
      auto_spin_entry_condition_active_ = false;
      if (hold_active) {
        return "spin";
      }

      if (conditionHeld(
          spin_exit_condition, auto_spin_exit_condition_start_time_,
          auto_spin_exit_condition_active_, current_time, auto_spin_exit_duration_sec_))
      {
        auto_spin_exit_condition_active_ = false;
        return "normal";
      }

      return "spin";
    }

    auto_spin_exit_condition_active_ = false;
    if (!hold_active && conditionHeld(
        spin_entry_condition, auto_spin_entry_condition_start_time_,
        auto_spin_entry_condition_active_, current_time, auto_spin_entry_duration_sec_))
    {
      auto_spin_entry_condition_active_ = false;
      return "spin";
    }

    if (auto_crab_enabled_ &&
      std::fabs(vy) >= auto_crab_lateral_threshold_ &&
      std::fabs(vx) <= auto_spin_linear_threshold_ &&
      std::fabs(wz) <= auto_crab_angular_threshold_)
    {
      return "crab";
    }

    return "normal";
  }

  void publishDebugState(
    const rclcpp::Time &current_time,
    const std::string &effective_drive_mode,
    bool cmd_recent,
    bool active_motion_cmd,
    bool have_recent_odom,
    bool have_recent_steering_state,
    double vx,
    double vy,
    double wz,
    double odom_vx,
    double odom_wz,
    double first_steer_target,
    double first_wheel_speed)
  {
    if (!debug_enabled_) {
      return;
    }

    if ((current_time - last_debug_time_).seconds() < debug_period_sec_) {
      return;
    }
    last_debug_time_ = current_time;

    char buffer[512];
    std::snprintf(
      buffer, sizeof(buffer),
      "cmd(vx=%.3f,vy=%.3f,wz=%.3f) odom(vx=%.3f,wz=%.3f) mode=%s/%s recent(cmd=%s,odom=%s,steer=%s) active=%s steer=%.3f wheel=%.3f",
      vx, vy, wz, odom_vx, odom_wz,
      drive_mode_.c_str(), effective_drive_mode.c_str(),
      cmd_recent ? "true" : "false",
      have_recent_odom ? "true" : "false",
      have_recent_steering_state ? "true" : "false",
      active_motion_cmd ? "true" : "false",
      first_steer_target, first_wheel_speed);

    std_msgs::msg::String debug_msg;
    debug_msg.data = buffer;
    debug_pub_->publish(debug_msg);

    RCLCPP_INFO(get_logger(), "%s", debug_msg.data.c_str());
  }

  bool haveRecentSteeringState(const rclcpp::Time &current_time) const {
    if (!steering_feedback_enabled_) {
      return false;
    }
    for (const auto have_state : have_steer_state_) {
      if (!have_state) {
        return false;
      }
    }
    if ((current_time - last_joint_state_time_).seconds() > steering_feedback_timeout_sec_) {
      return false;
    }
    return true;
  }

  double correctedSteerCommand(
    std::size_t wheel_index, double target_angle, double dt, bool have_recent_steering_state)
  {
    if (!have_recent_steering_state) {
      resetPid(steer_pids_[wheel_index]);
      return target_angle;
    }

    const double correction = anglePidCorrection(
      target_angle, actual_steer_angles_[wheel_index],
      steer_pids_[wheel_index], steering_kp_, steering_ki_, steering_kd_,
      steering_integral_limit_, max_steering_correction_, dt);

    return normalizeAngle(target_angle + correction);
  }

  bool steeringReady(
    std::size_t wheel_index, double target_angle, bool have_recent_steering_state) const
  {
    if (!have_recent_steering_state) {
      return true;
    }
    const double error = normalizeAngle(target_angle - actual_steer_angles_[wheel_index]);
    return std::fabs(error) <= steering_drive_tolerance_rad_;
  }

  bool canDriveWheel(
    std::size_t wheel_index, double target_angle, bool have_recent_steering_state) const
  {
    return !require_steering_ready_for_drive_ ||
      steeringReady(wheel_index, target_angle, have_recent_steering_state);
  }

  void publishCommands() {
    const auto current_time = now();
    const auto age = (current_time - last_cmd_time_).seconds();

    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
    const bool cmd_recent = age <= cmd_timeout_sec_;
    if (cmd_recent) {
      vx = target_cmd_.linear.x;
      vy = target_cmd_.linear.y;
      wz = target_cmd_.angular.z;
    }

    const bool cmd_nonzero =
      std::hypot(target_cmd_.linear.x, target_cmd_.linear.y) > feedback_cmd_threshold_ ||
      std::fabs(target_cmd_.angular.z) > feedback_cmd_threshold_;
    const bool active_motion_cmd = cmd_recent && cmd_nonzero;

    double dt = 1.0 / std::max(1.0, publish_rate_hz_);
    if (have_last_control_time_) {
      dt = (current_time - last_control_time_).seconds();
      dt = clamp(dt, 1e-3, 0.1);
    }
    last_control_time_ = current_time;
    have_last_control_time_ = true;

    const bool have_recent_odom =
      have_odom_ && (current_time - last_odom_time_).seconds() <= feedback_timeout_sec_;

    if (feedback_enabled_ && cmd_recent && cmd_nonzero && have_recent_odom) {
      if (std::fabs(target_cmd_.linear.x) > feedback_cmd_threshold_) {
        vx += pidCorrection(
          vx, latest_odom_twist_.linear.x, vx_pid_,
          linear_kp_, linear_ki_, linear_kd_,
          linear_integral_limit_, max_linear_correction_, dt);
      } else {
        resetPid(vx_pid_);
      }

      if (std::fabs(target_cmd_.linear.y) > feedback_cmd_threshold_) {
        vy += pidCorrection(
          vy, latest_odom_twist_.linear.y, vy_pid_,
          linear_kp_, linear_ki_, linear_kd_,
          linear_integral_limit_, max_linear_correction_, dt);
      } else {
        resetPid(vy_pid_);
      }

      if (std::fabs(target_cmd_.angular.z) > feedback_cmd_threshold_) {
        wz += pidCorrection(
          wz, latest_odom_twist_.angular.z, wz_pid_,
          angular_kp_, angular_ki_, angular_kd_,
          angular_integral_limit_, max_angular_correction_, dt);
      } else {
        resetPid(wz_pid_);
      }
    } else {
      resetPid(vx_pid_);
      resetPid(vy_pid_);
      resetPid(wz_pid_);
    }

    std::string effective_drive_mode = drive_mode_;
    if (drive_mode_ == "auto") {
      effective_drive_mode = selectAutoDriveMode(current_time, vx, vy, wz, active_motion_cmd);
      if (effective_drive_mode != last_auto_effective_mode_) {
        RCLCPP_INFO(
          get_logger(), "Auto mode selected: %s", effective_drive_mode.c_str());
        last_auto_effective_mode_ = effective_drive_mode;
        last_auto_mode_switch_time_ = current_time;
      }

      if (effective_drive_mode != "crab") {
        vy = 0.0;
      }
    }

    if (effective_drive_mode == "crab") {
      wz = 0.0;
    } else if (effective_drive_mode == "spin") {
      vx = 0.0;
      vy = 0.0;
      wz = clamp(wz, -auto_spin_max_angular_speed_, auto_spin_max_angular_speed_);
    }

    const bool have_recent_steering_state = haveRecentSteeringState(current_time);
    double first_steer_target = 0.0;
    double first_wheel_speed = 0.0;

    for (std::size_t i = 0; i < wheel_positions_.size(); ++i) {
      const auto &wheel = wheel_positions_[i];

      double vix = 0.0;
      double viy = 0.0;

      if (effective_drive_mode == "crab") {
        vix = vx;
        viy = vy;
      } else {
        vix = vx - wz * wheel.y;
        viy = vy + wz * wheel.x;
      }

      const double linear_speed = std::hypot(vix, viy);

      double steer_target_angle = commanded_steer_angles_[i];
      double wheel_angular_speed = 0.0;

      if (effective_drive_mode == "crab" && active_motion_cmd && linear_speed > min_speed_threshold_) {
        const double crab_target_angle = crabSteerAngleForWheel(wheel.name);
        steer_target_angle = stepCrabSteerAngle(
          i, crab_target_angle, crabSteerDirectionForWheel(wheel.name));
        double crab_error = normalizeAngle(crab_target_angle - steer_target_angle);

        if (std::fabs(crab_error) <= crab_steer_tolerance_rad_ &&
          canDriveWheel(i, steer_target_angle, have_recent_steering_state))
        {
          wheel_angular_speed = drive_direction_sign_ * (linear_speed / wheel_radius_);

          // Reverse wheel spin when commanding motion to the robot's right so the
          // fixed 90 degree steering still produces left/right crab motion.
          if (viy < 0.0) {
            wheel_angular_speed *= -1.0;
          }
        }
      } else if (effective_drive_mode == "spin" && active_motion_cmd && linear_speed > min_speed_threshold_) {
        steer_target_angle = spinSteerAngleForWheel(wheel.name);

        if (canDriveWheel(i, steer_target_angle, have_recent_steering_state)) {
          const double heading_x = std::cos(steer_target_angle);
          const double heading_y = std::sin(steer_target_angle);
          const double projected_speed = vix * heading_x + viy * heading_y;
          wheel_angular_speed = drive_direction_sign_ * (projected_speed / wheel_radius_);
        }
      } else if (active_motion_cmd && linear_speed > min_speed_threshold_) {
        steer_target_angle = normalizeAngle(std::atan2(viy, vix));
        wheel_angular_speed = drive_direction_sign_ * (linear_speed / wheel_radius_);

        // Prefer reversing the wheel over large steering excursions.
        if (steer_target_angle > M_PI_2) {
          steer_target_angle -= M_PI;
          wheel_angular_speed *= -1.0;
        } else if (steer_target_angle < -M_PI_2) {
          steer_target_angle += M_PI;
          wheel_angular_speed *= -1.0;
        }

        if (!canDriveWheel(i, steer_target_angle, have_recent_steering_state)) {
          wheel_angular_speed = 0.0;
        }
      }

      double steer_command_angle = steer_target_angle;
      if (active_motion_cmd) {
        steer_command_angle =
          correctedSteerCommand(i, steer_target_angle, dt, have_recent_steering_state);
      } else {
        resetPid(steer_pids_[i]);
      }

      gz::msgs::Double steer_msg;
      steer_msg.set_data(steer_command_angle);
      steer_pubs_[i].Publish(steer_msg);
      commanded_steer_angles_[i] = clamp(steer_target_angle, -M_PI, M_PI);

      gz::msgs::Double wheel_msg;
      wheel_msg.set_data(wheel_angular_speed);
      wheel_pubs_[i].Publish(wheel_msg);

      if (i == 0) {
        first_steer_target = steer_target_angle;
        first_wheel_speed = wheel_angular_speed;
      }
    }

    publishDebugState(
      current_time, effective_drive_mode, cmd_recent, active_motion_cmd,
      have_recent_odom, have_recent_steering_state, vx, vy, wz,
      latest_odom_twist_.linear.x, latest_odom_twist_.angular.z,
      first_steer_target, first_wheel_speed);
  }

  std::string model_name_;
  std::string drive_mode_{"normal"};
  double cmd_timeout_sec_{0.3};
  double wheel_radius_{0.103};
  double publish_rate_hz_{50.0};
  double min_speed_threshold_{1e-3};
  double drive_direction_sign_{-1.0};
  double crab_steer_rate_rad_s_{1.5};
  double crab_steer_tolerance_rad_{0.08};
  std::string odom_topic_{"/odom"};
  bool feedback_enabled_{true};
  double feedback_timeout_sec_{0.5};
  double feedback_cmd_threshold_{1e-3};
  double linear_kp_{0.8};
  double linear_ki_{0.0};
  double linear_kd_{0.02};
  double angular_kp_{0.8};
  double angular_ki_{0.0};
  double angular_kd_{0.02};
  double linear_integral_limit_{0.5};
  double angular_integral_limit_{1.0};
  double max_linear_correction_{0.5};
  double max_angular_correction_{1.0};
  std::string joint_state_topic_{"/world/world_demo/model/tugbot/joint_state"};
  bool steering_feedback_enabled_{true};
  double steering_feedback_timeout_sec_{0.5};
  double steering_kp_{0.7};
  double steering_ki_{0.0};
  double steering_kd_{0.02};
  double steering_integral_limit_{0.4};
  double max_steering_correction_{0.35};
  double steering_drive_tolerance_rad_{0.08};
  bool require_steering_ready_for_drive_{false};
  bool auto_crab_enabled_{false};
  double auto_spin_angular_threshold_{0.35};
  double auto_spin_release_angular_threshold_{0.03};
  double auto_spin_linear_threshold_{0.04};
  double auto_spin_exit_linear_threshold_{0.10};
  double auto_spin_max_angular_speed_{0.45};
  double auto_spin_entry_duration_sec_{0.20};
  double auto_spin_exit_duration_sec_{0.40};
  double auto_mode_min_hold_sec_{0.80};
  double auto_crab_lateral_threshold_{0.05};
  double auto_crab_angular_threshold_{0.10};
  std::string last_auto_effective_mode_;
  rclcpp::Time last_auto_mode_switch_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time auto_spin_entry_condition_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time auto_spin_exit_condition_start_time_{0, 0, RCL_ROS_TIME};
  bool auto_spin_entry_condition_active_{false};
  bool auto_spin_exit_condition_active_{false};
  bool debug_enabled_{true};
  double debug_period_sec_{1.0};

  geometry_msgs::msg::Twist target_cmd_;
  geometry_msgs::msg::Twist latest_odom_twist_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_joint_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_debug_time_{0, 0, RCL_ROS_TIME};
  bool have_odom_{false};
  bool have_last_control_time_{false};
  PidState vx_pid_;
  PidState vy_pid_;
  PidState wz_pid_;
  std::array<PidState, 4> steer_pids_;

  gz::transport::Node gz_node_;
  std::array<gz::transport::Node::Publisher, 4> steer_pubs_;
  std::array<gz::transport::Node::Publisher, 4> wheel_pubs_;
  std::array<double, 4> commanded_steer_angles_{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> actual_steer_angles_{{0.0, 0.0, 0.0, 0.0}};
  std::array<bool, 4> have_steer_state_{{false, false, false, false}};
  std::array<WheelPosition, 4> wheel_positions_;
  std::array<std::string, 4> steer_joint_names_;
  std::array<std::string, 4> wheel_joint_names_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IndependentSteerController>());
  rclcpp::shutdown();
  return 0;
}
