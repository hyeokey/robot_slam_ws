from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration("world")

    default_world = PathJoinSubstitution(
        [FindPackageShare("tugbot_description"), "worlds", "warehouse_vehicle.world"]
    )
    model_path = PathJoinSubstitution(
        [FindPackageShare("tugbot_description"), "models"]
    )
    state_pub_launch = PathJoinSubstitution(
        [FindPackageShare("tugbot_description"), "launch", "tugbot_state_publisher.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(state_pub_launch),
                launch_arguments={"use_sim_time": "true"}.items(),
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    model_path,
                    ":/home/dong/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models:/home/dong/.gz/fuel/fuel.ignitionrobotics.org/movai/models",
                ],
            ),
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=[
                    model_path,
                    ":/home/dong/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models:/home/dong/.gz/fuel/fuel.ignitionrobotics.org/movai/models",
                ],
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-r", "-v", "4", world],
                output="screen",
            ),
            Node(
                package="gz_relay_bridge",
                executable="gz_relay_bridge_node",
                name="gz_relay_bridge",
                output="screen",
                parameters=[
                    {
                        "ros_clock_topic": "/clock",
                        "ros_odom_topic": "/odom",
                        "ros_scan_topic": "/scan",
                        "ros_cmdvel_topic": "/unused_cmd_vel_for_bridge",
                        "odom_frame_id": "odom",
                        "base_frame_id": "base_footprint",
                        "scan_frame_id": "scan_omni",
                    }
                ],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="gz_relay_bridge",
                        executable="independent_steer_controller_node",
                        name="independent_steer_controller",
                        output="screen",
                        parameters=[
                            {
                                "model_name": "tugbot",
                                "odom_topic": "/odom",
                                "joint_state_topic": "/world/world_demo/model/tugbot/joint_state",
                                "drive_direction_sign": 1.0,
                                "cmd_timeout_sec": 0.8,
                                "feedback_enabled": False,
                                "steering_feedback_enabled": True,
                                "auto_spin_angular_threshold": 0.25,
                                "auto_spin_release_angular_threshold": 0.10,
                                "auto_spin_linear_threshold": 0.05,
                                "auto_spin_exit_linear_threshold": 0.08,
                                "auto_spin_max_angular_speed": 0.45,
                                "auto_spin_entry_duration_sec": 0.15,
                                "auto_spin_exit_duration_sec": 0.12,
                                "auto_mode_min_hold_sec": 0.25,
                                "steering_kp": 0.25,
                                "steering_kd": 0.01,
                                "max_steering_correction": 0.08,
                                "steering_drive_tolerance_rad": 0.12,
                                "require_steering_ready_for_drive": False,
                            }
                        ],
                    ),
                ],
            ),
        ]
    )
