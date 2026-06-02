from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration("world")

    default_world = PathJoinSubstitution(
        [FindPackageShare("tugbot_description"), "worlds", "warehouse_vehicle.world"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value="/home/dong/Desktop:/home/dong/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models:/home/dong/.gz/fuel/fuel.ignitionrobotics.org/movai/models",
            ),
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value="/home/dong/Desktop:/home/dong/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models:/home/dong/.gz/fuel/fuel.ignitionrobotics.org/movai/models",
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
                                "feedback_enabled": True,
                                "steering_feedback_enabled": True,
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
