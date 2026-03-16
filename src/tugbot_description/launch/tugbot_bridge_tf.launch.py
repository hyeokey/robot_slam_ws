from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    state_pub_launch = PathJoinSubstitution(
        [FindPackageShare("tugbot_description"), "launch", "tugbot_state_publisher.launch.py"]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(state_pub_launch),
                launch_arguments={"use_sim_time": "true"}.items(),
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
                        "ros_cmdvel_topic": "/cmd_vel",
                        "odom_frame_id": "odom",
                        "base_frame_id": "base_link",
                        "scan_frame_id": "scan_omni",
                    }
                ],
            ),
        ]
    )
