from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    tugbot_share = FindPackageShare("tugbot_description")
    slam_share = FindPackageShare("slam_toolbox")

    bridge_launch = PathJoinSubstitution(
        [tugbot_share, "launch", "tugbot_bridge_tf.launch.py"]
    )
    slam_launch = PathJoinSubstitution(
        [slam_share, "launch", "online_async_launch.py"]
    )
    slam_params = PathJoinSubstitution(
        [tugbot_share, "params", "slam_toolbox_params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bridge_launch),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "slam_params_file": slam_params,
                }.items(),
            ),
        ]
    )
