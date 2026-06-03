from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")

    tugbot_share = FindPackageShare("tugbot_description")
    nav2_share = FindPackageShare("nav2_bringup")

    nav2_params = PathJoinSubstitution(
        [tugbot_share, "params", "nav2_params.yaml"]
    )
    nav2_launch = PathJoinSubstitution([nav2_share, "launch", "bringup_launch.py"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("map", default_value="/home/dong/robot_ws/my_map.yaml"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "slam": "False",
                    "map": map_yaml,
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params,
                    "autostart": "true",
                    "use_composition": "False",
                }.items(),
            ),
        ]
    )
