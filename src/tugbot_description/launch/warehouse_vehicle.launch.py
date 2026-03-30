from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
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
                executable="independent_steer_controller_node",
                name="independent_steer_controller",
                output="screen",
                parameters=[{"model_name": "tugbot"}],
            ),
        ]
    )
