from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    narval_wrench_system = FindPackageShare('narval_wrench_system')
    

    default_config_path = PathJoinSubstitution([narval_wrench_system, 'config', 'params.yaml'])
    ld.add_action(DeclareLaunchArgument(name="config", default_value=default_config_path, description="File name of configuration"))

    ld.add_action(Node(
        package='ds4_driver',
        executable='ds4_driver_node.py',
        output="screen"
    ))

    ld.add_action(Node(
        package="narval_wrench_system",
        executable='base_node',
        parameters=[LaunchConfiguration('config')],
        output="screen"
    ))

    return ld

