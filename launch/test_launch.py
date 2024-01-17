from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='template_pkg_ros2',
            executable='cpp_executable',
            name='cpp_executable',
            output='screen',
        ),
        Node(
            package='template_pkg_ros2',
            executable='py_node.py',
            name='py_node.py',
            output='screen',
        ),
    ])