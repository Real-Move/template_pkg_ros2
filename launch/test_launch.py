from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_template_pkg',
            executable='cpp_executable',
            name='cpp_executable',
            output='screen',
        ),
        Node(
            package='ros2_template_pkg',
            executable='py_node.py',
            name='py_node.py',
            output='screen',
        ),
    ])