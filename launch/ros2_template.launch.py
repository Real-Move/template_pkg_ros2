# MIT License
#
# Copyright (c) 2024 Real-Move
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)

def generate_launch_description():
    # Create a LaunchDescription object
    ld = LaunchDescription()

    # Add a log message to indicate the start of the launch
    ld.add_action(launch.actions.LogInfo(msg='Hello Welcome to the template_pkg_ros2!'))

    log_level_arg_name = "log_level"
    log_level_arg = DeclareLaunchArgument(
        name=log_level_arg_name,
        default_value="INFO",
        choices=['DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'],
        description="Logging level for the node. Set \'DEBUG\', \'INFO\', \'WARN\', ect."
    )

    # Define a C++ node configuration
    cpp_node = Node(
        package='template_pkg_ros2',
        executable='ros2_template_node_cpp',
        name='ros2_template_node_cpp',
        exec_name='ros2_template_node_cpp',
        output='screen',
        arguments=["--ros-args", "--log-level", PythonExpression([
                "'ros2_template_node_cpp:=' + '", LaunchConfiguration("log_level"), "'"
            ])],
    )

    # Define a Python node configuration
    py_node = Node(
        package='template_pkg_ros2',
        executable='ros2_template_node_python.py',
        name='ros2_template_node_python',
        exec_name='ros2_template_node_python',
        output='screen',
        arguments=["--ros-args", "--log-level", PythonExpression([
                "'ros2_template_node_python:=' + '", LaunchConfiguration("log_level"), "'"
            ])],
    )

    logging_demo_node = Node(
        package='template_pkg_ros2',
        executable='logging_demo_node',
        name='logging_demo_node',
        exec_name='logging_demo_node',
        output='screen',
        arguments=["--ros-args", "--log-level", PythonExpression([
                "'logging_demo_node:=' + '", LaunchConfiguration("log_level"), "'"
            ])],
    )

    include = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                FindPackageShare("template_pkg_ros2"), '/launch', '/include.launch.py']))

    args = [
        log_level_arg,
    ]

    launch_includes = [
        include,
    ]

    nodes = [
        cpp_node,
        py_node,
        logging_demo_node
    ]

    for action in args + launch_includes + nodes:
        ld.add_action(action)

    # Return the LaunchDescription object
    return ld
