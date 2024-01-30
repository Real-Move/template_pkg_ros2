# MIT License
#
# Copyright (c) 2024 Luca Fortini
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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Create a LaunchDescription object
    ld = LaunchDescription()

    # Add a log message to indicate the start of the launch
    ld.add_action(launch.actions.LogInfo(msg='Hello Welcome to the ros2_template_pkg!'))

    # Define a C++ node configuration
    cpp_node = Node(
        package='ros2_template_pkg',
        executable='ros2_template_node_cpp',
        name='ros2_template_node_cpp',
        exec_name='ros2_template_node_cpp',
        output='screen',
    )

    # Define a Python node configuration
    py_node = Node(
        package='ros2_template_pkg',
        executable='ros2_template_node_python.py',
        name='ros2_template_node_python',
        exec_name='ros2_template_node_python',
        output='screen',
    )

    include = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                FindPackageShare("ros2_template_pkg"), '/launch', '/include_launch.py']))

    # Add the C++ and Python nodes to the LaunchDescription
    ld.add_action(cpp_node)
    ld.add_action(py_node)
    ld.add_action(include)

    # Return the LaunchDescription object
    return ld
