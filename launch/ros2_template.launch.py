# Copyright (c) 2024 Real-Move. All rights reserved.
# Proprietary and confidential.
# See LICENSE for full terms.

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription(
        [
            LogInfo(msg="Starting template_pkg_ros2 example launch."),
            DeclareLaunchArgument(
                "log_level",
                default_value="DEBUG",
                choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
                description="Logging level for the example nodes.",
            ),
            Node(
                package="template_pkg_ros2",
                executable="minimal_cpp_node",
                name="minimal_cpp_node",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="template_pkg_ros2",
                executable="minimal_py_node.py",
                name="minimal_py_node",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="template_pkg_ros2",
                executable="logging_demo_node",
                name="logging_demo_node",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("template_pkg_ros2"), "/launch", "/example_include.launch.py"]
                )
            ),
        ]
    )
