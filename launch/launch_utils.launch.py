# Copyright (c) 2024 Real-Move. All rights reserved.
# Proprietary and confidential.
# See LICENSE for full terms.

"""Example: ros2 launch template_pkg_ros2 test.launch.py log_level:=DEBUG"""

from ros2_launch_utils import (
    add_launch_arg,
    add_launch_file,
    add_launch_params,
    add_log_msg,
    add_node,
    launch_file,
)

configurable_parameters = [
    {
        "name": "my_int",
        "default": 10,
        "description": "Example integer printed by the minimal C++ node.",
        "value_type": int,
    },
    {
        "name": "my_topic",
        "default": "hello",
        "description": "Example topic name printed by the node (no publisher is created).",
        "value_type": str,
    },
]


@launch_file
def launch():

    log_level = add_launch_arg(
        "log_level",
        default="INFO",
        description="Logging verbosity for the minimal C++ node.",
        choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
    )

    minimal_cpp_node_params = add_launch_params(configurable_parameters)

    add_log_msg(msg="Starting template_pkg_ros2 example launch.", level="INFO")

    add_node(
        package="template_pkg_ros2",
        executable="minimal_cpp_node",
        name="minimal_cpp_node",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[minimal_cpp_node_params],
    )

    add_launch_file(
        package="template_pkg_ros2",
        launch_file="example_include.launch.py",
        argv={"log_level": log_level},
    )
