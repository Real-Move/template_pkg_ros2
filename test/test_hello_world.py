# Copyright (c) 2024 Real-Move. All rights reserved.
# Proprietary and confidential.
# See LICENSE for full terms.

import importlib.util
import sys
import types
from pathlib import Path


def test_hello_world_example_logs_once_and_exits():
    script_path = (
        Path(__file__).resolve().parents[1] / "examples" / "python" / "hello_world_node.py"
    )

    events = []

    class FakeLogger:
        def info(self, message):
            events.append(("info", message))

    class FakeNode:
        def __init__(self, name):
            events.append(("node", name))
            self._logger = FakeLogger()

        def get_logger(self):
            return self._logger

        def destroy_node(self):
            events.append(("destroy", None))

    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.init = lambda args=None: events.append(("init", args))
    fake_rclpy.shutdown = lambda: events.append(("shutdown", None))
    fake_node_module = types.ModuleType("rclpy.node")
    fake_node_module.Node = FakeNode

    sys.modules["rclpy"] = fake_rclpy
    sys.modules["rclpy.node"] = fake_node_module

    spec = importlib.util.spec_from_file_location("hello_world_node", script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    module.main()

    assert events == [
        ("init", None),
        ("node", "hello_world_node"),
        ("info", "Hello, world!"),
        ("destroy", None),
        ("shutdown", None),
    ]
