# Copyright (c) 2024 Real-Move. All rights reserved.
# Proprietary and confidential.
# See LICENSE for full terms.

from template_pkg_ros2.minimal_py_node import main


def test_main_prints_expected_message(capsys):
    main()
    captured = capsys.readouterr()
    assert captured.out.strip() == "Hello from template_pkg_ros2 minimal Python node."
