# Copyright (c) 2024 Real-Move. All rights reserved.
# Proprietary and confidential.
# See LICENSE for full terms.

import launch


def generate_launch_description():

    ld = launch.LaunchDescription()
    ld.add_action(launch.actions.LogInfo(msg="Hi from example include launch!"))

    return ld
