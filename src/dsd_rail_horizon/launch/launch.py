# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix

import os


def generate_launch_description():
    config = os.path.join(get_package_prefix('dsd_rail_horizon'), 'config',
                          'dsd_rail_horizon'
                          'parameters.yaml')

    print(config)

    return LaunchDescription([
        Node(package='dsd_rail_horizon',
             executable='dsd_rail_horizon',
             parameters=[config])
    ])
