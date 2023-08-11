#!/bin/bash
# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: CC0-1.0

export ROS_DOMAIN_ID=1
export FASTRTPS_DEFAULT_PROFILES_FILE=~/CLIENT_FASTRTPS_PROFILES.xml
source /opt/ros/eloquent/setup.bash
source ~/workspace/install/local_setup.bash
cd /home/pld/workspace
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 antenna_eta rail_horizon_eta &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 antenna_etb rail_horizon_etb
