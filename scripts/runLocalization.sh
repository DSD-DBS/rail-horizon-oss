# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: CC0-1.0

# This script sources and runs coupled localization.
# Humble
source /opt/ros/humble/setup.bash && source ./install/local_setup.bash
#coupled_localization_topic
ros2 topic pub "/coupled_localization" "dsd_ros_messages/msg/CoupledLocalizationStamped"
