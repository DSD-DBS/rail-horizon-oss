# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: CC0-1.0

# This script sources and runs mission profile
#Humble
source /opt/ros/humble/setup.bash && source ./install/local_setup.bash
#mission_profile_topic
ros2 run dsd_mission_profile dsd_mission_profile --ros-args --params-file ./install/dsd_mission_profile/config/dsd_mission_profile/parameters1.yaml
