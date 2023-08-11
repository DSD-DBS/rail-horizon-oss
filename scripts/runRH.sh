# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: CC0-1.0

# This script sources and runs rail horizon
#Humble
source /opt/ros/humble/setup.bash && source ./install/local_setup.bash
#rail_horizon_topic
ros2 run dsd_rail_horizon dsd_rail_horizon --ros-args --params-file ./install/dsd_rail_horizon/config/dsd_rail_horizon/parameters.yaml
