/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_PARAMETERS_H
#define DSD_RAIL_HORIZON_PARAMETERS_H

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

/**
 * @brief ROS Configuration Parameters
 */
class RailHorizonParameters
{
public:
    /**
     * @brief Constructs new Rail Horizon Parameters object
     * @param parent_node passing parent node where the object will be created and used
     */
    RailHorizonParameters(rclcpp::Node& parent_node);

    /**
     * @brief Namespace for all rail horizon topics
     */
    std::string rail_horizon_namespace_;
    /**
     * @brief Topic on which RailHorizonStamped.msg is published
     */
    std::string rail_horizon_topic_;
    /**
     * @brief Frame ID is published
     */
    std::string rail_horizon_frame_id_;
    /**
     * @brief Reference frame for the markers (relevant for rviz)
     */
    std::string rail_horizon_marker_reference_frame_;
    /**
     * @brief Position marker for the actual train position (relevant for rviz)
     */
    std::string rail_horizon_marker_position_topic_;
    /**
     * @brief Topic on which vertical structure markers are published (relevant for rviz)
     */
    std::string rail_horizon_marker_vertical_structure_topic_;
    /**
     * @brief Topic on which horizontal structure markers are published (relevant for rviz)
     */
    std::string rail_horizon_marker_horizontal_structure_topic_;
    /**
     * @brief Topic on which plane structure markers are published (relevant for rviz)
     */
    std::string rail_horizon_marker_plane_structure_topic_;
    /**
     * @brief Topic on which body structure markers are published (relevant for rviz)
     */
    std::string rail_horizon_marker_body_structure_topic_;
    /**
     * @brief Frequency at which RailHorizonStampled.msg is published
     */
    int rail_horizon_topic_frequency_{};
    /**
     * @brief Whether global horizon is filled in the message
     */
    bool rail_horizon_global_horizon_{};
    /**
     * @brief Whether local horizon is filled in the message
     */
    bool rail_horizon_local_horizon_{};
    /**
     * @brief Whether rotation of the local horizon is applied
     */
    bool rail_horizon_local_horizon_apply_rotations_{};
    /**
     * @brief Whether markers are published (relevant for rviz)
     */
    bool rail_horizon_markers_{};
    /**
     * @brief Whether virtual centerline is filled in the message
     */
    bool rail_horizon_virtual_centerline_{};
    /**
     * @brief Whether left and right rail are filled in the message
     */
    bool rail_horizon_left_right_rail_{};
    /**
     * @brief Topic on which ApplicationConfigurationStamped.msg is received
     */
    std::string application_configuration_topic_;
    /**
     * @brief Topic on which ApplicationStatusStamped.msg is received
     */
    std::string application_status_topic_;
    /**
     * @brief Whether mission control supervision is active
     */
    bool mission_control_supervision_{};
    /**
     * @brief Application ID (relevant if mission control supervision is active)
     */
    std::string mission_control_app_id_;
    /**
     * @brief Node ID (relevant if mission control supervision is active)
     */
    std::string mission_control_node_id_;
    /**
     * @brief Whether mission profile is active
     */
    bool mission_profile_apply_{};
    /**
     * @brief Topic on which MissionProfileStamped.msg is received
     */
    std::string mission_profile_topic_;
    /**
     * @brief Topic on which CoupledLocalizationStamped.msg is received
     */
    std::string coupled_localization_topic_;
    /**
     * @brief Whether Coupled Localization uses geoid
     */
    bool coupled_localization_use_geoid_{};
    /**
     * @brief Whether GNSS position is spoofed (true for static poistion)
     */
    bool spoof_gnss_position_{};
    /**
     * @brief GNSS position in WG84 coordinate system
     */
    std::vector<double> gnss_position_;
    /**
     * @brief GNSS rotation angles (yaw, pitch, roll) in radians
     */
    std::vector<double> gnss_rotation_;
    /**
     * @brief Whether Latitude and Longitude of the GNSS Position are swapped (RH works with LON/LAT format)
     */
    bool gnss_swap_lat_lon_{};
    /**
     * @brief Region of interest in the map
     */
    std::vector<double> map_roi_;
    /**
     * @brief Map foresight (radius) in meters
     */
    double map_foresight_{};
    /**
     * @brief Whether map uses geoid
     */
    bool map_use_geoid_{};
    /**
     * @brief Whether map over the air update is active
     */
    bool map_ota_update_{};
    /**
     * @brief Quality of service setting for rail horizon publisher
     */
    int dds_qos_{};

private:
    /**
     * @brief Declares all node config parameters
     *
     */
    void declare_config_parameters(rclcpp::Node& parent_node);
    /**
     * @brief Reads all config parameters and converts them in the appropriate data type
     *
     */
    void init_config_parameters(rclcpp::Node& parent_node);
};

#endif // DSD_RAIL_HORIZON_RAIL_PUBLISHER_H