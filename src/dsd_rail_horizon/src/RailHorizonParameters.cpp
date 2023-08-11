/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <dsd_rail_horizon/RailHorizonParameters.h>

#include <rclcpp/rclcpp.hpp>

RailHorizonParameters::RailHorizonParameters(rclcpp::Node& parent_node)
{
    declare_config_parameters(parent_node);
    init_config_parameters(parent_node);
}

void RailHorizonParameters::declare_config_parameters(rclcpp::Node& parent_node)
{
    parent_node.declare_parameter("rail_horizon_namespace", "/eta/rail_horizon/");
    parent_node.declare_parameter("rail_horizon_topic", "rail_horizon");
    parent_node.declare_parameter("rail_horizon_frame_id", "rail_horizon_eta");
    parent_node.declare_parameter("rail_horizon_marker_reference_frame", "train_eta");
    parent_node.declare_parameter("rail_horizon_marker_position_topic", "marker/position");
    parent_node.declare_parameter("rail_horizon_marker_vertical_structure_topic", "marker/vertical_structures");
    parent_node.declare_parameter("rail_horizon_marker_horizontal_structure_topic", "marker/horizontal_structures");
    parent_node.declare_parameter("rail_horizon_marker_plane_structure_topic", "marker/plane_structures");
    parent_node.declare_parameter("rail_horizon_marker_body_structure_topic", "marker/body_structures");
    parent_node.declare_parameter("rail_horizon_topic_frequency", 1);
    parent_node.declare_parameter("rail_horizon_global_horizon", true);
    parent_node.declare_parameter("rail_horizon_local_horizon", false);
    parent_node.declare_parameter("rail_horizon_local_horizon_apply_rotations", false);
    parent_node.declare_parameter("rail_horizon_markers", true);
    parent_node.declare_parameter("rail_horizon_virtual_centerline", true);
    parent_node.declare_parameter("rail_horizon_left_right_rail", true);
    parent_node.declare_parameter("application_configuration_topic", "/system_config");
    parent_node.declare_parameter("application_status_topic", "/app_status");
    parent_node.declare_parameter("mission_control_supervision", false);
    parent_node.declare_parameter("mission_control_app_id", "rail_horizon_eta");
    parent_node.declare_parameter("mission_control_node_id", "pld_server_1_card_2");
    parent_node.declare_parameter("mission_profile_apply", true);
    parent_node.declare_parameter("mission_profile_topic", "/mission_profile");
    parent_node.declare_parameter("coupled_localization_topic", "coupled_localization_ground_truth");
    parent_node.declare_parameter("coupled_localization_use_geoid", false);
    parent_node.declare_parameter("spoof_gnss_position", true);
    parent_node.declare_parameter("gnss_position", std::vector<double>{569220.39, 5932662.01, 52.5});
    parent_node.declare_parameter("gnss_rotation", std::vector<double>{0.0, 0.0, 0.0});
    parent_node.declare_parameter("gnss_swap_lat_lon", true);
    parent_node.declare_parameter("map_roi", std::vector<double>{9.42564, 53.36550, 10.38914, 53.69209});
    parent_node.declare_parameter("map_foresight", 500.0);
    parent_node.declare_parameter("map_use_geoid", false);
    parent_node.declare_parameter("map_ota_update", false);
    parent_node.declare_parameter("dds_qos", 3);
}

void RailHorizonParameters::init_config_parameters(rclcpp::Node& parent_node)
{
    rail_horizon_namespace_ = parent_node.get_parameter("rail_horizon_namespace").as_string();
    rail_horizon_topic_ = parent_node.get_parameter("rail_horizon_topic").as_string();
    rail_horizon_frame_id_ = parent_node.get_parameter("rail_horizon_frame_id").as_string();
    rail_horizon_marker_reference_frame_ = parent_node.get_parameter("rail_horizon_marker_reference_frame").as_string();
    rail_horizon_marker_position_topic_ = parent_node.get_parameter("rail_horizon_marker_position_topic").as_string();
    rail_horizon_marker_vertical_structure_topic_ =
        parent_node.get_parameter("rail_horizon_marker_vertical_structure_topic").as_string();
    rail_horizon_marker_horizontal_structure_topic_ =
        parent_node.get_parameter("rail_horizon_marker_horizontal_structure_topic").as_string();
    rail_horizon_marker_plane_structure_topic_ =
        parent_node.get_parameter("rail_horizon_marker_plane_structure_topic").as_string();
    rail_horizon_marker_body_structure_topic_ =
        parent_node.get_parameter("rail_horizon_marker_body_structure_topic").as_string();
    rail_horizon_topic_frequency_ = parent_node.get_parameter("rail_horizon_topic_frequency").as_int();
    rail_horizon_global_horizon_ = parent_node.get_parameter("rail_horizon_global_horizon").as_bool();
    rail_horizon_local_horizon_ = parent_node.get_parameter("rail_horizon_local_horizon").as_bool();
    rail_horizon_local_horizon_apply_rotations_ =
        parent_node.get_parameter("rail_horizon_local_horizon_apply_rotations").as_bool();
    rail_horizon_markers_ = parent_node.get_parameter("rail_horizon_markers").as_bool();
    rail_horizon_virtual_centerline_ = parent_node.get_parameter("rail_horizon_virtual_centerline").as_bool();
    rail_horizon_left_right_rail_ = parent_node.get_parameter("rail_horizon_left_right_rail").as_bool();
    application_configuration_topic_ = parent_node.get_parameter("application_configuration_topic").as_string();
    application_status_topic_ = parent_node.get_parameter("application_status_topic").as_string();
    mission_control_supervision_ = parent_node.get_parameter("mission_control_supervision").as_bool();
    mission_control_app_id_ = parent_node.get_parameter("mission_control_app_id").as_string();
    mission_control_node_id_ = parent_node.get_parameter("mission_control_node_id").as_string();
    mission_profile_apply_ = parent_node.get_parameter("mission_profile_apply").as_bool();
    mission_profile_topic_ = parent_node.get_parameter("mission_profile_topic").as_string();
    coupled_localization_topic_ = parent_node.get_parameter("coupled_localization_topic").as_string();
    coupled_localization_use_geoid_ = parent_node.get_parameter("coupled_localization_use_geoid").as_bool();
    spoof_gnss_position_ = parent_node.get_parameter("spoof_gnss_position").as_bool();
    gnss_position_ = parent_node.get_parameter("gnss_position").as_double_array();
    gnss_rotation_ = parent_node.get_parameter("gnss_rotation").as_double_array();
    gnss_swap_lat_lon_ = parent_node.get_parameter("gnss_swap_lat_lon").as_bool();
    map_roi_ = parent_node.get_parameter("map_roi").as_double_array();
    map_foresight_ = parent_node.get_parameter("map_foresight").as_double();
    map_use_geoid_ = parent_node.get_parameter("map_use_geoid").as_bool();
    map_ota_update_ = parent_node.get_parameter("map_ota_update").as_bool();
    dds_qos_ = parent_node.get_parameter("dds_qos").as_int();
}