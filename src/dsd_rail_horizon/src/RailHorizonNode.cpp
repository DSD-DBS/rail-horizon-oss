/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dsd_rail_horizon/RailHorizonNode.h"

#include "dsd_rail_horizon/publisher/MarkerStructurePublisher.h"
#include "dsd_rail_horizon/publisher/RailHorizonPublisher.h"

#include <dsd_rail_horizon/interfaces/implementations/RosClock.h>
#include <dsd_rail_horizon/interfaces/implementations/RosLogger.h>
#include <dsd_rail_horizon/publisher/AppStatusPublisher.h>
#include <dsd_rail_horizon/RailHorizonParameters.h>
#include <dsd_rail_horizon/subscriber/CoupledLocalizationSubscriber.h>
#include <dsd_rail_horizon/subscriber/MissionProfileSubscriber.h>
#include <dsd_rail_horizon/utils/CalculationHelper.h>

#include <dsd_rail_horizon_core/LookupStructures.h>
#include <dsd_rail_horizon_core/MapDBReader.h>

#include <dsd_common_types/GeoGeometryTypes.h>
#include <dsd_common_types/GeometryTypes.h>

#include <dsd_ros_messages/msg/coupled_localization_stamped.hpp>

#include <boost/format.hpp>
#include <boost/qvm/all.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <vector>

RailHorizonNode::RailHorizonNode(const rclcpp::NodeOptions& options) : Node("rail_horizon_publisher", options)
{
    parameters_ = std::make_shared<RailHorizonParameters>(*this);
    logger_ = std::make_shared<RosLogger>(this->get_logger());
    clock_ = std::make_shared<RosClock>(this->get_clock());

    auto gnss_logger = std::make_shared<RosLogger>(this->get_logger(), "gnss projection");
    auto gnss_projection_config = create_wsg84_to_utm_config(gnss_logger, parameters_->coupled_localization_use_geoid_);
    gnss_projection_ = std::make_shared<Projection>(gnss_projection_config, gnss_logger);

    auto map_logger = std::make_shared<RosLogger>(this->get_logger(), "map projection");
    auto map_projection_config = create_wsg84_to_utm_config(map_logger, parameters_->map_use_geoid_);
    map_projection_ = std::make_shared<Projection>(map_projection_config, map_logger);

    setup_publishers_and_subscribers();
    init_lookup_structures();
    timer_rh_callback_ =
        this->create_wall_timer(1s, std::bind(&RailHorizonNode::rail_horizon_periodic_broadcast_callback, this));
}

void RailHorizonNode::setup_publishers_and_subscribers()
{
    RailHorizonPublisherConfig rail_horizon_publisher_config{
        parameters_->rail_horizon_namespace_ + parameters_->rail_horizon_topic_,
        parameters_->dds_qos_,
        parameters_->rail_horizon_frame_id_,
        parameters_->rail_horizon_global_horizon_,
        parameters_->rail_horizon_local_horizon_,
        parameters_->rail_horizon_virtual_centerline_,
        parameters_->rail_horizon_left_right_rail_,
    };
    rail_horizon_publisher_ = std::make_shared<RailHorizonPublisher>(*this, rail_horizon_publisher_config);

    CoupledLocalizationSubscriberConfig coupled_localization_subscriber_config{
        parameters_->coupled_localization_topic_,
        parameters_->dds_qos_,
        parameters_->spoof_gnss_position_,
        parameters_->gnss_position_,
        std::vector<float>{parameters_->gnss_rotation_.begin(), parameters_->gnss_rotation_.end()},
        parameters_->gnss_swap_lat_lon_,
    };
    coupled_localization_subscriber_ =
        std::make_shared<CoupledLocalizationSubscriber>(*this, coupled_localization_subscriber_config, logger_, clock_);

    if (parameters_->rail_horizon_markers_)
    {
        MarkerStructurePublisherConfig marker_structure_publisher_config{
            parameters_->rail_horizon_namespace_ + parameters_->rail_horizon_marker_position_topic_,
            parameters_->rail_horizon_namespace_ + parameters_->rail_horizon_marker_vertical_structure_topic_,
            parameters_->rail_horizon_namespace_ + parameters_->rail_horizon_marker_horizontal_structure_topic_,
            parameters_->rail_horizon_namespace_ + parameters_->rail_horizon_marker_plane_structure_topic_,
            parameters_->rail_horizon_namespace_ + parameters_->rail_horizon_marker_body_structure_topic_,
            parameters_->dds_qos_,
            parameters_->rail_horizon_marker_reference_frame_,
            parameters_->rail_horizon_virtual_centerline_,
            parameters_->rail_horizon_left_right_rail_,
        };
        marker_publisher_ =
            std::make_shared<MarkerStructurePublisher>(*this, marker_structure_publisher_config, clock_);
    }

    if (parameters_->mission_profile_apply_)
    {
        MissionProfileSubscriberConfig mission_profile_subscriber_config{
            parameters_->mission_profile_topic_, parameters_->dds_qos_};
        mission_profile_subscriber_ =
            std::make_shared<MissionProfileSubscriber>(*this, mission_profile_subscriber_config, logger_);
    }

    if (parameters_->mission_control_supervision_)
    {
        AppStatusPublisherConfig app_status_publisher_config{parameters_->application_status_topic_,
            parameters_->application_configuration_topic_, parameters_->dds_qos_, parameters_->mission_control_app_id_,
            parameters_->mission_control_node_id_};
        app_status_publisher_ =
            std::make_shared<AppStatusPublisher>(*this, app_status_publisher_config, logger_, clock_);
        app_status_publisher_->set_error_status(
            gnss_projection_->get_error_status() + map_projection_->get_error_status());
    }
}

void RailHorizonNode::init_lookup_structures()
{
    auto map_reader =
        std::make_shared<MapDBReader>(read_db_map_reader_config_from_env(logger_), map_projection_, logger_);
    bool map_was_updated_ota = parameters_->map_ota_update_ ? map_reader->update_map_over_the_air() : false;
    lookup_structures_ = map_reader->read(parameters_->map_roi_);
    if (!lookup_structures_)
    {
        logger_->error("Rail Horizon is exiting!");
        exit(-1);
    }

    map_version_ = std::to_string(map_reader->get_local_map_version());

    std::string running_status = std::invoke([&] {
        if (map_was_updated_ota)
        {
            return "Map update via MapCDB successful. Using online map (version " + map_version_ + ")";
        }
        return "Using OFFLINE map (version " + map_version_ + ")";
    });
    logger_->info(running_status);
    if (parameters_->mission_control_supervision_)
    {
        app_status_publisher_->set_running_status(running_status);
    }
}

void RailHorizonNode::rail_horizon_periodic_broadcast_callback()
{
    gnss_projection_->create_projections_on_error();
    map_projection_->create_projections_on_error();

    if (parameters_->mission_control_supervision_)
    {
        app_status_publisher_->set_error_status(
            gnss_projection_->get_error_status() + map_projection_->get_error_status());
        if (!app_status_publisher_->is_app_running())
        {
            return;
        }
    }

    if (!coupled_localization_subscriber_->is_data_available())
    {
        logger_->info("Received no position update ...");
        return;
    }
    auto coupled_localization_message = coupled_localization_subscriber_->get_message();

    PointXyz current_gnss_position = extract_current_gnss_position(coupled_localization_message);
    GeometryTransformation local_transformation{
        current_gnss_position, extract_rotation_matrix(coupled_localization_message)};

    auto structures =
        lookup_structures_->get_structures_in_map_foresight(current_gnss_position, parameters_->map_foresight_);

    auto map_foresight = static_cast<int>(parameters_->map_foresight_);
    logger_->info(
        boost::format(
            "Identified %ld vertical structures within the radius %d m according to the current GNSS position.") %
        structures.vertical.size() % map_foresight);
    logger_->info(
        boost::format(
            "Identified %ld horizontal structures within the radius %d m according to the current GNSS position.") %
        structures.horizontal.size() % map_foresight);
    logger_->info(
        boost::format("Identified %ld body structures within the radius %d m according to the current GNSS position.") %
        structures.body.size() % map_foresight);
    logger_->info(
        boost::format(
            "Identified %ld plane structures within the radius %d m according to the current GNSS position.") %
        structures.plane.size() % map_foresight);

    logger_->info(boost::format("Publishing Rail Horizon message (global=%d, local=%d, markers=%d).") %
                  parameters_->rail_horizon_global_horizon_ % parameters_->rail_horizon_local_horizon_ %
                  parameters_->rail_horizon_markers_);

    rail_horizon_publisher_->publish(
        structures, local_transformation, coupled_localization_message, get_track_ids(), map_version_);
    if (marker_publisher_)
    {
        marker_publisher_->publish(structures, local_transformation);
    }
}

PointXyz RailHorizonNode::extract_current_gnss_position(
    const dsd_ros_messages::msg::CoupledLocalizationStamped& coupled_localization_message)
{
    PointXyzWgs84 current_wgs84_gnss_position = get_current_gnss_position(coupled_localization_message);
    PointXyz current_gnss_position = gnss_projection_->transform_point_wgs84_to_utm(current_wgs84_gnss_position);
    logger_->info(boost::format("Reference position for RH: %f %f %f (%f %f %f)") % current_gnss_position.get<0>() %
                  current_gnss_position.get<1>() % current_gnss_position.get<2>() %
                  current_wgs84_gnss_position.get<0>() % current_wgs84_gnss_position.get<1>() %
                  current_wgs84_gnss_position.get<2>());
    return current_gnss_position;
}

RotationMatrix RailHorizonNode::extract_rotation_matrix(
    const dsd_ros_messages::msg::CoupledLocalizationStamped& coupled_localization_message)
{
    if (parameters_->rail_horizon_local_horizon_apply_rotations_)
    {
        PointXyz current_gnss_orientation = get_current_gnss_orientation(coupled_localization_message);
        RCLCPP_INFO(this->get_logger(), "Rotation of local horizon is applied: %f %f %f (roll, pitch, yaw)",
            current_gnss_orientation.get<0>(), current_gnss_orientation.get<1>(), current_gnss_orientation.get<2>());
        return calculate_rotation_matrix(current_gnss_orientation);
    }
    return boost::qvm::identity_mat<double, 4>();
}

std::vector<Id> RailHorizonNode::get_track_ids()
{
    if (mission_profile_subscriber_)
    {
        return mission_profile_subscriber_->get_track_ids();
    }
    return {};
}