/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dsd_rail_horizon/publisher/RailHorizonPublisher.h"

#include <dsd_rail_horizon/interfaces/implementations/RosPublisher.h>
#include <dsd_rail_horizon/utils/CalculationHelper.h>
#include <dsd_rail_horizon/utils/MessageHelper.h>

#include <dsd_rail_horizon_core/MapModel.h>

#include <dsd_common_types/GeoGeometryTypes.h>

#include <dsd_ros_messages/msg/rail_horizon.hpp>
#include <dsd_ros_messages/msg/track.hpp>

#include <boost/geometry/geometries/geometries.hpp>

#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <array>
#include <ext/alloc_traits.h>
#include <iterator>
#include <memory>
#include <utility>

RailHorizonPublisher::RailHorizonPublisher(rclcpp::Node& parent_node, const RailHorizonPublisherConfig& config)
: config_{config}
{
    rail_horizon_publisher_ =
        std::make_shared<RosPublisher<RailHorizonStampedMessage>>(parent_node, config.publisher_topic_name, config.qos);
}

void RailHorizonPublisher::publish(const StructureContainer& structures,
    const GeometryTransformation& local_transformation,
    const dsd_ros_messages::msg::CoupledLocalizationStamped& coupled_localization_message,
    const std::vector<Id>& track_ids, const std::string& map_version)
{
    dsd_ros_messages::msg::RailHorizonStamped message;
    message.header.stamp = coupled_localization_message.header.stamp;
    message.header.frame_id = config_.rail_horizon_frame_id;
    message.rail_horizon.reference_position = coupled_localization_message.coupled_localization;
    message.rail_horizon.global_horizon = fill_global_horizon(structures, track_ids);
    message.rail_horizon.local_horizon = fill_local_horizon(structures, track_ids, local_transformation);
    message.rail_horizon.optional_attributes = create_key_value_map_message({{"map_version", map_version}});
    rail_horizon_publisher_->publish(message);
}

// TOD(leon): Split this method up into smaller submethods, just poc right now
dsd_ros_messages::msg::RailHorizonData RailHorizonPublisher::fill_global_horizon(
    const StructureContainer& structures, const std::vector<Id>& track_ids)
{
    RailHorizonDataMessage global_horizon;
    if (config_.publish_global_horizon)
    {
        add_vertical_structures_to_global_horizon(global_horizon, structures.vertical);
        add_horizontal_structures_to_global_horizon(global_horizon, structures.horizontal, track_ids);
        add_body_structures_to_global_horizon(global_horizon, structures.body);
        add_plane_structures_to_global_horizon(global_horizon, structures.plane);
    }
    return global_horizon;
}

dsd_ros_messages::msg::RailHorizonData RailHorizonPublisher::fill_local_horizon(const StructureContainer& structures,
    const std::vector<Id>& track_ids, const GeometryTransformation& local_transformation)
{
    RailHorizonDataMessage local_horizon;
    if (config_.publish_local_horizon)
    {
        add_vertical_structures_to_local_horizon(local_horizon, structures.vertical, local_transformation);
        add_horizontal_structures_to_local_horizon(
            local_horizon, structures.horizontal, track_ids, local_transformation);
        add_body_structures_to_local_horizon(local_horizon, structures.body, local_transformation);
        add_plane_structures_to_local_horizon(local_horizon, structures.plane, local_transformation);
    }
    return local_horizon;
}

void RailHorizonPublisher::add_vertical_structures_to_global_horizon(
    RailHorizonDataMessage& global_horizon, const std::vector<VerticalStructure::Ptr>& structures)
{
    for (const auto& vertical_struct : structures)
    {
        auto landmark =
            create_vertical_landmark(vertical_struct->id_, vertical_struct->type_, vertical_struct->bottom_top_line_[0],
                vertical_struct->bottom_radius_, vertical_struct->bottom_top_line_[1], vertical_struct->top_radius_);
        global_horizon.landmarks.push_back(landmark);
    }
}

void RailHorizonPublisher::add_horizontal_structures_to_global_horizon(RailHorizonDataMessage& global_horizon,
    const std::vector<HorizontalStructure::Ptr>& structures, const std::vector<Id>& track_ids)
{
    for (const auto& horizontal_struct : structures)
    {
        bool part_of_mission = std::find(begin(track_ids), end(track_ids), horizontal_struct->id_) != end(track_ids);

        std::vector<PointXyzWgs84> virtual_centerline = {};
        if (config_.rail_horizon_virtual_centerline)
        {
            virtual_centerline = horizontal_struct->centerline_geometry_;
        }

        std::vector<PointXyzWgs84> left_rail = {};
        std::vector<PointXyzWgs84> right_rail = {};
        if (config_.rail_horizon_left_right_rail)
        {
            left_rail = horizontal_struct->left_rail_geometry_;
            right_rail = horizontal_struct->right_rail_geometry_;
        }

        auto track = create_track(horizontal_struct->id_, horizontal_struct->start_node_id_,
            horizontal_struct->end_node_id_, virtual_centerline, left_rail, right_rail, part_of_mission);
        global_horizon.tracks.push_back(track);

        if (track.part_of_mission)
        {
            global_horizon.driving_path.push_back(create_id_message(horizontal_struct->id_));
        }
    }
}

void RailHorizonPublisher::add_body_structures_to_global_horizon(
    RailHorizonDataMessage& global_horizon, const std::vector<BodyStructure::Ptr>& structures)
{
    for (const auto& body_struct : structures)
    {
        auto landmark = create_body_landmark(
            body_struct->id_, body_struct->type_, body_struct->geometry_.outer(), body_struct->height_);
        global_horizon.landmarks.push_back(landmark);
    }
}

void RailHorizonPublisher::add_plane_structures_to_global_horizon(
    RailHorizonDataMessage& global_horizon, const std::vector<PlaneStructure::Ptr>& structures)
{
    for (const auto& plane_struct : structures)
    {
        if (plane_struct->type_ == PlaneStructure::Type::ZONE)
        {
            auto landmark = create_zone_landmark(
                plane_struct->id_, plane_struct->type_, plane_struct->sub_type_, plane_struct->geometry_.outer());
            global_horizon.zones.push_back(landmark);
        }
        else
        {
            auto landmark =
                create_plane_landmark(plane_struct->id_, plane_struct->type_, plane_struct->geometry_.outer());
            global_horizon.landmarks.push_back(landmark);
        }
    }
}

void RailHorizonPublisher::add_vertical_structures_to_local_horizon(RailHorizonDataMessage& local_horizon,
    const std::vector<VerticalStructure::Ptr>& structures, const GeometryTransformation& local_transformation)
{
    for (const auto& vertical_struct : structures)
    {
        auto local_geometry = local_transformation.transform(vertical_struct->bottom_top_line_utm_);
        auto landmark = create_vertical_landmark(vertical_struct->id_, vertical_struct->type_, local_geometry[0],
            vertical_struct->bottom_radius_, local_geometry[1], vertical_struct->top_radius_);
        local_horizon.landmarks.push_back(landmark);
    }
}

void RailHorizonPublisher::add_horizontal_structures_to_local_horizon(RailHorizonDataMessage& local_horizon,
    const std::vector<HorizontalStructure::Ptr>& structures, const std::vector<Id>& track_ids,
    const GeometryTransformation& local_transformation)
{
    for (const auto& horizontal_struct : structures)
    {
        bool part_of_mission = std::find(begin(track_ids), end(track_ids), horizontal_struct->id_) != end(track_ids);

        std::vector<PointXyz> virtual_centerline = {};
        if (config_.rail_horizon_virtual_centerline)
        {
            virtual_centerline = local_transformation.transform(horizontal_struct->centerline_geometry_utm_);
        }

        std::vector<PointXyz> left_rail = {};
        std::vector<PointXyz> right_rail = {};
        if (config_.rail_horizon_left_right_rail)
        {
            left_rail = local_transformation.transform(horizontal_struct->left_rail_geometry_utm_);
            right_rail = local_transformation.transform(horizontal_struct->right_rail_geometry_utm_);
        }

        auto track = create_track(horizontal_struct->id_, horizontal_struct->start_node_id_,
            horizontal_struct->end_node_id_, virtual_centerline, left_rail, right_rail, part_of_mission);
        local_horizon.tracks.push_back(track);

        if (track.part_of_mission)
        {
            local_horizon.driving_path.push_back(create_id_message(horizontal_struct->id_));
        }
    }
}

void RailHorizonPublisher::add_body_structures_to_local_horizon(RailHorizonDataMessage& local_horizon,
    const std::vector<BodyStructure::Ptr>& structures, const GeometryTransformation& local_transformation)
{
    for (const auto& body_struct : structures)
    {
        auto local_geometry = local_transformation.transform(body_struct->geometry_utm_.outer());
        auto landmark =
            create_body_landmark(body_struct->id_, body_struct->type_, local_geometry, body_struct->height_);
        local_horizon.landmarks.push_back(landmark);
    }
}

void RailHorizonPublisher::add_plane_structures_to_local_horizon(RailHorizonDataMessage& local_horizon,
    const std::vector<PlaneStructure::Ptr>& structures, const GeometryTransformation& local_transformation)
{
    for (const auto& plane_struct : structures)
    {
        auto local_geometry = local_transformation.transform(plane_struct->geometry_utm_.outer());
        if (plane_struct->type_ == PlaneStructure::Type::ZONE)
        {
            auto landmark =
                create_zone_landmark(plane_struct->id_, plane_struct->type_, plane_struct->sub_type_, local_geometry);
            local_horizon.zones.push_back(landmark);
        }
        else
        {
            auto landmark = create_plane_landmark(plane_struct->id_, plane_struct->type_, local_geometry);
            local_horizon.landmarks.push_back(landmark);
        }
    }
}