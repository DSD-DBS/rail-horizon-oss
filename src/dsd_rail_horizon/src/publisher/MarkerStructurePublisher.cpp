/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dsd_rail_horizon/publisher/MarkerStructurePublisher.h"

#include <dsd_rail_horizon/interfaces/IClock.h>
#include <dsd_rail_horizon/interfaces/implementations/RosPublisher.h>
#include <dsd_rail_horizon/interfaces/IPublisher.h>
#include <dsd_rail_horizon/utils/CalculationHelper.h>
#include <dsd_rail_horizon/utils/MessageHelper.h>

#include <dsd_rail_horizon_core/LookupStructures.h>
#include <dsd_rail_horizon_core/MapModel.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <array>
#include <iterator>
#include <memory>

MarkerStructurePublisher::MarkerStructurePublisher(
    rclcpp::Node& parent_node, const MarkerStructurePublisherConfig& config, IClock::ConstSharedPtr clock)
: config_{config}, clock_{clock}
{
    position_marker_publisher_ = std::make_shared<RosPublisher<visualization_msgs::msg::Marker>>(
        parent_node, config.position_marker_publisher_topic_name, config.qos);
    vertical_structure_marker_publisher_ = std::make_shared<RosPublisher<visualization_msgs::msg::MarkerArray>>(
        parent_node, config.vertical_structure_marker_publisher_topic_name, config.qos);
    horizontal_structure_marker_publisher_ = std::make_shared<RosPublisher<visualization_msgs::msg::MarkerArray>>(
        parent_node, config.horizontal_structure_marker_publisher_topic_name, config.qos);
    plane_structure_marker_publisher_ = std::make_shared<RosPublisher<visualization_msgs::msg::MarkerArray>>(
        parent_node, config.plane_structure_marker_publisher_topic_name, config.qos);
    body_structure_marker_publisher_ = std::make_shared<RosPublisher<visualization_msgs::msg::MarkerArray>>(
        parent_node, config.body_structure_marker_publisher_topic_name, config.qos);
}

void MarkerStructurePublisher::publish(
    const StructureContainer& structures, const GeometryTransformation& local_transformation)
{
    publish_position_marker();
    publish_vertical_structure_marker(structures.vertical, local_transformation);
    publish_horizontal_structure_marker(structures.horizontal, local_transformation);
    publish_plane_structure_marker(structures.plane, local_transformation);
    publish_body_structure_marker(structures.body, local_transformation);
}

void MarkerStructurePublisher::publish_position_marker()
{
    visualization_msgs::msg::Marker position_marker = create_marker_message(0, visualization_msgs::msg::Marker::CUBE);
    position_marker.color = create_color_message(1.0, 0.0, 0.0, 1.0);
    position_marker_publisher_->publish(position_marker);
}

void MarkerStructurePublisher::publish_vertical_structure_marker(
    std::vector<VerticalStructure::Ptr> vertical_structures, const GeometryTransformation& local_transformation)
{
    visualization_msgs::msg::MarkerArray marker_array{};
    std::transform(begin(vertical_structures), end(vertical_structures), std::back_inserter(marker_array.markers),
        [&](const auto& vertical_struct) {
        return create_marker_from_vertical_structure(vertical_struct, local_transformation);
    });
    vertical_structure_marker_publisher_->publish(marker_array);
}

void MarkerStructurePublisher::publish_horizontal_structure_marker(
    std::vector<HorizontalStructure::Ptr> horizontal_structures, const GeometryTransformation& local_transformation)
{
    visualization_msgs::msg::MarkerArray marker_array{};
    for (const auto& horizontal_struct : horizontal_structures)
    {
        if (config_.rail_horizon_virtual_centerline)
        {
            marker_array.markers.push_back(
                create_centerline_marker_from_horizontal_structure(horizontal_struct, local_transformation));
        }
        if (config_.rail_horizon_left_right_rail)
        {
            marker_array.markers.push_back(
                create_left_rail_marker_from_horizontal_structure(horizontal_struct, local_transformation));
            marker_array.markers.push_back(
                create_right_rail_marker_from_horizontal_structure(horizontal_struct, local_transformation));
        }
    }
    horizontal_structure_marker_publisher_->publish(marker_array);
}

void MarkerStructurePublisher::publish_plane_structure_marker(
    std::vector<PlaneStructure::Ptr> plane_structures, const GeometryTransformation& local_transformation)
{
    visualization_msgs::msg::MarkerArray marker_array{};
    std::transform(begin(plane_structures), end(plane_structures), std::back_inserter(marker_array.markers),
        [&](const auto& plane_struct) {
        return create_marker_from_plane_structure(plane_struct, local_transformation);
    });
    plane_structure_marker_publisher_->publish(marker_array);
}

void MarkerStructurePublisher::publish_body_structure_marker(
    std::vector<BodyStructure::Ptr> body_structures, const GeometryTransformation& local_transformation)
{
    visualization_msgs::msg::MarkerArray marker_array{};
    std::transform(begin(body_structures), end(body_structures), std::back_inserter(marker_array.markers),
        [&](const auto& body_struct) {
        return create_marker_from_body_structure(body_struct, local_transformation);
    });
    body_structure_marker_publisher_->publish(marker_array);
}

visualization_msgs::msg::Marker MarkerStructurePublisher::create_marker_from_vertical_structure(
    const VerticalStructure::Ptr vertical_struct, const GeometryTransformation& local_transformation)
{
    visualization_msgs::msg::Marker marker =
        create_marker_message(vertical_struct->id_, visualization_msgs::msg::Marker::LINE_LIST);
    marker.color = create_color_message(0.0, 1.0, 0.0);
    marker.points = geometry_to_point_vector(local_transformation.transform(vertical_struct->bottom_top_line_utm_));
    return marker;
}


visualization_msgs::msg::Marker MarkerStructurePublisher::create_centerline_marker_from_horizontal_structure(
    const HorizontalStructure::Ptr horizontal_struct, const GeometryTransformation& local_transformation)
{
    visualization_msgs::msg::Marker marker =
        create_marker_message(horizontal_struct->id_, visualization_msgs::msg::Marker::LINE_STRIP);
    marker.color = create_color_message(0.0, 0.0, 1.0);
    marker.ns = "track_center";
    marker.points =
        geometry_to_point_vector(local_transformation.transform(horizontal_struct->centerline_geometry_utm_));
    return marker;
}

visualization_msgs::msg::Marker MarkerStructurePublisher::create_left_rail_marker_from_horizontal_structure(
    const HorizontalStructure::Ptr horizontal_struct, const GeometryTransformation& local_transformation)
{
    visualization_msgs::msg::Marker marker =
        create_marker_message(horizontal_struct->id_, visualization_msgs::msg::Marker::LINE_STRIP);
    marker.color = create_color_message(0.0, 0.6, 0.9);
    marker.ns = "track_left";
    marker.points =
        geometry_to_point_vector(local_transformation.transform(horizontal_struct->left_rail_geometry_utm_));
    return marker;
}


visualization_msgs::msg::Marker MarkerStructurePublisher::create_right_rail_marker_from_horizontal_structure(
    const HorizontalStructure::Ptr horizontal_struct, const GeometryTransformation& local_transformation)
{
    visualization_msgs::msg::Marker marker =
        create_marker_message(horizontal_struct->id_, visualization_msgs::msg::Marker::LINE_STRIP);
    marker.color = create_color_message(0.0, 1.0, 1.0);
    marker.ns = "track_right";
    marker.points =
        geometry_to_point_vector(local_transformation.transform(horizontal_struct->right_rail_geometry_utm_));
    return marker;
}

visualization_msgs::msg::Marker MarkerStructurePublisher::create_marker_from_plane_structure(
    const PlaneStructure::Ptr plane_struct, const GeometryTransformation& local_transformation)
{
    visualization_msgs::msg::Marker marker =
        create_marker_message(plane_struct->id_, visualization_msgs::msg::Marker::LINE_STRIP);
    marker.points = geometry_to_point_vector(local_transformation.transform(plane_struct->geometry_utm_.outer()));

    if (plane_struct->type_ == PlaneStructure::Type::ZONE)
    {
        if (plane_struct->sub_type_ == PlaneStructure::SubType::RISK_ZONE)
        {
            marker.color = create_color_message(1.0, 0.0, 0.0);
            marker.ns = "plane_risk_zone";
        }
        else if (plane_struct->sub_type_ == PlaneStructure::SubType::NEAR_TRACK_ZONE)
        {
            marker.color = create_color_message(1.0, 0.5, 0.0);
            marker.ns = "plane_near track_zone";
        }
        else if (plane_struct->sub_type_ == PlaneStructure::SubType::SAFE_ZONE)
        {
            marker.color = create_color_message(1.0, 1.0, 0.0);
            marker.ns = "plane_safe_zone";
        }
    }
    else
    {
        marker.color = create_color_message(0.6, 0.3, 0.0);
        marker.ns = "plane_other";
    }
    return marker;
}

visualization_msgs::msg::Marker MarkerStructurePublisher::create_marker_from_body_structure(
    const BodyStructure::Ptr body_struct, const GeometryTransformation& local_transformation)
{
    visualization_msgs::msg::Marker marker =
        create_marker_message(body_struct->id_, visualization_msgs::msg::Marker::LINE_STRIP);
    marker.color = create_color_message(1.0, 0.0, 1.0);
    marker.points = geometry_to_point_vector(local_transformation.transform(body_struct->geometry_utm_.outer()));
    return marker;
}

visualization_msgs::msg::Marker MarkerStructurePublisher::create_marker_message(const Id& id, int32_t type)
{
    visualization_msgs::msg::Marker marker{};
    marker.header.frame_id = config_.marker_reference_frame;
    marker.header.stamp = clock_->now();
    marker.ns = "default";
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale = create_scale_message(0.2, 0.2, 0.2);
    return marker;
}
