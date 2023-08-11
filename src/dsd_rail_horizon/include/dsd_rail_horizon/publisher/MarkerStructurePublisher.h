/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MARKER_STRUCTURE_PUBLISHER_H
#define MARKER_STRUCTURE_PUBLISHER_H

#include "dsd_rail_horizon/utils/CalculationHelper.h"

#include <dsd_rail_horizon/interfaces/IClock.h>
#include <dsd_rail_horizon/interfaces/IPublisher.h>

#include <dsd_rail_horizon_core/LookupStructures.h>
#include <dsd_rail_horizon_core/MapModel.h>

#include <dsd_common_types/GeometryTypes.h>

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cstdint>
#include <string>
#include <vector>

/**
 * @brief Config parameters for the marker publishers
 */
struct MarkerStructurePublisherConfig
{
    /**
     * @brief Name of the topic the position marker publisher is publishíng to
     */
    std::string position_marker_publisher_topic_name;
    /**
     * @brief Name of the topic the vertical structure markers publisher is publishíng to
     */
    std::string vertical_structure_marker_publisher_topic_name;
    /**
     * @brief Name of the topic the horizontal structure markers publisher is publishíng to
     */
    std::string horizontal_structure_marker_publisher_topic_name;
    /**
     * @brief Name of the topic the plane structure markers publisher is publishíng to
     */
    std::string plane_structure_marker_publisher_topic_name;
    /**
     * @brief Name of the topic the body structure markers publisher is publishíng to
     */
    std::string body_structure_marker_publisher_topic_name;
    /**
     * @brief Quality of service setting for the publishers
     */
    rclcpp::QoS qos;
    /**
     * @brief Reference frame for the markers (relevant for rviz)
     */
    std::string marker_reference_frame;
    /**
     * @brief Whether virtual centerline is filled in the message
     */
    bool rail_horizon_virtual_centerline;
    /**
     * @brief Whether left and right rail are filled in the message
     */
    bool rail_horizon_left_right_rail;
};

/**
 * @brief Marker Structures publisher for publishing markers for different structure types
 */
class MarkerStructurePublisher
{
public:
    /**
     * @brief Constructor for marker structure publisher
     * @param parent_node The parent node this publisher is associated with.
     * @param config Config for the marker structures publisher (see \ref MarkerStructurePublisherConfig)
     * @param clock Clock instance to get the current time for spoofing
     */
    MarkerStructurePublisher(
        rclcpp::Node& parent_node, const MarkerStructurePublisherConfig& config, IClock::ConstSharedPtr clock);
    /**
     * @brief Function to publish marker message
     * @param structures Container of structures near the train which should be published
     * @param local_transformation Transformation which encodes the positon and rotation of the train to get the
     * structures position relative to the train
     */
    void publish(const StructureContainer& structures, const GeometryTransformation& local_transformation);

private:
    /**
     * @brief Publish position marker
     */
    void publish_position_marker();
    /**
     * @brief Publish vertical structure marker
     * @param vertical_structures Vector of vertical structure pointers
     * @param local_transformation Transformation which encodes the positon and rotation of the train to get the
     * structures position relative to the train
     */
    void publish_vertical_structure_marker(
        std::vector<VerticalStructure::Ptr> vertical_structures, const GeometryTransformation& local_transformation);
    /**
     * @brief Publish horizontal structure marker
     * @param horizontal_structures Vector of horizontal structure pointers
     * @param local_transformation Transformation which encodes the positon and rotation of the train to get the
     * structures position relative to the train
     */
    void publish_horizontal_structure_marker(std::vector<HorizontalStructure::Ptr> horizontal_structures,
        const GeometryTransformation& local_transformation);
    /**
     * @brief Publish plane structure marker
     * @param plane_structures Vector of plane structure pointers
     * @param local_transformation Transformation which encodes the positon and rotation of the train to get the
     * structures position relative to the train
     */
    void publish_plane_structure_marker(
        std::vector<PlaneStructure::Ptr> plane_structures, const GeometryTransformation& local_transformation);
    /**
     * @brief Publish body structure marker
     * @param body_structures Vector of body structure pointers
     * @param local_transformation Transformation which encodes the positon and rotation of the train to get the
     * structures position relative to the train
     */
    void publish_body_structure_marker(
        std::vector<BodyStructure::Ptr> body_structures, const GeometryTransformation& local_transformation);
    /**
     * @brief Create marker from vertical structure
     * @param vertical_struct Constant pointer to vertical structure
     * @param local_transformation Transformation which encodes the positon and rotation of the train to get the
     * structures position relative to the train
     */
    visualization_msgs::msg::Marker create_marker_from_vertical_structure(
        const VerticalStructure::Ptr vertical_struct, const GeometryTransformation& local_transformation);
    /**
     * @brief Create centerline marker from horizontal structure
     * @param horizontal_struct Constant pointer to horizontal structure
     * @param local_transformation Transformation which encodes the positon and rotation of the train to get the
     * structures position relative to the train
     * @return mareker ROS Marker
     */
    visualization_msgs::msg::Marker create_centerline_marker_from_horizontal_structure(
        const HorizontalStructure::Ptr horizontal_struct, const GeometryTransformation& local_transformation);
    /**
     * @brief Create left rail marker from horizontal structure
     * @param horizontal_struct Constant pointer to horizontal structure
     * @param local_transformation Transformation which encodes the positon and rotation of the train to get the
     * structures position relative to the train
     * @return mareker ROS Marker
     */
    visualization_msgs::msg::Marker create_left_rail_marker_from_horizontal_structure(
        const HorizontalStructure::Ptr horizontal_struct, const GeometryTransformation& local_transformation);
    /**
     * @brief Create right rail marker from horizontal structure
     * @param horizontal_struct Constant pointer to horizontal structure
     * @param local_transformation Transformation which encodes the positon and rotation of the train to get the
     * structures position relative to the train
     * @return mareker ROS Marker
     */
    visualization_msgs::msg::Marker create_right_rail_marker_from_horizontal_structure(
        const HorizontalStructure::Ptr horizontal_struct, const GeometryTransformation& local_transformation);
    /**
     * @brief Create plane marker from plane structure
     * @param plane_struct Constant pointer to plane structure
     * @param local_transformation Transformation which encodes the positon and rotation of the train to get the
     * structures position relative to the train
     * @return mareker ROS Marker
     */
    visualization_msgs::msg::Marker create_marker_from_plane_structure(
        const PlaneStructure::Ptr plane_struct, const GeometryTransformation& local_transformation);
    /**
     * @brief Create body marker from body structure
     * @param body_struct Constant pointer to body structure
     * @param local_transformation Transformation which encodes the positon and rotation of the train to get the
     * structures position relative to the train
     * @return mareker ROS Marker
     */
    visualization_msgs::msg::Marker create_marker_from_body_structure(
        const BodyStructure::Ptr body_struct, const GeometryTransformation& local_transformation);
    /**
     * @brief Create marker
     * @param id Constant reference to Id
     * @param type integer value representing a type
     * @return mareker ROS Marker
     */
    visualization_msgs::msg::Marker create_marker_message(const Id& id, int32_t type);
    /**
     * @brief Config of type MarkerStructurePublisherConfig
     */
    MarkerStructurePublisherConfig config_;
    /**
     * @brief Clock for getting the current time for spoofing
     */
    IClock::ConstSharedPtr clock_;

protected:
    /**
     * @brief Shared pointer to the publisher for publishing the position marker
     */
    IPublisher<visualization_msgs::msg::Marker>::SharedPtr position_marker_publisher_;
    /**
     * @brief Shared pointer to the publisher for publishing the vertical structure marker
     */
    IPublisher<visualization_msgs::msg::MarkerArray>::SharedPtr vertical_structure_marker_publisher_;
    /**
     * @brief Shared pointer to the publisher for publishing the horizontal structure marker
     */
    IPublisher<visualization_msgs::msg::MarkerArray>::SharedPtr horizontal_structure_marker_publisher_;
    /**
     * @brief Shared pointer to the publisher for publishing the plane structure marker
     */
    IPublisher<visualization_msgs::msg::MarkerArray>::SharedPtr plane_structure_marker_publisher_;
    /**
     * @brief Shared pointer to the publisher for publishing the body structure marker
     */
    IPublisher<visualization_msgs::msg::MarkerArray>::SharedPtr body_structure_marker_publisher_;
};

#endif // MARKER_STRUCTURE_PUBLISHER_H