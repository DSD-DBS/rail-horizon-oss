/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_RAIL_HORIZON_PUBLISHER_H
#define DSD_RAIL_HORIZON_RAIL_HORIZON_PUBLISHER_H

#include "dsd_rail_horizon/interfaces/IPublisher.h"
#include "dsd_rail_horizon/utils/CalculationHelper.h"

#include <dsd_rail_horizon_core/LookupStructures.h>

#include <dsd_common_types/GeometryTypes.h>

#include <dsd_ros_messages/msg/coupled_localization_stamped.hpp>
#include <dsd_ros_messages/msg/rail_horizon_data.hpp>
#include <dsd_ros_messages/msg/rail_horizon_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

/**
 * @brief Configuration for class \ref RailHorizonPublisher
 */
struct RailHorizonPublisherConfig
{
    /**
     * @brief Topic name of rail horizon publisher
     */
    std::string publisher_topic_name;
    /**
     * @brief Quality of service setting for publisher
     */
    rclcpp::QoS qos;
    /**
     * @brief Frame id which is included in rail horizon message
     */
    std::string rail_horizon_frame_id;
    /**
     * @brief Whether global horizon with structures in WSG84 should be published
     */
    bool publish_global_horizon;
    /**
     * @brief Whether local horizon with local structures in utm should be published
     */
    bool publish_local_horizon;
    /**
     * @brief Whether virtual centerline track information should be included
     */
    bool rail_horizon_virtual_centerline;
    /**
     * @brief Whether left and right rail track information should be included
     */
    bool rail_horizon_left_right_rail;
};

/**
 * @brief Publisher for global and local structures near the train
 */
class RailHorizonPublisher
{
public:
    /**
     * @brief Constructor for RailHorizonPublisher
     *
     * @param parent_node The node this published is associated with
     * @param config Config for this class
     */
    RailHorizonPublisher(rclcpp::Node& parent_node, const RailHorizonPublisherConfig& config);

    /**
     * @brief Publishes RailHorizonMessage
     *
     * @param structures Container of structures near the train which should be published
     * @param local_transformation Transformation which encodes the positon and rotation of the train to get the
     * structures position relative to the train
     * @param coupled_localization_message Message from coupled localization publisher which is included in this message
     * as well
     * @param track_ids List of track ids of the current mission
     * @param map_version Currently used map version
     */
    void publish(const StructureContainer& structures, const GeometryTransformation& local_transformation,
        const dsd_ros_messages::msg::CoupledLocalizationStamped& coupled_localization_message,
        const std::vector<Id>& track_ids, const std::string& map_version);

protected:
    /**
     * @brief Type alias for message sent by publisher
     */
    using RailHorizonStampedMessage = dsd_ros_messages::msg::RailHorizonStamped;

    /**
     * @brief Type alias for the global and local horizon message
     */
    using RailHorizonDataMessage = dsd_ros_messages::msg::RailHorizonData;

    /**
     * @brief Construct the global horizon data message based on the given structures
     *
     * @param structures The structures which should be added to the message
     * @param track_ids List of track ids of the current mission
     */
    RailHorizonDataMessage fill_global_horizon(const StructureContainer& structures, const std::vector<Id>& track_ids);

    /**
     * @brief Construct the lcaol horizon data message based on the given structures and the given transformation
     *
     * @param structures The structures which should be added to the message
     * @param track_ids List of track ids of the current mission
     * @param local_transformation Transformation used to make the position of the structures relative to the train
     */
    RailHorizonDataMessage fill_local_horizon(const StructureContainer& structures, const std::vector<Id>& track_ids,
        const GeometryTransformation& local_transformation);

    /**
     * @brief Adding vertical structures to the global horizon
     *
     * @param global_horizon Structures within the map-foresight in WGS84 coordinate system
     * @param structures Contant reference to the vector of vertical structure pointers
     */
    void add_vertical_structures_to_global_horizon(
        RailHorizonDataMessage& global_horizon, const std::vector<VerticalStructure::Ptr>& structures);
    /**
     * @brief Adding horizontal structures to the global horizon
     *
     * @param global_horizon Structures within the map-foresight in WGS84 coordinate system
     * @param structures Contant reference to the vector of horizontal structure pointers
     * @param track_ids List of track ids of the current mission
     */
    void add_horizontal_structures_to_global_horizon(RailHorizonDataMessage& global_horizon,
        const std::vector<HorizontalStructure::Ptr>& structures, const std::vector<Id>& track_ids);
    /**
     * @brief Adding body structures to the global horizon
     *
     * @param global_horizon Structures within the map-foresight in WGS84 coordinate system
     * @param structures Contant reference to the vector of body structure pointers
     */
    void add_body_structures_to_global_horizon(
        RailHorizonDataMessage& global_horizon, const std::vector<BodyStructure::Ptr>& structures);
    /**
     * @brief Adding plane structures to the global horizon
     *
     * @param global_horizon Structures within the map-foresight in WGS84 coordinate system
     * @param structures Contant reference to the vector of plane structure pointers
     */
    void add_plane_structures_to_global_horizon(
        RailHorizonDataMessage& global_horizon, const std::vector<PlaneStructure::Ptr>& structures);
    /**
     * @brief Adding vertical structures to the local horizon
     *
     * @param local_horizon Structures within the map-foresight in UTM coordinate system relative to the current train
     * position
     * @param structures Contant reference to the vector of vertical structure pointers
     * @param local_transformation Transformation used to make the position of the structures relative to the train
     */
    void add_vertical_structures_to_local_horizon(RailHorizonDataMessage& local_horizon,
        const std::vector<VerticalStructure::Ptr>& structures, const GeometryTransformation& local_transformation);
    /**
     * @brief Adding horizontal structures to the local horizon
     *
     * @param local_horizon Structures within the map-foresight in UTM coordinate system relative to the current train
     * position
     * @param structures Contant reference to the vector of horizontal structure pointers
     * @param track_ids List of track ids of the current mission
     * @param local_transformation Transformation used to make the position of the structures relative to the train
     */
    void add_horizontal_structures_to_local_horizon(RailHorizonDataMessage& local_horizon,
        const std::vector<HorizontalStructure::Ptr>& structures, const std::vector<Id>& track_ids,
        const GeometryTransformation& local_transformation);
    /**
     * @brief Adding body structures to the local horizon
     *
     * @param local_horizon Structures within the map-foresight in UTM coordinate system relative to the current train
     * position
     * @param structures Contant reference to the vector of body structure pointers
     * @param local_transformation Transformation used to make the position of the structures relative to the train
     */
    void add_body_structures_to_local_horizon(RailHorizonDataMessage& local_horizon,
        const std::vector<BodyStructure::Ptr>& structures, const GeometryTransformation& local_transformation);
    /**
     * @brief Adding plane structures to the local horizon
     *
     * @param local_horizon Structures within the map-foresight in UTM coordinate system relative to the current train
     * position
     * @param structures Contant reference to the vector of plane structure pointers
     * @param local_transformation Transformation used to make the position of the structures relative to the train
     */
    void add_plane_structures_to_local_horizon(RailHorizonDataMessage& local_horizon,
        const std::vector<PlaneStructure::Ptr>& structures, const GeometryTransformation& local_transformation);

    /**
     * @brief Config of the publisher
     */
    RailHorizonPublisherConfig config_;

    /**
     * @brief The ROS2 published used for publishing the rail horizon message
     */
    IPublisher<RailHorizonStampedMessage>::SharedPtr rail_horizon_publisher_;
};

#endif // DSD_RAIL_HORIZON_RAIL_HORIZON_PUBLISHER_H