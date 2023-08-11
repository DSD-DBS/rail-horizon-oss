/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_MESSAGE_HELPER_H
#define DSD_RAIL_HORIZON_MESSAGE_HELPER_H

#include "dsd_rail_horizon/utils/StructureConverter.h"

#include <dsd_rail_horizon_core/MapModel.h>

#include <dsd_common_types/GeometryTypes.h>

#include <dsd_ros_messages/msg/application_status.hpp>
#include <dsd_ros_messages/msg/application_status_stamped.hpp>
#include <dsd_ros_messages/msg/id.hpp>
#include <dsd_ros_messages/msg/key_value_pair.hpp>
#include <dsd_ros_messages/msg/object.hpp>
#include <dsd_ros_messages/msg/object_base.hpp>
#include <dsd_ros_messages/msg/object_classification.hpp>
#include <dsd_ros_messages/msg/object_tracking.hpp>
#include <dsd_ros_messages/msg/object_types.hpp>
#include <dsd_ros_messages/msg/shape.hpp>
#include <dsd_ros_messages/msg/shape_types.hpp>
#include <dsd_ros_messages/msg/track.hpp>
#include <dsd_ros_messages/msg/track_point.hpp>

#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>

#include <iterator>

/**
 * @brief Creates a message of type ColorRGBA based on given parameters
 * @param r double value of red (0.0-1.0)
 * @param g double value of green (0.0-1.0)
 * @param b double value of blue (0.0-1.0)
 * @param a double value of alpha channel (by default = 1.0)
 * @return color - message of type std_msgs::msg::ColorRGBA
 */
inline std_msgs::msg::ColorRGBA create_color_message(double r, double g, double b, double a = 1.0)
{
    std_msgs::msg::ColorRGBA color{};
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

/**
 * @brief Creates a message of type Vector3 to be used for scaling, based on given parameters
 * @param x double value of x
 * @param y double value of y
 * @param z double value of z
 * @return scale - message of type geometry_msgs::msg::Vector3
 */
inline geometry_msgs::msg::Vector3 create_scale_message(double x, double y, double z)
{
    geometry_msgs::msg::Vector3 scale;
    scale.x = x;
    scale.y = y;
    scale.z = z;
    return scale;
}

/**
 * @brief Creates a message of type KeyValueMap based on given parameters
 * @param key_value_vector Vector of pairs
 * @return message of type KeyValueMap
 */
inline dsd_ros_messages::msg::KeyValueMap create_key_value_map_message(
    const std::vector<std::pair<std::string, std::string>>& key_value_vector)
{
    dsd_ros_messages::msg::KeyValueMap key_value_map_message{};
    std::transform(begin(key_value_vector), end(key_value_vector), std::back_inserter(key_value_map_message.data),
        [](const auto& pair) {
        dsd_ros_messages::msg::KeyValuePair pair_message{};
        pair_message.key = pair.first;
        pair_message.value = pair.second;
        return pair_message;
    });
    return key_value_map_message;
}

/**
 * @brief Creates a message of type Point
 * @param p constant reference to Point in 3D space
 * @return point - message of type geometry_msgs::msg::Point
 */
template <typename Point>
geometry_msgs::msg::Point create_point_message(const Point& p)
{
    geometry_msgs::msg::Point point{};
    point.x = p.template get<0>();
    point.y = p.template get<1>();
    point.z = p.template get<2>();
    return point;
}

/**
 * @brief Creates a message of type TrackPoint
 * @param p constant reference to Point in 3D space
 * @return point - message of type dsd_ros_messages::msg::TrackPoint
 */
template <typename Point>
dsd_ros_messages::msg::TrackPoint create_track_point_message(const Point& point)
{
    dsd_ros_messages::msg::TrackPoint track_point;
    track_point.position = create_point_message(point);
    return track_point;
}

/**
 * @brief Converts geometry to a vector of point messages
 * This functions should be used only with UTM coordinates
 * @param geometry geometry of an object
 * @return points - a vector which contains geometry points
 */
template <typename GeometryXyz>
std::vector<geometry_msgs::msg::Point> geometry_to_point_vector(const GeometryXyz& geometry)
{
    std::vector<geometry_msgs::msg::Point> points{};
    std::transform(begin(geometry), end(geometry), std::back_inserter(points), [&](const auto& point) {
        return create_point_message(point);
    });
    return points;
}

/**
 * @brief Creates a message of type ID
 * @param id unsigned integer which is identifier
 * @return id_message - message of type dsd_ros_messages::msg::ID
 */
inline dsd_ros_messages::msg::ID create_id_message(uint64_t id)
{
    dsd_ros_messages::msg::ID id_message;
    id_message.id = id;
    return id_message;
}

/**
 * @brief Adds the point \p{point} to the vector \p{parameters}
 * @param parameters vector of doubles that contains the points
 * @param p constant reference to Point in 3D space
 */
template <typename Point>
void add_point_to_shape_parameter(std::vector<double>& parameters, const Point& point)
{
    parameters.push_back(point.template get<0>());
    parameters.push_back(point.template get<1>());
    parameters.push_back(point.template get<2>());
}

/**
 * @brief Adds points from the line \p{line} to the vector \p{parameters}
 * @param parameters vector of doubles that contains the points
 * @param line constant reference to Line in 3D space
 */
template <typename Line>
void add_line_to_shape_parameter(std::vector<double>& parameters, const Line& line)
{
    for (const auto& point : line)
    {
        add_point_to_shape_parameter(parameters, point);
    }
}

/**
 * @brief Create vertical shape message
 * @param bottom_point center (3D point) of the bottom structure
 * @param bottom_radius radius of the bottom structure
 * @param top_point center (3D point) of the top structure
 * @param top_radius radius of the top structure
 * @return landmark_base_shape - message of type dsd_ros_messages::msg::Shape
 */
template <typename Point>
dsd_ros_messages::msg::Shape create_vertical_shape(
    const Point& bottom_point, double bottom_radius, const Point& top_point, double top_radius)
{
    dsd_ros_messages::msg::Shape landmark_base_shape;
    landmark_base_shape.type = dsd_ros_messages::msg::ShapeTypes::SHAPE_TYPE_VERTICAL_STRUCTURE_WITH_RADIUS;
    add_point_to_shape_parameter(landmark_base_shape.parameters, bottom_point);
    landmark_base_shape.parameters.push_back(bottom_radius);
    add_point_to_shape_parameter(landmark_base_shape.parameters, top_point);
    landmark_base_shape.parameters.push_back(top_radius);
    return landmark_base_shape;
}

/**
 * @brief Create plane shape message
 * @param line constant reference to Line in 3D space
 * @return landmark_base_shape - message of type dsd_ros_messages::msg::Shape
 */
template <typename Line>
dsd_ros_messages::msg::Shape create_plane_shape(const Line& line)
{
    dsd_ros_messages::msg::Shape landmark_base_shape;
    landmark_base_shape.type = dsd_ros_messages::msg::ShapeTypes::SHAPE_TYPE_PLANE_STRUCTURE;
    add_line_to_shape_parameter(landmark_base_shape.parameters, line);
    return landmark_base_shape;
}

/**
 * @brief Create body shape message
 * @param line constant reference to Line in 3D space
 * @param height the height of the body structure
 * @return landmark_base_shape - message of type dsd_ros_messages::msg::Shape
 */
template <typename Line>
dsd_ros_messages::msg::Shape create_body_shape(const Line& line, double height)
{
    dsd_ros_messages::msg::Shape landmark_base_shape;
    landmark_base_shape.type = dsd_ros_messages::msg::ShapeTypes::SHAPE_TYPE_BODY_STRUCTURE;
    add_line_to_shape_parameter(landmark_base_shape.parameters, line);
    landmark_base_shape.parameters.push_back(height);
    return landmark_base_shape;
}

/**
 * @brief Create track message
 * @param id unsigned integer which is identifier
 * @param start_id unsigned integer which is identifier of the start track point
 * @param end_id unsigned integer which is identifier of the end track point
 * @param part_of_mission boolean if the track is part of a mission
 * @return track - message of type dsd_ros_messages::msg::Track
 */
template <typename Point>
inline dsd_ros_messages::msg::Track create_track(const Id& id, const Id& start_id, const Id& end_id,
    const std::vector<Point>& virtual_centerline, const std::vector<Point>& left_rail,
    const std::vector<Point>& right_rail, bool part_of_mission)
{
    dsd_ros_messages::msg::Track track;
    track.id.id = id;
    track.part_of_mission = part_of_mission;
    track.optional_attributes = create_key_value_map_message({{"virtual_centerline_start_id", std::to_string(start_id)},
        {"virtual_centerline_end_id", std::to_string(end_id)}});

    std::transform(begin(virtual_centerline), end(virtual_centerline), std::back_inserter(track.virtual_centerline),
        create_track_point_message<Point>);
    std::transform(
        begin(left_rail), end(left_rail), std::back_inserter(track.left_rail), create_track_point_message<Point>);
    std::transform(
        begin(right_rail), end(right_rail), std::back_inserter(track.right_rail), create_track_point_message<Point>);
    return track;
}

/**
 * @brief Create landmark message
 * @param id unsigned integer which is identifier
 * @param landmark_base_shape a reference to the dsd_ros_messages::msg::Shape
 * @param landmark_classification a reference to the dsd_ros_messages::msg::ObjectClassification
 * @return landmark - message of type dsd_ros_messages::msg::Object
 */
inline dsd_ros_messages::msg::Object create_landmark(const Id& id,
    const dsd_ros_messages::msg::Shape& landmark_base_shape,
    const dsd_ros_messages::msg::ObjectClassification& landmark_classification)
{
    dsd_ros_messages::msg::Object landmark;
    landmark.base.shape = landmark_base_shape;

    dsd_ros_messages::msg::ObjectTracking tmp_landmark_tracking;
    tmp_landmark_tracking.id.id = id;
    landmark.tracking.push_back(tmp_landmark_tracking);

    landmark.classification.push_back(landmark_classification);
    return landmark;
}

/**
 * @brief Create vertical landmark message
 * @param id unsigned integer which is identifier
 * @param type type of the vertical structure
 * @param bottom_point center (3D point) of the bottom structure
 * @param bottom_radius radius of the bottom structure
 * @param top_point center (3D point) of the top structure
 * @param top_radius radius of the top structure
 * @return object - message of type dsd_ros_messages::msg::Object
 */
template <typename Point>
dsd_ros_messages::msg::Object create_vertical_landmark(const Id& id, const VerticalStructure::Type& type,
    const Point& bottom_point, double bottom_radius, const Point& top_point, double top_radius)
{
    dsd_ros_messages::msg::ObjectClassification landmark_classification;
    landmark_classification.class_name = dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_STRUCTURE;
    landmark_classification.type_name = convert_vertical_structure_type_to_object_type(type);
    return create_landmark(
        id, create_vertical_shape(bottom_point, bottom_radius, top_point, top_radius), landmark_classification);
}

/**
 * @brief Create body landmark message
 * @param id unsigned integer which is identifier
 * @param type type of the body structure
 * @param line constant reference to Line in 3D space
 * @param height the height of the body structure
 * @return object - message of type dsd_ros_messages::msg::Object
 */
template <typename Line>
dsd_ros_messages::msg::Object create_body_landmark(
    const Id& id, const BodyStructure::Type& type, const Line& line, double height)
{
    dsd_ros_messages::msg::ObjectClassification landmark_classification;
    landmark_classification.class_name = dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_STRUCTURE;
    landmark_classification.type_name = convert_body_structure_type_to_object_type(type);
    return create_landmark(id, create_body_shape(line, height), landmark_classification);
}

/**
 * @brief Create plane landmark message
 * @param id unsigned integer which is identifier
 * @param type type of the plane structure
 * @param line constant reference to Line in 3D space
 * @return object - message of type dsd_ros_messages::msg::Object
 */
template <typename Line>
dsd_ros_messages::msg::Object create_plane_landmark(const Id& id, const PlaneStructure::Type& type, const Line& line)
{
    dsd_ros_messages::msg::ObjectClassification landmark_classification;
    landmark_classification.class_name = dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_STRUCTURE;
    landmark_classification.type_name = convert_plane_structure_type_to_object_type(type);
    return create_landmark(id, create_plane_shape(line), landmark_classification);
}

/**
 * @brief Create zone landmark message
 * @param id unsigned integer which is identifier
 * @param type type of the plane structure
 * @param sub_type sub type of the plane structure
 * @param line constant reference to Line in 3D space
 * @return landmark - message of type dsd_ros_messages::msg::Object
 */
template <typename Line>
dsd_ros_messages::msg::Object create_zone_landmark(
    const Id& id, const PlaneStructure::Type& type, const PlaneStructure::SubType& sub_type, const Line& line)
{
    auto landmark = create_plane_landmark(id, type, line);
    landmark.classification[0].sub_type_name = convert_plane_structure_sub_type_to_object_type(sub_type);
    return landmark;
}

/**
 * @brief Create application status stamped message
 * @param stamp timestamp
 * @param application_status message of type dsd_ros_messages::msg::Object
 * @return message - message of type dsd_ros_messages::msg::ApplicationStatusStamped
 */
inline dsd_ros_messages::msg::ApplicationStatusStamped create_app_status_stamped_message(
    const rclcpp::Time& stamp, const dsd_ros_messages::msg::ApplicationStatus& application_status)
{
    dsd_ros_messages::msg::ApplicationStatusStamped message;
    message.stamp = stamp;
    message.application_status = application_status;

    return message;
}
/**
 * @brief Create application status message
 * @param compute_node_id vector of identifiers of compute nodes
 * @param application_id vector of identifiers of applications
 * @param status vector of modes related to the applications
 * @param sub_status vector of sub modes related to the applications
 * @param comment vector of comments related to applications
 * @return app_status - message of type dsd_ros_messages::msg::ApplicationStatus
 */
inline dsd_ros_messages::msg::ApplicationStatus create_app_status_message(std::vector<std::string> compute_node_id,
    std::vector<std::string> application_id, std::vector<uint8_t> status, std::vector<uint8_t> sub_status,
    std::vector<std::string> comment)
{
    dsd_ros_messages::msg::ApplicationStatus app_status;
    app_status.compute_node_id = compute_node_id;
    app_status.application_id = application_id;
    app_status.status = status;
    app_status.sub_status = sub_status;
    app_status.comment = comment;

    return app_status;
}

#endif // DSD_RAIL_HORIZON_MESSAGE_HELPER_H
