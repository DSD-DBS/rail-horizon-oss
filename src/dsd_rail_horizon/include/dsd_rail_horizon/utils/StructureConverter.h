/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_STRUCTURE_CONVERTER_H
#define DSD_RAIL_HORIZON_STRUCTURE_CONVERTER_H

#include <dsd_rail_horizon_core/MapModel.h>

#include <dsd_ros_messages/msg/object_types.hpp>

/**
 * @brief Converts vertical structure type to an object type
 * @param type Type of Vertical Structure
 * @return object type
 */
inline uint8_t convert_vertical_structure_type_to_object_type(VerticalStructure::Type type)
{
    switch (type)
    {
        case VerticalStructure::Type::LIGHT:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_LIGHT;
        case VerticalStructure::Type::SIGN:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_SIGN;
        case VerticalStructure::Type::SIGNAL:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_SIGNAL;
        case VerticalStructure::Type::CATENARY:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_CATENARY;
        case VerticalStructure::Type::PILLAR:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_PILLAR;
        case VerticalStructure::Type::CAMERA:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_CAMERA;
        case VerticalStructure::Type::POLE_INFORMATION:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_POLE_INFORMATION;
        case VerticalStructure::Type::POLE_SHELTER:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_POLE_SHELTER;
        case VerticalStructure::Type::POLE_BOARD_INFORMATION:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_POLE_BOARD_INFORMATION;
        case VerticalStructure::Type::BAR_BARRIER:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_BAR_BARRIER;
        default:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_VERTICAL_TYPE_UNDEFINED;
    }
}
/**
 * @brief Converts body structure type to an object type
 * @param type Type of Body Structure
 * @return object type
 */
inline uint8_t convert_body_structure_type_to_object_type(BodyStructure::Type type)
{
    switch (type)
    {
        case BodyStructure::Type::FUSE_BOX:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_TYPE_FUSE_BOX;
        case BodyStructure::Type::SWITCH_CABINET:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_TYPE_SWITCH_CABINET;
        case BodyStructure::Type::BUILDING:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_TYPE_BUILDING;
        case BodyStructure::Type::PLATFORM:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_TYPE_PLATFORM;
        case BodyStructure::Type::TRASH_CAN:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_TYPE_TRASH_CAN;
        case BodyStructure::Type::CONTAINER:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_TYPE_CONTAINER;
        case BodyStructure::Type::FOUNDATION:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_TYPE_FOUNDATION;
        case BodyStructure::Type::BARRIER_BODY:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_TYPE_BARRIER_BODY;
        default:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_BODY_TYPE_UNDEFINED;
    }
}
/**
 * @brief Converts plane structure type to an object type
 * @param type Type of Plane Structure
 * @return object type
 */
inline uint8_t convert_plane_structure_type_to_object_type(PlaneStructure::Type type)
{
    switch (type)
    {
        case PlaneStructure::Type::WALL:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_TYPE_WALL;
        case PlaneStructure::Type::BILLBOARD:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_TYPE_BILLBOARD;
        case PlaneStructure::Type::TUNNEL:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_TYPE_TUNNEL;
        case PlaneStructure::Type::ZONE:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_TYPE_ZONE;
        default:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_TYPE_UNDEFINED;
    }
}
/**
 * @brief Converts plane structure sub type to an object type
 * @param type Subtype of Plane Structure
 * @return object type
 */
inline uint8_t convert_plane_structure_sub_type_to_object_type(PlaneStructure::SubType type)
{
    switch (type)
    {
        case PlaneStructure::SubType::SAFE_ZONE:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_SUBTYPE_SAFE_ZONE;
        case PlaneStructure::SubType::RISK_ZONE:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_SUBTYPE_RISK_ZONE;
        case PlaneStructure::SubType::TRACK_ZONE:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_SUBTYPE_TRACK_ZONE;
        case PlaneStructure::SubType::NEAR_TRACK_ZONE:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_SUBTYPE_NEAR_TRACK_ZONE;
        case PlaneStructure::SubType::ENVIRONMENT_ZONE:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_SUBTYPE_ENVIRONMENT_ZONE;
        case PlaneStructure::SubType::LEVEL_CROSSING_ZONE:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_SUBTYPE_LEVEL_CROSSING_ZONE;
        default:
            return dsd_ros_messages::msg::ObjectTypes::STATIC_OBJECT_CLASS_PLANE_SUBTYPE_UNDEFINED;
    }
}

#endif // DSD_RAIL_HORIZON_STRUCTURE_CONVERTER_H
