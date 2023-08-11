/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_CORE_MAP_MODEL_H
#define DSD_RAIL_HORIZON_CORE_MAP_MODEL_H

#include <dsd_common_types/GeoGeometryTypes.h>
/**
 * @brief Horizontal Structure object containig track informaton
 */
struct HorizontalStructure
{
    /**
     * @brief Type alias for shared ptr on a const object of this class
     */
    using Ptr = std::shared_ptr<const HorizontalStructure>;
    /**
     * @brief Type alias for shared ptr on a mutable object of this class
     */
    using MutablePtr = std::shared_ptr<HorizontalStructure>;

    /**
     * @brief Unique track identifier
     */
    Id id_{};
    /**
     * @brief Track start node identifier
     */
    Id start_node_id_{};
    /**
     * @brief Track end node identifier
     */
    Id end_node_id_{};
    /**
     * @brief Virtual centerline of a track in WGS84 coordinate system
     */
    LineStringXyzWgs84 centerline_geometry_;
    /**
     * @brief Track lenght in meters
     */
    double track_length_in_meters_{};
    /**
     * @brief Left rail of a track in WGS84 coordinate system
     */
    LineStringXyzWgs84 left_rail_geometry_;
    /**
     * @brief Right rail of a track in WGS84 coordinate system
     */
    LineStringXyzWgs84 right_rail_geometry_;
    /**
     * @brief Virtual centerline of a track in UTM coordinate system
     */
    LineStringXyz centerline_geometry_utm_;
    /**
     * @brief Left rail of a track in UTM coordinate system
     */
    LineStringXyz left_rail_geometry_utm_;
    /**
     * @brief Right rail of a track in UTM coordinate system
     */
    LineStringXyz right_rail_geometry_utm_;
};
/**
 * @brief Vertical Structure object
 */
struct VerticalStructure
{
    /**
     * @brief Type alias for shared ptr on a const object of this class
     */
    using Ptr = std::shared_ptr<const VerticalStructure>;
    /**
     * @brief Type alias for shared ptr on a mutable object of this class
     */
    using MutablePtr = std::shared_ptr<VerticalStructure>;
    /**
     * @brief Enumeration Types that contain vertical structure types
     */
    enum Type
    {
        UNDEFINED = 0,
        LIGHT,
        SIGN,
        SIGNAL,
        CATENARY,
        PILLAR,
        CAMERA,
        POLE_INFORMATION,
        POLE_SHELTER,
        POLE_BOARD_INFORMATION,
        BAR_BARRIER
    };

    /**
     * @brief Unique identifier of a vertical structure
     */
    Id id_{};
    /**
     * @brief Type of a vertical structure
     */
    Type type_{};
    /**
     * @brief Line in WGS84 coordinate system starting from the bottom and ending at the top of a vertical structure
     */
    LineStringXyzWgs84 bottom_top_line_;
    /**
     * @brief Line in UTM coordinate system starting from the bottom and ending at the top of a vertical structure
     */
    LineStringXyz bottom_top_line_utm_;
    /**
     * @brief Bottom radius of a vertical structure
     */
    double bottom_radius_{};
    /**
     * @brief Top radius of a vertical structure
     */
    double top_radius_{};
};

/**
 * @brief Plane Structure object
 */
struct PlaneStructure
{
    /**
     * @brief Type alias for shared ptr on a const object of this class
     */
    using Ptr = std::shared_ptr<const PlaneStructure>;
    /**
     * @brief Type alias for shared ptr on a mutable object of this class
     */
    using MutablePtr = std::shared_ptr<PlaneStructure>;
    /**
     * @brief Enumeration Types that contain plane structure types
     */
    enum Type
    {
        TYPE_UNDEFINED = 0,
        WALL,
        BILLBOARD,
        ZONE,
        TUNNEL,
    };
    /**
     * @brief Enumeration Types that contain plane structure subtypes
     */
    enum SubType
    {
        SUBTYPE_UNDEFINED = 0,
        SAFE_ZONE,
        RISK_ZONE,
        TRACK_ZONE,
        NEAR_TRACK_ZONE,
        ENVIRONMENT_ZONE,
        LEVEL_CROSSING_ZONE,
    };
    /**
     * @brief Unique identifier of a plane structure
     */
    Id id_{};
    /**
     * @brief Polygon in WGS84 coordinate system describing the plane structure
     */
    PolygonXyzWgs84 geometry_;
    /**
     * @brief Polygon in UTM coordinate system describing the plane structure
     */
    PolygonXyz geometry_utm_;
    /**
     * @brief Type of a plane structure
     */
    Type type_{};
    /**
     * @brief SubType of a plane structure
     */
    SubType sub_type_{};
};

/**
 * @brief Body Structure object
 */
struct BodyStructure
{
    /**
     * @brief Type alias for shared ptr on a const object of this class
     */
    using Ptr = std::shared_ptr<const BodyStructure>;
    /**
     * @brief Type alias for shared ptr on a mutable object of this class
     */
    using MutablePtr = std::shared_ptr<BodyStructure>;
    /**
     * @brief Enumeration Types that contain body structure types
     */
    enum Type
    {
        UNDEFINED = 0,
        FUSE_BOX,
        SWITCH_CABINET,
        BUILDING,
        PLATFORM,
        TRASH_CAN,
        CONTAINER,
        FOUNDATION,
        BARRIER_BODY,
    };
    /**
     * @brief Unique identifier of a body structure
     */
    Id id_{};
    /**
     * @brief Polygon in WGS84 coordinate system describing the body structure
     */
    PolygonXyzWgs84 geometry_;
    /**
     * @brief Polygon in UTM coordinate system describing the body structure
     */
    PolygonXyz geometry_utm_;
    /**
     * @brief Height of a plane structure
     */
    double height_{};
    /**
     * @brief Type of a plane structure
     */
    Type type_{};
};
#endif // DSD_RAIL_HORIZON_CORE_MAP_MODEL_H
