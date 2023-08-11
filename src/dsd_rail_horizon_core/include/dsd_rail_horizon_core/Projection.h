/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_CORE_PROJECTION_H
#define DSD_RAIL_HORIZON_CORE_PROJECTION_H

#include <dsd_rail_horizon_core/interfaces/ILogger.h>

#include <dsd_common_types/GeoGeometryTypes.h>
#include <dsd_common_types/GeometryTypes.h>

#include <proj.h>

#include <memory>
#include <string>


/**
 * @brief Configuration object for \ref Projection class
 */
struct ProjectionConfig
{
    /**
     * @brief Cordinate reference system of the input data
     */
    std::string source_crs;
    /**
     * @brief Cordinate reference system of the projected output data
     */
    std::string target_crs;
};

/**
 * @brief Projection class allows transforming objects with wsg84 coordinates to objects with utm coordinates
 */
class Projection
{
public:
    /**
     * @brief Create the projection class and set up the projections
     * @param config Configuration objection for projections
     * @param logger Logger instance for reporting of errors during projection setup
     */
    Projection(const ProjectionConfig& config, ILogger::SharedPtr logger);

    /**
     * @brief Checks if there has been some error during projection setup and in this case tries to redo the setup
     */
    void create_projections_on_error();

    /**
     * @brief Transform data from WGS84 to UTM coordinate system
     * @param ls_wgs84 linestring in WGS84 coordinate system
     * @return LinestringXyz linestring in UTM coordinate system
     */
    LineStringXyz transform_linestring_wgs84_to_utm(const LineStringXyzWgs84& ls_wgs84);

    /**
     * @brief Transform data from WGS84 to UTM coordinate system
     * @param p_wgs84 polygon in WGS84 coordinate system
     * @return PolygonXyz polygon in UTM coordinate system
     */
    PolygonXyz transform_polygon_wgs84_to_utm(const PolygonXyzWgs84& p_wgs84);

    /**
     * @brief Transform data from WGS84 to UTM coordinate system
     * @param point_wsg84 point in WGS84 coordinate system
     * @return PointXyz point in UTM coordinate system
     */
    PointXyz transform_point_wgs84_to_utm(const PointXyzWgs84& point_wsg84);

    /**
     * @brief Get the current error status
     *
     * @return The current error message
     */
    [[nodiscard]] const std::string& get_error_status() const;

    /**
     * @brief Type alias for shared ptr on a mutable object of this class
     */
    using SharedPtr = std::shared_ptr<Projection>;

    /**
     * @brief Type alias for shared ptr on a const object of this class
     */
    using ConstSharedPtr = std::shared_ptr<const Projection>;

private:
    /**
     * @brief Sets up the projections
     */
    void create_projections();

    /**
     * @brief Sets the error status and loogs it
     */
    void set_error_status(const std::string& error_status);

    /**
     * @brief Configuration of this class
     */
    ProjectionConfig config;

    /**
     * @brief Logger instance to log errors
     */
    ILogger::SharedPtr logger_;

    /**
     * @brief Current error status of the projection setup
     */
    std::string error_status_;

    /**
     * @brief Thread context for projections
     */
    std::shared_ptr<PJ_CONTEXT> proj_context_{nullptr, &proj_context_destroy};

    /**
     * @brief Projection
     */
    std::shared_ptr<PJ> projection_{nullptr, &proj_destroy};

    /**
     * @brief Normalized projection
     */
    std::shared_ptr<PJ> projection_normalized_{nullptr, &proj_destroy};
};

/**
 * @brief Helper function to create config for projection rom WGS84 (GPS) to WGS84-UTM Zone 32N
 * @param logger Logger instance
 * @param use_geoid_for_projection Whether geoid or ellipsoid should be used for the projection
 */
inline ProjectionConfig create_wsg84_to_utm_config(ILogger::SharedPtr logger, bool use_geoid_for_projection)
{
    if (use_geoid_for_projection)
    {
        logger->info("Using EGM96 geoid for projecting data");
        return ProjectionConfig{"EPSG:4326", "+proj=utm +zone=32 +datum=WGS84 +geoidgrids=egm96_15.gtx"};
    }
    logger->info("Using ellipsoid for projecting data");
    return ProjectionConfig{"EPSG:4326", "+proj=utm +zone=32 +datum=WGS84"};
}

#endif // DSD_RAIL_HORIZON_CORE_PROJECTION_H