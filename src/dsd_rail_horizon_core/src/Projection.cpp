/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dsd_rail_horizon_core/Projection.h"

#include <dsd_rail_horizon_core/interfaces/ILogger.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include <vector>

Projection::Projection(const ProjectionConfig& config, ILogger::SharedPtr logger) : config{config}, logger_{logger}
{
    create_projections();
}

void Projection::create_projections_on_error()
{
    if (!error_status_.empty())
    {
        create_projections();
        logger_->info("Create projection context and projections upon error");
    }
}

LineStringXyz Projection::transform_linestring_wgs84_to_utm(const LineStringXyzWgs84& ls_wgs84)
{
    LineStringXyz ls_utm;
    for (const auto& ls : ls_wgs84)
    {
        double x = ls.get<0>();
        double y = ls.get<1>();
        double z = ls.get<2>();
        // logger_->info(boost::format("WGS84: %1% %2% %3%") % x % y % z);

        if (projection_normalized_)
        {
            proj_trans_generic(projection_normalized_.get(), PJ_FWD, &x, sizeof(double), 1, &y, sizeof(double), 1, &z,
                sizeof(double), 1, nullptr, 0, 0);
        }
        else
        {
            set_error_status("Cannot reproject data. As setting up transform failed.");
        }

        boost::geometry::append(ls_utm, PointXyz{x, y, z});
        // logger_->info(boost::format("UTM: %1% %2% %3%") % x % y % z);
    }
    return ls_utm;
}

PolygonXyz Projection::transform_polygon_wgs84_to_utm(const PolygonXyzWgs84& p_wgs84)
{
    PolygonXyz p_utm;
    for (const auto& p_ptr : p_wgs84.outer())
    {
        double x = p_ptr.get<0>();
        double y = p_ptr.get<1>();
        double z = p_ptr.get<2>();
        // logger_->info(boost::format("WGS84: %1% %2% %3%") % x % y % z);

        if (projection_normalized_)
        {
            proj_trans_generic(projection_normalized_.get(), PJ_FWD, &x, sizeof(double), 1, &y, sizeof(double), 1, &z,
                sizeof(double), 1, nullptr, 0, 0);
        }
        else
        {
            set_error_status("Cannot reproject data. As setting up transform failed.");
        }

        boost::geometry::append(p_utm, PointXyz{x, y, z});
        // logger_->info(boost::format("UTM: %1% %2% %3%") % x % y % z);
    }
    return p_utm;
}

PointXyz Projection::transform_point_wgs84_to_utm(const PointXyzWgs84& point_wsg84)
{
    double x = point_wsg84.get<0>();
    double y = point_wsg84.get<1>();
    double z = point_wsg84.get<2>();

    if (projection_normalized_)
    {
        // TODO(leon): proj_trans can perhaps be used in all places? Should be tested with some test cases
        proj_trans_generic(projection_normalized_.get(), PJ_FWD, &x, sizeof(double), 1, &y, sizeof(double), 1, &z,
            sizeof(double), 1, nullptr, 2, 0);
    }
    else
    {
        set_error_status("Cannot reproject data. As setting up transform failed.");
    }

    return {x, y, z};
}

[[nodiscard]] const std::string& Projection::get_error_status() const
{
    return error_status_;
}

void Projection::create_projections()
{
    proj_context_.reset(proj_context_create(), &proj_context_destroy);
    error_status_ = "";

    projection_.reset(
        proj_create_crs_to_crs(proj_context_.get(), config.source_crs.c_str(), config.target_crs.c_str(), nullptr),
        &proj_destroy);

    if (!projection_)
    {
        set_error_status("Could not set-up transform.");
        return;
    }

    projection_normalized_.reset(
        proj_normalize_for_visualization(proj_context_.get(), projection_.get()), &proj_destroy);
    if (!projection_normalized_)
    {
        set_error_status("Could not set-up transform.");
        return;
    }
}

void Projection::set_error_status(const std::string& error_status)
{
    error_status_ = error_status;
    logger_->error(error_status);
}