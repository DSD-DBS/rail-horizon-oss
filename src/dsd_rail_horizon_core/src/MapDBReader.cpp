/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dsd_rail_horizon_core/MapDBReader.h"

#include "dsd_rail_horizon_core/interfaces/ILogger.h"
#include "dsd_rail_horizon_core/Projection.h"

#include <dsd_rail_horizon_core/LookupStructures.h>
#include <dsd_rail_horizon_core/MapGeometryTypes.h>
#include <dsd_rail_horizon_core/MapModel.h>

#include <dsd_common_types/GeoGeometryTypes.h>
#include <dsd_common_types/GeometryTypes.h>

#include <dbs-map-api/CommonTypes.h>
#include <dbs-map-api/MapService.h>
#include <dbs-map-api/model/ConsolidatedLayers.h>
#include <dbs-map-api/model/Landmarks.h>
#include <dbs-map-api/model/RcaTopology.h>
#include <dbs-map-api/model/TrackEdge.h>
#include <dbs-map-api/model/TrackNode.h>
#include <dbs-map-api/model/Zone.h>

#include <boost/format.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

MapDBReader::MapDBReader(const MapDBReaderConfig& config, Projection::SharedPtr projection, ILogger::SharedPtr logger)
: projection_{projection}, logger_{logger}, map_service_{create_map_service(config, logger)}, config_{config}
{
}

bool MapDBReader::update_map_over_the_air()
{
    if (!config_.db_requested_map_version)
    {
        logger_->error(boost::format("Map update over the air was requested, but map version not set. Using local map "
                                     "version: %1% instead") %
                       map_service_->GetLocalMapVersion());
        return false;
    }

    auto download_errors = map_service_->UpdateMap(config_.db_requested_map_version.value());
    if (download_errors.empty())
    {
        return true;
    }

    for (const auto& err : download_errors)
    {
        logger_->error(boost::format("Error message: %1%") % err.msg_);
        if (err.error_code_ == dbs_map::download::ErrorCode::CurlError)
        {
            logger_->error(boost::format("Curl Error: %1%") % err.curl_code_);
        }
    }
    return false;
}

LookupStructures::Ptr MapDBReader::read(const std::vector<double>& map_roi)
{
    // TODO(leon): Check where to put this
    if (map_service_->GetLocalMapVersion() == 0U)
    {
        logger_->error("No local map available.");
        return nullptr;
    }

    auto south_west = dbs_map::GeoCoordinates{map_roi[0], map_roi[1]};
    auto north_east = dbs_map::GeoCoordinates{map_roi[2], map_roi[3]};
    const auto result = map_service_->GetLayersForRectangle({south_west, north_east});

    auto lookup_structures = std::make_shared<LookupStructures>();
    read_landmarks_from_map(result, lookup_structures);
    read_topology_from_map(result, lookup_structures);
    read_zones_from_map(result, lookup_structures);

    logger_->info(boost::format("Added %1% vertical structures to index.") %
                  lookup_structures->rtree_vertical_structures_->size());
    logger_->info(boost::format("Added %1% horizontal structures to index.") %
                  lookup_structures->rtree_horizontal_structures_->size());
    logger_->info(
        boost::format("Added %1% plane structures to index.") % lookup_structures->rtree_plane_structures_->size());
    logger_->info(
        boost::format("Added %1% body structures to index.") % lookup_structures->rtree_body_structures_->size());

    return lookup_structures;
}

uint64_t MapDBReader::get_local_map_version()
{
    return map_service_->GetLocalMapVersion();
}

void MapDBReader::add_vertical_structure(dbs_map::model::Landmark::Ptr landmark,
    LookupStructures::MutablePtr lookup_structures, VerticalStructure::Type type)
{
    /* (0) Create vertical structure and fill its members with the given function arguments */
    auto tmp_vertical_struct = std::make_shared<VerticalStructure>();
    tmp_vertical_struct->id_ = std::hash<std::string>{}(landmark->id_);
    tmp_vertical_struct->bottom_top_line_ = std::get<dbs_map::Polyline3d>(landmark->geometry_);
    tmp_vertical_struct->type_ = type;

    /* (1) Project data to metric CS */
    tmp_vertical_struct->bottom_top_line_utm_ =
        projection_->transform_linestring_wgs84_to_utm(tmp_vertical_struct->bottom_top_line_);

    /* (2) Inject data to rtree */
    Box tmp_box = boost::geometry::return_envelope<Box>(tmp_vertical_struct->bottom_top_line_utm_);
    lookup_structures->rtree_vertical_structures_->insert({tmp_box, tmp_vertical_struct->id_});

    /* (3) Inject data to lookup map */
    lookup_structures->lookup_map_id_vertical_structures_.emplace(tmp_vertical_struct->id_, tmp_vertical_struct);
}

void MapDBReader::add_plane_structure(
    dbs_map::model::Landmark::Ptr landmark, LookupStructures::MutablePtr lookup_structures, PlaneStructure::Type type)
{
    /* (0) Create plane structure and fill its members with the given function arguments */
    auto tmp_plane_struct = std::make_shared<PlaneStructure>();
    tmp_plane_struct->geometry_ = std::get<dbs_map::Polygon3d>(landmark->geometry_);
    tmp_plane_struct->id_ = std::hash<std::string>{}(landmark->id_);
    tmp_plane_struct->type_ = type;

    /* (1) Project data to metric CS */
    tmp_plane_struct->geometry_utm_ = projection_->transform_polygon_wgs84_to_utm(tmp_plane_struct->geometry_);

    /* (2) Inject data to rtree */
    Box tmp_box = boost::geometry::return_envelope<Box>(tmp_plane_struct->geometry_utm_);
    lookup_structures->rtree_plane_structures_->insert({tmp_box, tmp_plane_struct->id_});

    /* (3) Inject data to lookup map */
    lookup_structures->lookup_map_id_plane_structures_.emplace(tmp_plane_struct->id_, tmp_plane_struct);
}

void MapDBReader::add_body_structure(
    dbs_map::model::Landmark::Ptr landmark, LookupStructures::MutablePtr lookup_structures, BodyStructure::Type type)
{
    /* (0) Create body structure and fill its members with the given function arguments */
    auto tmp_body_struct = std::make_shared<BodyStructure>();
    const auto geometry = std::get<dbs_map::model::SurfaceWithHeight>(landmark->geometry_);
    tmp_body_struct->geometry_ = geometry.surface_;
    tmp_body_struct->height_ = geometry.height_cm;
    tmp_body_struct->id_ = std::hash<std::string>{}(landmark->id_);
    tmp_body_struct->type_ = type;

    /* (1) Project data to metric CS */
    tmp_body_struct->geometry_utm_ = projection_->transform_polygon_wgs84_to_utm(tmp_body_struct->geometry_);

    /* (2) Inject data to rtree */
    Box tmp_box = boost::geometry::return_envelope<Box>(tmp_body_struct->geometry_utm_);
    lookup_structures->rtree_body_structures_->insert({tmp_box, tmp_body_struct->id_});

    /* (3) Inject data to lookup map */
    lookup_structures->lookup_map_id_body_structures_.emplace(tmp_body_struct->id_, tmp_body_struct);
}

void MapDBReader::read_landmarks_from_map(
    dbs_map::model::ConsolidatedLayers::Ptr consolidated_layers, LookupStructures::MutablePtr lookup_structures)
{
    /* (0) Extract landmarks from map*/
    for (const auto& landmarks_ptr : consolidated_layers->landmarks_)
    {
        switch (landmarks_ptr->type_)
        {
            case dbs_map::model::Landmark::Type::CatenaryPole:
                add_vertical_structure(landmarks_ptr, lookup_structures, VerticalStructure::Type::CATENARY);
                break;
            case dbs_map::model::Landmark::Type::Platform:
                add_body_structure(landmarks_ptr, lookup_structures, BodyStructure::Type::PLATFORM);
                break;
            case dbs_map::model::Landmark::Type::Wall:
                add_plane_structure(landmarks_ptr, lookup_structures, PlaneStructure::Type::WALL);
                break;
            case dbs_map::model::Landmark::Type::SignPole:
                add_vertical_structure(landmarks_ptr, lookup_structures, VerticalStructure::Type::SIGN);
                break;
            case dbs_map::model::Landmark::Type::SignalPole:
                add_vertical_structure(landmarks_ptr, lookup_structures, VerticalStructure::Type::SIGNAL);
                break;
            case dbs_map::model::Landmark::Type::Billboard:
                add_plane_structure(landmarks_ptr, lookup_structures, PlaneStructure::Type::BILLBOARD);
                break;
            case dbs_map::model::Landmark::Type::BillboardPole:
                add_vertical_structure(
                    landmarks_ptr, lookup_structures, VerticalStructure::Type::POLE_BOARD_INFORMATION);
                break;
            case dbs_map::model::Landmark::Type::BridgePillar:
                add_vertical_structure(landmarks_ptr, lookup_structures, VerticalStructure::Type::PILLAR);
                break;
            case dbs_map::model::Landmark::Type::Building:
                add_body_structure(landmarks_ptr, lookup_structures, BodyStructure::Type::BUILDING);
                break;
            case dbs_map::model::Landmark::Type::CameraPole:
                add_vertical_structure(landmarks_ptr, lookup_structures, VerticalStructure::Type::CAMERA);
                break;
            case dbs_map::model::Landmark::Type::Container:
                add_body_structure(landmarks_ptr, lookup_structures, BodyStructure::Type::CONTAINER);
                break;
            case dbs_map::model::Landmark::Type::CoveredPlatform:
                add_vertical_structure(landmarks_ptr, lookup_structures, VerticalStructure::Type::PILLAR);
                break;
            case dbs_map::model::Landmark::Type::FuseBox:
                add_body_structure(landmarks_ptr, lookup_structures, BodyStructure::Type::FUSE_BOX);
                break;
            case dbs_map::model::Landmark::Type::LightPole:
                add_vertical_structure(landmarks_ptr, lookup_structures, VerticalStructure::Type::LIGHT);
                break;
            case dbs_map::model::Landmark::Type::OtherPole:
                add_vertical_structure(landmarks_ptr, lookup_structures, VerticalStructure::Type::UNDEFINED);
                break;
            case dbs_map::model::Landmark::Type::Shelter:
                add_vertical_structure(landmarks_ptr, lookup_structures, VerticalStructure::Type::POLE_SHELTER);
                break;
            case dbs_map::model::Landmark::Type::TrashCan:
                add_body_structure(landmarks_ptr, lookup_structures, BodyStructure::Type::TRASH_CAN);
                break;
            case dbs_map::model::Landmark::Type::TunnelWall:
                add_plane_structure(landmarks_ptr, lookup_structures, PlaneStructure::Type::TUNNEL);
                break;
            case dbs_map::model::Landmark::Type::BarrierBar:
                add_vertical_structure(landmarks_ptr, lookup_structures, VerticalStructure::Type::BAR_BARRIER);
                break;
            case dbs_map::model::Landmark::Type::BarrierBody:
                add_body_structure(landmarks_ptr, lookup_structures, BodyStructure::Type::BARRIER_BODY);
                break;
            default:
                break;
        }
    }
    logger_->info("Landmarks read.");
}

void MapDBReader::read_topology_from_map(
    dbs_map::model::ConsolidatedLayers::Ptr consolidated_layers, LookupStructures::MutablePtr lookup_structures)
{
    /* (0) Extract topology from map*/
    for (const auto& edge_ptr : consolidated_layers->topology_->edges_)
    {
        auto tmp_horizontal_struct = std::make_shared<HorizontalStructure>();
        tmp_horizontal_struct->id_ = std::hash<std::string>{}(edge_ptr->id_);
        tmp_horizontal_struct->start_node_id_ = std::hash<std::string>{}(edge_ptr->side_a_->id_);
        tmp_horizontal_struct->end_node_id_ = std::hash<std::string>{}(edge_ptr->side_b_->id_);
        tmp_horizontal_struct->centerline_geometry_ = edge_ptr->geometry_;

        /* (1) Project data to metric CS */
        tmp_horizontal_struct->centerline_geometry_utm_ =
            projection_->transform_linestring_wgs84_to_utm(tmp_horizontal_struct->centerline_geometry_);

        /* (2) Inject data to rtree */
        Box tmp_box = boost::geometry::return_envelope<Box>(tmp_horizontal_struct->centerline_geometry_utm_);
        lookup_structures->rtree_horizontal_structures_->insert({tmp_box, tmp_horizontal_struct->id_});

        /* (3) Inject data to lookup map */
        lookup_structures->lookup_map_id_horizontal_structures_.emplace(
            tmp_horizontal_struct->id_, tmp_horizontal_struct);
    }

    logger_->info("Topology read.");
}

void MapDBReader::read_zones_from_map(
    dbs_map::model::ConsolidatedLayers::Ptr consolidated_layers, LookupStructures::MutablePtr lookup_structures)
{
    /* (0) Extract zones structures from map*/
    for (const auto& zone_ptr : consolidated_layers->zones_)
    {
        auto tmp_plane_struct = std::make_shared<PlaneStructure>();
        tmp_plane_struct->geometry_ = zone_ptr->geometry_;
        tmp_plane_struct->id_ = std::hash<std::string>{}(zone_ptr->id_);
        tmp_plane_struct->type_ = PlaneStructure::Type::ZONE;
        switch (zone_ptr->type_)
        {
            case dbs_map::model::Zone::Type::SafeZone:
                tmp_plane_struct->sub_type_ = PlaneStructure::SubType::SAFE_ZONE;
                break;
            case dbs_map::model::Zone::Type::RiskZone:
                tmp_plane_struct->sub_type_ = PlaneStructure::SubType::RISK_ZONE;
                break;
            case dbs_map::model::Zone::Type::LevelCrossingZone:
                tmp_plane_struct->sub_type_ = PlaneStructure::SubType::LEVEL_CROSSING_ZONE;
                break;
            default:
                tmp_plane_struct->sub_type_ = PlaneStructure::SubType::SUBTYPE_UNDEFINED;
                break;
        }

        /* (1) Project data to metric CS */
        tmp_plane_struct->geometry_utm_ = projection_->transform_polygon_wgs84_to_utm(tmp_plane_struct->geometry_);

        /* (2) Inject data to rtree */
        Box tmp_box = boost::geometry::return_envelope<Box>(tmp_plane_struct->geometry_utm_);
        lookup_structures->rtree_plane_structures_->insert({tmp_box, tmp_plane_struct->id_});

        /* (3) Inject data to lookup map */
        lookup_structures->lookup_map_id_plane_structures_.emplace(tmp_plane_struct->id_, tmp_plane_struct);
    }
    logger_->info("Zones read.");
}
