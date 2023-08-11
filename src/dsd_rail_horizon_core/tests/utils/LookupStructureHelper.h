/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LOOKUP_STRUCTURE_HELPER_H
#define LOOKUP_STRUCTURE_HELPER_H

#include "dsd_rail_horizon_core/MapModel.h"

#include <dsd_rail_horizon_core/LookupStructures.h>

#include <dbs-map-api/model/ConsolidatedLayers.h>
#include <dbs-map-api/model/Landmarks.h>

#include <iostream>

struct StructureCounter
{
    struct VerticalStructureCounter
    {
        int num_light = 0;
        int num_sign = 0;
        int num_signal = 0;
        int num_catenary = 0;
        int num_pillar = 0;
        int num_camera = 0;
        int num_pole_information = 0;
        int num_pole_shelter = 0;
        int num_pole_board_information = 0;
        int num_bar_barrier = 0;
        int num_type_unknown = 0;

        bool operator==(const VerticalStructureCounter& rhs) const
        {
            return num_light == rhs.num_light && num_sign == rhs.num_sign && num_signal == rhs.num_signal &&
                   num_catenary == rhs.num_catenary && num_pillar == rhs.num_pillar && num_camera == rhs.num_camera &&
                   num_pole_information == rhs.num_pole_information && num_pole_shelter == rhs.num_pole_shelter &&
                   num_pole_board_information == rhs.num_pole_board_information &&
                   num_bar_barrier == rhs.num_bar_barrier && num_type_unknown == rhs.num_type_unknown;
        }
    } vertical_structures;

    struct PlaneStructureCounter
    {
        int num_wall = 0;
        int num_billboard = 0;
        int num_tunnel = 0;
        int num_zone = 0;
        int num_risk_zone = 0;
        int num_safe_zone = 0;
        int num_level_crossing_zone = 0;
        int num_subtype_unknown = 0;
        int num_type_unknown = 0;

        bool operator==(const PlaneStructureCounter& rhs) const
        {
            return num_wall == rhs.num_wall && num_billboard == rhs.num_billboard && num_tunnel == rhs.num_tunnel &&
                   num_zone == rhs.num_zone && num_risk_zone == rhs.num_risk_zone &&
                   num_safe_zone == rhs.num_safe_zone && num_level_crossing_zone == rhs.num_level_crossing_zone &&
                   num_subtype_unknown == rhs.num_subtype_unknown && num_type_unknown == rhs.num_type_unknown;
        }
    } plane_structures;

    struct BodyStructureCounter
    {
        int num_fuse_box = 0;
        int num_switch_cabinet = 0;
        int num_building = 0;
        int num_platform = 0;
        int num_trash_can = 0;
        int num_container = 0;
        int num_foundation = 0;
        int num_barrier_body = 0;
        int num_type_unknown = 0;

        bool operator==(const BodyStructureCounter& rhs) const
        {
            return num_fuse_box == rhs.num_fuse_box && num_switch_cabinet == rhs.num_switch_cabinet &&
                   num_building == rhs.num_building && num_platform == rhs.num_platform &&
                   num_trash_can == rhs.num_trash_can && num_container == rhs.num_container &&
                   num_foundation == rhs.num_foundation && num_barrier_body == rhs.num_barrier_body &&
                   num_type_unknown == rhs.num_type_unknown;
        }
    } body_structures;

    bool operator==(const StructureCounter& rhs) const
    {
        return vertical_structures == rhs.vertical_structures && plane_structures == rhs.plane_structures &&
               body_structures == rhs.body_structures;
    }
};

inline StructureCounter count_structures(LookupStructures::Ptr lookup_structures)
{
    StructureCounter counter{};

    auto rtree_vertical_structures = lookup_structures->rtree_vertical_structures_;
    auto rtree_horizontal_structures = lookup_structures->rtree_horizontal_structures_;
    auto rtree_plane_structures = lookup_structures->rtree_plane_structures_;
    auto rtree_body_structures = lookup_structures->rtree_body_structures_;

    for (const auto& it : lookup_structures->lookup_map_id_vertical_structures_)
    {
        switch (it.second->type_)
        {
            case VerticalStructure::Type::LIGHT:
                counter.vertical_structures.num_light++;
                break;
            case VerticalStructure::Type::SIGN:
                counter.vertical_structures.num_sign++;
                break;
            case VerticalStructure::Type::SIGNAL:
                counter.vertical_structures.num_signal++;
                break;
            case VerticalStructure::Type::CATENARY:
                counter.vertical_structures.num_catenary++;
                break;
            case VerticalStructure::Type::PILLAR:
                counter.vertical_structures.num_pillar++;
                break;
            case VerticalStructure::Type::CAMERA:
                counter.vertical_structures.num_camera++;
                break;
            case VerticalStructure::Type::POLE_INFORMATION:
                counter.vertical_structures.num_pole_information++;
                break;
            case VerticalStructure::Type::POLE_SHELTER:
                counter.vertical_structures.num_pole_shelter++;
                break;
            case VerticalStructure::Type::POLE_BOARD_INFORMATION:
                counter.vertical_structures.num_pole_board_information++;
                break;
            case VerticalStructure::Type::BAR_BARRIER:
                counter.vertical_structures.num_bar_barrier++;
                break;
            default:
                counter.vertical_structures.num_type_unknown++;
                break;
        }
    }

    for (const auto& it : lookup_structures->lookup_map_id_plane_structures_)
    {
        switch (it.second->type_)
        {
            case PlaneStructure::Type::WALL:
                counter.plane_structures.num_wall++;
                break;
            case PlaneStructure::Type::BILLBOARD:
                counter.plane_structures.num_billboard++;
                break;
            case PlaneStructure::Type::TUNNEL:
                counter.plane_structures.num_tunnel++;
                break;
            case PlaneStructure::Type::ZONE:
                counter.plane_structures.num_zone++;
                switch (it.second->sub_type_)
                {
                    case PlaneStructure::SubType::RISK_ZONE:
                        counter.plane_structures.num_risk_zone++;
                        break;
                    case PlaneStructure::SubType::SAFE_ZONE:
                        counter.plane_structures.num_safe_zone++;
                        break;
                    case PlaneStructure::SubType::LEVEL_CROSSING_ZONE:
                        counter.plane_structures.num_level_crossing_zone++;
                        break;
                    default:
                        counter.plane_structures.num_subtype_unknown++;
                        break;
                }
                break;
            default:
                counter.plane_structures.num_type_unknown++;
                break;
        }
    }

    for (const auto& it : lookup_structures->lookup_map_id_body_structures_)
    {
        switch (it.second->type_)
        {
            case BodyStructure::Type::FUSE_BOX:
                counter.body_structures.num_fuse_box++;
                break;
            case BodyStructure::Type::SWITCH_CABINET:
                counter.body_structures.num_switch_cabinet++;
                break;
            case BodyStructure::Type::BUILDING:
                counter.body_structures.num_building++;
                break;
            case BodyStructure::Type::PLATFORM:
                counter.body_structures.num_platform++;
                break;
            case BodyStructure::Type::TRASH_CAN:
                counter.body_structures.num_trash_can++;
                break;
            case BodyStructure::Type::CONTAINER:
                counter.body_structures.num_container++;
                break;
            case BodyStructure::Type::FOUNDATION:
                counter.body_structures.num_foundation++;
                break;
            case BodyStructure::Type::BARRIER_BODY:
                counter.body_structures.num_barrier_body++;
                break;
            default:
                counter.body_structures.num_type_unknown++;
                break;
        }
    }
    return counter;
}

inline StructureCounter count_map_service_structures(dbs_map::model::ConsolidatedLayers::Ptr map_service_structures)
{
    StructureCounter counter{};

    for (const auto& landmarks_ptr : map_service_structures->landmarks_)
    {
        switch (landmarks_ptr->type_)
        {
            case dbs_map::model::Landmark::Type::CatenaryPole:
                counter.vertical_structures.num_catenary++;
                break;
            case dbs_map::model::Landmark::Type::Platform:
                counter.body_structures.num_platform++;
                break;
            case dbs_map::model::Landmark::Type::Wall:
                counter.plane_structures.num_wall++;
                break;
            case dbs_map::model::Landmark::Type::SignPole:
                counter.vertical_structures.num_sign++;
                break;
            case dbs_map::model::Landmark::Type::SignalPole:
                counter.vertical_structures.num_signal++;
                break;
            case dbs_map::model::Landmark::Type::Billboard:
                counter.plane_structures.num_billboard++;
                break;
            case dbs_map::model::Landmark::Type::BillboardPole:
                counter.vertical_structures.num_pole_board_information++;
                break;
            case dbs_map::model::Landmark::Type::BridgePillar:
                counter.vertical_structures.num_pillar++;
                break;
            case dbs_map::model::Landmark::Type::Building:
                counter.body_structures.num_building++;
                break;
            case dbs_map::model::Landmark::Type::CameraPole:
                counter.vertical_structures.num_camera++;
                break;
            case dbs_map::model::Landmark::Type::Container:
                counter.body_structures.num_container++;
                break;
            case dbs_map::model::Landmark::Type::CoveredPlatform:
                counter.vertical_structures.num_pillar++;
                break;
            case dbs_map::model::Landmark::Type::FuseBox:
                counter.body_structures.num_fuse_box++;
                break;
            case dbs_map::model::Landmark::Type::LightPole:
                counter.vertical_structures.num_light++;
                break;
            case dbs_map::model::Landmark::Type::OtherPole:
                counter.vertical_structures.num_type_unknown++;
                break;
            case dbs_map::model::Landmark::Type::Shelter:
                counter.vertical_structures.num_pole_shelter++;
                break;
            case dbs_map::model::Landmark::Type::BarrierBar:
                counter.vertical_structures.num_bar_barrier++;
                break;
            case dbs_map::model::Landmark::Type::BarrierBody:
                counter.body_structures.num_barrier_body++;
                break;
            case dbs_map::model::Landmark::Type::TunnelWall:
                counter.plane_structures.num_tunnel++;
                break;
            case dbs_map::model::Landmark::Type::TrashCan:
                counter.body_structures.num_trash_can++;
                break;
            default:
                break;
        }
    }

    for (const auto& zone_ptr : map_service_structures->zones_)
    {
        counter.plane_structures.num_zone++;
        switch (zone_ptr->type_)
        {
            case dbs_map::model::Zone::Type::SafeZone:
                counter.plane_structures.num_safe_zone++;
                break;
            case dbs_map::model::Zone::Type::RiskZone:
                counter.plane_structures.num_risk_zone++;
                break;
            case dbs_map::model::Zone::Type::LevelCrossingZone:
                counter.plane_structures.num_level_crossing_zone++;
                break;
            default:
                counter.plane_structures.num_subtype_unknown++;
                break;
        }
    }

    return counter;
}

void display(const StructureCounter& map_service_counter, const StructureCounter& map_db_counter)
{
    std::cout << "==============================================" << std::endl;
    std::cout << "OBJECT               Map-Service   Map DB Reader" << std::endl;
    std::cout << "=====================================================" << std::endl;
    std::cout << "LIGHT:               " << std::setw(8) << map_service_counter.vertical_structures.num_light
              << std::setw(16) << map_db_counter.vertical_structures.num_light << std::endl;
    std::cout << "SIGN:                " << std::setw(8) << map_service_counter.vertical_structures.num_sign
              << std::setw(16) << map_db_counter.vertical_structures.num_sign << std::endl;
    std::cout << "SIGNAL:              " << std::setw(8) << map_service_counter.vertical_structures.num_signal
              << std::setw(16) << map_db_counter.vertical_structures.num_signal << std::endl;
    std::cout << "CATENARY:            " << std::setw(8) << map_service_counter.vertical_structures.num_catenary
              << std::setw(16) << map_db_counter.vertical_structures.num_catenary << std::endl;
    std::cout << "PILLAR:              " << std::setw(8) << map_service_counter.vertical_structures.num_pillar
              << std::setw(16) << map_db_counter.vertical_structures.num_pillar << std::endl;
    std::cout << "CAMERA:              " << std::setw(8) << map_service_counter.vertical_structures.num_camera
              << std::setw(16) << map_db_counter.vertical_structures.num_camera << std::endl;
    std::cout << "SHELTER:             " << std::setw(8) << map_service_counter.vertical_structures.num_pole_shelter
              << std::setw(16) << map_db_counter.vertical_structures.num_pole_shelter << std::endl;
    std::cout << "BOARD_INFORMATION:   " << std::setw(8)
              << map_service_counter.vertical_structures.num_pole_board_information << std::setw(16)
              << map_db_counter.vertical_structures.num_pole_board_information << std::endl;
    std::cout << "BAR_BARRIER:         " << std::setw(8) << map_service_counter.vertical_structures.num_bar_barrier
              << std::setw(16) << map_db_counter.vertical_structures.num_bar_barrier << std::endl;
    std::cout << "UNKNOWN:             " << std::setw(8) << map_service_counter.vertical_structures.num_type_unknown
              << std::setw(16) << map_db_counter.vertical_structures.num_type_unknown << std::endl;

    std::cout << "WALL:                " << std::setw(8) << map_service_counter.plane_structures.num_wall
              << std::setw(16) << map_db_counter.plane_structures.num_wall << std::endl;
    std::cout << "BILLBOARD:           " << std::setw(8) << map_service_counter.plane_structures.num_billboard
              << std::setw(16) << map_db_counter.plane_structures.num_billboard << std::endl;
    std::cout << "TUNNEL:              " << std::setw(8) << map_service_counter.plane_structures.num_tunnel
              << std::setw(16) << map_db_counter.plane_structures.num_tunnel << std::endl;
    std::cout << "ZONE:                " << std::setw(8) << map_service_counter.plane_structures.num_zone
              << std::setw(16) << map_db_counter.plane_structures.num_zone << std::endl;
    std::cout << "RISK_ZONE:           " << std::setw(8) << map_service_counter.plane_structures.num_risk_zone
              << std::setw(16) << map_db_counter.plane_structures.num_risk_zone << std::endl;
    std::cout << "SAFE_ZONE:           " << std::setw(8) << map_service_counter.plane_structures.num_safe_zone
              << std::setw(16) << map_db_counter.plane_structures.num_safe_zone << std::endl;
    std::cout << "LEVEL_CROSSING_ZONE: " << std::setw(8) << map_service_counter.plane_structures.num_level_crossing_zone
              << std::setw(16) << map_db_counter.plane_structures.num_level_crossing_zone << std::endl;
    std::cout << "SUBTYPE_UNKNOWN:     " << std::setw(8) << map_service_counter.plane_structures.num_subtype_unknown
              << std::setw(16) << map_db_counter.plane_structures.num_subtype_unknown << std::endl;

    std::cout << "FUSE_BOX:            " << std::setw(8) << map_service_counter.body_structures.num_fuse_box
              << std::setw(16) << map_db_counter.body_structures.num_fuse_box << std::endl;
    std::cout << "BUILDING:            " << std::setw(8) << map_service_counter.body_structures.num_building
              << std::setw(16) << map_db_counter.body_structures.num_building << std::endl;
    std::cout << "PLATFORM:            " << std::setw(8) << map_service_counter.body_structures.num_platform
              << std::setw(16) << map_db_counter.body_structures.num_platform << std::endl;
    std::cout << "TRASH_CAN:           " << std::setw(8) << map_service_counter.body_structures.num_trash_can
              << std::setw(16) << map_db_counter.body_structures.num_trash_can << std::endl;
    std::cout << "CONTAINER:           " << std::setw(8) << map_service_counter.body_structures.num_container
              << std::setw(16) << map_db_counter.body_structures.num_container << std::endl;
    std::cout << "BARRIER_BODY:        " << std::setw(8) << map_service_counter.body_structures.num_barrier_body
              << std::setw(16) << map_db_counter.body_structures.num_barrier_body << std::endl;
    std::cout << "=====================================================" << std::endl;
}

#endif // LOOKUP_STRUCTURE_HELPER_H