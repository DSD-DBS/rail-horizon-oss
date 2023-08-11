/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_STRUCTURE_DATA_H
#define DSD_RAIL_HORIZON_STRUCTURE_DATA_H

#include <dsd_rail_horizon_core/LookupStructures.h>

#include <dsd_common_types/GeoGeometryTypes.h>
#include <dsd_common_types/GeometryTypes.h>

inline StructureContainer create_structures()
{
    LineStringXyzWgs84 ls_1 = {{1, 2, 3}, {4, 5, 6}};
    LineStringXyzWgs84 ls_1_2 = {{7, 8, 9}, {10, 11, 12}};
    LineStringXyzWgs84 ls_1_3 = {{13, 14, 15}, {16, 17, 18}};
    LineStringXyz ls_1_utm = {{6, 5, 4}, {3, 2, 1}};
    LineStringXyz ls_1_2_utm = {{12, 11, 10}, {9, 8, 7}};
    LineStringXyz ls_1_3_utm = {{15, 14, 13}, {12, 11, 10}};

    LineStringXyzWgs84 ls_2 = {{2, 4, 6}, {8, 10, 12}};
    LineStringXyzWgs84 ls_2_2 = {{14, 16, 18}, {20, 22, 24}};
    LineStringXyzWgs84 ls_2_3 = {{26, 28, 30}, {32, 34, 36}};
    LineStringXyz ls_2_utm = {{12, 10, 8}, {6, 4, 2}};
    LineStringXyz ls_2_2_utm = {{24, 22, 20}, {18, 16, 14}};
    LineStringXyz ls_2_3_utm = {{36, 34, 32}, {30, 28, 26}};

    PolygonXyzWgs84 p_1{{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}}};
    PolygonXyz p_1_utm{{{9, 8, 7}, {6, 5, 4}, {3, 2, 1}}};
    PolygonXyzWgs84 p_2{{{2, 4, 6}, {8, 10, 12}, {14, 16, 18}}};
    PolygonXyz p_2_utm{{{18, 16, 14}, {12, 10, 8}, {6, 4, 2}}};

    StructureContainer structures{};
    structures.vertical = {
        std::make_shared<VerticalStructure>(
            VerticalStructure{10, VerticalStructure::Type::LIGHT, ls_1, ls_1_utm, 1, 2}),
        std::make_shared<VerticalStructure>(VerticalStructure{20, VerticalStructure::Type::SIGN, ls_2, ls_2_utm, 3, 4}),
    };

    structures.horizontal = {
        std::make_shared<HorizontalStructure>(
            HorizontalStructure{30, 1, 10, ls_1, 100, ls_1_2, ls_1_3, ls_1_utm, ls_1_2_utm, ls_1_3_utm}),
        std::make_shared<HorizontalStructure>(
            HorizontalStructure{100, 2, 20, ls_2, 100, ls_2_2, ls_2_3, ls_2_utm, ls_2_2_utm, ls_2_3_utm}),
    };

    structures.body = {
        std::make_shared<BodyStructure>(BodyStructure{60, p_1, p_1_utm, 100, BodyStructure::Type::CONTAINER}),
    };

    structures.plane = {
        std::make_shared<PlaneStructure>(
            PlaneStructure{40, p_1, p_1_utm, PlaneStructure::Type::TUNNEL, PlaneStructure::SubType::SUBTYPE_UNDEFINED}),
        std::make_shared<PlaneStructure>(
            PlaneStructure{50, p_2, p_2_utm, PlaneStructure::Type::ZONE, PlaneStructure::SubType::RISK_ZONE}),
    };

    return structures;
}

#endif // DSD_RAIL_HORIZON_STRUCTURE_DATA_H