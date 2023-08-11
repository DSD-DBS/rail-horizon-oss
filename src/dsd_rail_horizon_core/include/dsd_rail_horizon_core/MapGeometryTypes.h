/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_CORE_MAP_GEOMETRY_TYPES_H
#define DSD_RAIL_HORIZON_CORE_MAP_GEOMETRY_TYPES_H

#include <dsd_rail_horizon_core/MapModel.h>

#include <dsd_common_types/GeometryTypes.h>

#include <boost/bimap/bimap.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>

/**
 * @brief Type alias for lookup map (id and horizontal structure)
 */
using LookupMapIdHorizontalStructure = std::map<Id, HorizontalStructure::Ptr>;
/**
 * @brief Type alias for lookup map (id and vertical structure)
 */
using LookupMapIdVerticalStructure = std::map<Id, VerticalStructure::Ptr>;
/**
 * @brief Type alias for lookup map (id and plane structure)
 */
using LookupMapIdPlaneStructure = std::map<Id, PlaneStructure::Ptr>;
/**
 * @brief Type alias for lookup map (id and body structure)
 */
using LookupMapIdBodyStructure = std::map<Id, BodyStructure::Ptr>;
/**
 * @brief Type alias for rtree index
 */
using StructuresIndex = boost::geometry::index::rtree<BoxIdPair, boost::geometry::index::quadratic<16>>;
/**
 * @brief Type alias for mutable pointer to rtree index
 */
using StructuresIndexPtr = std::shared_ptr<StructuresIndex>;

#endif // DSD_RAIL_HORIZON_CORE_MAP_GEOMETRY_TYPES_H
