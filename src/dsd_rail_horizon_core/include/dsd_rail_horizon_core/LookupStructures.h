/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_CORE_LOOKUP_STRUCTURES_H
#define DSD_RAIL_HORIZON_CORE_LOOKUP_STRUCTURES_H

#include <dsd_rail_horizon_core/MapGeometryTypes.h>
#include <dsd_rail_horizon_core/MapModel.h>

#include <memory>

/**
 * @brief Structure to store lookup structures of all types. This is used for convenience to return all structures in a
 * local area.
 */
struct StructureContainer
{
    std::vector<VerticalStructure::Ptr> vertical;
    std::vector<HorizontalStructure::Ptr> horizontal;
    std::vector<PlaneStructure::Ptr> plane;
    std::vector<BodyStructure::Ptr> body;
};


/**
 * @brief Storage class for map structures with rtree index pointers and lookup maps for all map structure types.
 * Additionally provides functionality for accessing the data.
 */
class LookupStructures
{
public:
    /**
     * @brief Returns structures within the radius of the current gnss position
     *
     * @param current_gnss_position The current gnss position of the train
     * @param radius Radius in which structures should be returned
     */
    [[nodiscard]] StructureContainer get_structures_in_map_foresight(
        const PointXyz& current_gnss_position, double radius) const
    {
        StructureContainer structures{};
        structures.vertical = get_structure_in_map_foresight<VerticalStructure>(
            rtree_vertical_structures_, lookup_map_id_vertical_structures_, current_gnss_position, radius);
        structures.horizontal = get_structure_in_map_foresight<HorizontalStructure>(
            rtree_horizontal_structures_, lookup_map_id_horizontal_structures_, current_gnss_position, radius);
        structures.plane = get_structure_in_map_foresight<PlaneStructure>(
            rtree_plane_structures_, lookup_map_id_plane_structures_, current_gnss_position, radius);
        structures.body = get_structure_in_map_foresight<BodyStructure>(
            rtree_body_structures_, lookup_map_id_body_structures_, current_gnss_position, radius);
        return structures;
    }

    /**
     * @brief Type alias for shared ptr on a const object of this class
     */
    using Ptr = std::shared_ptr<const LookupStructures>;
    /**
     * @brief Type alias for shared ptr on a mutable object of this class
     */
    using MutablePtr = std::shared_ptr<LookupStructures>;

    /**
     * @brief Queries \p{structure} to get structures in the \p{radius} around \p{current_gnss_position}
     *
     * @param structure The lookup structure index, which should be queried for objects
     * @param lookup_map Map for accessing the found lookup structures
     * @param current_gnss_position The current gnss position of the train
     * @param radius Radius in which structures should be returned
     */
    template <typename Structure>
    [[nodiscard]] std::vector<typename Structure::Ptr> get_structure_in_map_foresight(StructuresIndexPtr structure,
        std::map<Id, typename Structure::Ptr> lookup_map, const PointXyz& current_gnss_position, double radius) const
    {
        auto box_id_pairs = get_map_foresight(structure, current_gnss_position, radius);
        std::vector<typename Structure::Ptr> structures{};
        std::transform(begin(box_id_pairs), end(box_id_pairs), std::back_inserter(structures), [&](const auto& pair) {
            return lookup_map.at(pair.second);
        });
        return structures;
    }

    /**
     * @brief Queries \p{structure} to get structure ids in the \p{radius} around \p{current_gnss_position}
     * @param structure pointer to the StructureIndex
     * @param current_gnss_position position of the train given as a point in 3D space
     * @param radius radius around the train unit
     * @return result - vector of BoxIdPairs within the given radius
     */
    [[nodiscard]] std::vector<BoxIdPair> get_map_foresight(
        StructuresIndexPtr structure, const PointXyz& current_gnss_position, double radius) const
    {
        std::vector<BoxIdPair> result;
        structure->query(boost::geometry::index::satisfies([&](BoxIdPair const& v) {
            return boost::geometry::distance(v.first, current_gnss_position) < radius;
        }),
            std::back_inserter(result));
        return result;
    }

    // Vertical Structures
    StructuresIndexPtr rtree_vertical_structures_ = std::make_shared<StructuresIndex>();
    /**
     * @brief Lookup map as a sorted associative container containig vertical structures and their respective
     * identifiers
     */
    LookupMapIdVerticalStructure lookup_map_id_vertical_structures_;
    /**
     * @brief Shared prointer on a mutable object of type R-tree spatial index used with horizontal structures
     */
    StructuresIndexPtr rtree_horizontal_structures_ = std::make_shared<StructuresIndex>();
    /**
     * @brief Lookup map as a sorted associative container containig horizontal structures and their respective
     * identifiers
     */
    LookupMapIdHorizontalStructure lookup_map_id_horizontal_structures_;
    // Plane Structures
    /**
     * @brief Shared prointer on a mutable object of type R-tree spatial index used with plane structures
     */
    StructuresIndexPtr rtree_plane_structures_ = std::make_shared<StructuresIndex>();
    /**
     * @brief Lookup map as a sorted associative container containig plain structures and their respective identifiers
     */
    LookupMapIdPlaneStructure lookup_map_id_plane_structures_;
    // Body Structures
    /**
     * @brief Shared prointer on a mutable object of type R-tree spatial index used with body structures
     */
    StructuresIndexPtr rtree_body_structures_ = std::make_shared<StructuresIndex>();
    /**
     * @brief Lookup map as a sorted associative container containig body structures and their respective identifiers
     */
    LookupMapIdBodyStructure lookup_map_id_body_structures_;
};
#endif // DSD_RAIL_HORIZON_CORE_LOOKUP_STRUCTURES_H
