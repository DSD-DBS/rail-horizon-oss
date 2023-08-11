/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_CORE_MAP_DB_READER_H
#define DSD_RAIL_HORIZON_CORE_MAP_DB_READER_H

#include "dsd_rail_horizon_core/Projection.h"

#include <dsd_rail_horizon_core/interfaces/ILogger.h>
#include <dsd_rail_horizon_core/LookupStructures.h>
#include <dsd_rail_horizon_core/MapModel.h>

#include <dbs-map-api/download/ClientSettings.h>
#include <dbs-map-api/MapService.h>
#include <dbs-map-api/MapServiceConfig.h>
#include <dbs-map-api/model/ConsolidatedLayers.h>
#include <dbs-map-api/model/Landmarks.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <istream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

/**
 * @brief Configuration object for \ref MapDBReader class
 */
struct MapDBReaderConfig
{
    /**
     * @brief Directory path to local map data. Should have been created via cmake when map-service was built.
     */
    std::string db_map_persistent_storage_root_dir;

    /**
     * @brief Map catalog that should be read in
     */
    std::optional<std::string> db_map_catalog;

    /**
     * @brief Éndpoint which is used for accessing map data
     */
    std::optional<std::string> db_map_endpoint;

    /**
     * @brief Version which should be requested from map service during the map over the air update
     */
    std::optional<std::uint64_t> db_requested_map_version;

    /**
     * @brief Equality operator for comparing two configs
     */
    bool operator==(const MapDBReaderConfig& rhs) const
    {
        return db_map_persistent_storage_root_dir == rhs.db_map_persistent_storage_root_dir &&
               db_map_catalog == rhs.db_map_catalog && db_map_endpoint == rhs.db_map_endpoint &&
               db_requested_map_version == rhs.db_requested_map_version;
    }
};

/**
 * @brief Allows access to map data, which is taken from map-service and converted into structured suitable for
 * RailHorizon
 */
class MapDBReader
{
public:
    /**
     * @brief Constructor for map reader
     * @param config Config for the application status pubisher (see \ref MapDBReaderConfig)
     * @param projection Mutable pointer of Projection used for the projection of the data between WGS84 and UTM
     * coordinate systems
     * @param logger Logger instance for logging messages
     */
    MapDBReader(const MapDBReaderConfig& config, Projection::SharedPtr projection, ILogger::SharedPtr logger);

    /**
     * @brief Updates local map version over the air with version
     * @return Boolean indicating whether the over the air update was sucessfull
     */
    bool update_map_over_the_air();

    /**
     * @brief Request data from the map-service

     * The data is transformed from WGS84 to UTM coordinate system and both the original map data
     * and the transformed data are stored into lookup structures.
     *
     * @param map_roi vector of doubles containing the north east and the south west coordinate of a rectangular region
     in hh
     * @return Transformed data from map-service
     */
    LookupStructures::Ptr read(const std::vector<double>& map_roi);

    /**
     * @brief Returns the current local map version
     * @return map version as uint64_t
     */
    uint64_t get_local_map_version();

    /**
     * @brief Type alias for shared ptr on a mutable object of this class
     */
    using MutablePtr = std::shared_ptr<MapDBReader>;

    /**
     * @brief Type alias for shared ptr on a const object of this class
     */
    using Ptr = std::shared_ptr<const MapDBReader>;

private:
    /**
     * @brief Add vertical structure to the lookup
     * @param landmark const pointer to Landmark in which map data is stored
     * @param lookup_structures mutable pointer to LookupStructures where all map data in the WGS84 and UTM coordinates
     * is stored
     * @param type type of the vertical structure
     */
    void add_vertical_structure(dbs_map::model::Landmark::Ptr landmark, LookupStructures::MutablePtr lookup_structures,
        VerticalStructure::Type type);

    /**
     * @brief Add plane structure to the lookup
     * @param landmark const pointer to Landmark in which map data is stored
     * @param lookup_structures mutable pointer to LookupStructures where all map data in the WGS84 and UTM coordinates
     * is stored
     * @param type type of the plane structure
     */
    void add_plane_structure(dbs_map::model::Landmark::Ptr landmark, LookupStructures::MutablePtr lookup_structures,
        PlaneStructure::Type type);

    /**
     * @brief Add body structure to the lookup
     * @param landmark const pointer to Landmark in which map data is stored
     * @param lookup_structures mutable pointer to LookupStructures where all map data in the WGS84 and UTM coordinates
     * is stored
     * @param type type of the body structure
     */
    void add_body_structure(dbs_map::model::Landmark::Ptr landmark, LookupStructures::MutablePtr lookup_structures,
        BodyStructure::Type type);

    /**
     * @brief Read vertical structures from a map service and transform this data
     * from WGS84 to UTM coordinate system. Store the original map data
     * and the transformed data into lookup structures.
     * @param consolidated_layers const pointer to CosolidatedLayers in which map data is stored
     * @param lookup_structures mutable pointer to LookupStructures where all map data in the WGS84 and UTM coordinates
     * is stored
     */
    void read_landmarks_from_map(
        dbs_map::model::ConsolidatedLayers::Ptr consolidated_layers, LookupStructures::MutablePtr lookup_structures);

    /**
     * @brief Read horizontal structures from a map service and transform this data
     * from WGS84 to UTM coordinate system. Store the original map data
     * and the transformed data into lookup structures.
     * @param consolidated_layers const pointer to CosolidatedLayers in which map data is stored
     * @param lookup_structures mutable pointer to LookupStructures where all map data in the WGS84 and UTM coordinates
     * is stored
     */
    void read_topology_from_map(
        dbs_map::model::ConsolidatedLayers::Ptr consolidated_layers, LookupStructures::MutablePtr lookup_structures);

    /**
     * @brief Read zones from a map service and transform this data
     * from WGS84 to UTM coordinate system. Store the original map data
     * and the transformed data into lookup structures.
     * @param consolidated_layers const pointer to CosolidatedLayers in which map data is stored
     * @param lookup_structures mutable pointer to LookupStructures where all map data in the WGS84 and UTM coordinates
     * is stored
     */
    void read_zones_from_map(
        dbs_map::model::ConsolidatedLayers::Ptr consolidated_layers, LookupStructures::MutablePtr lookup_structures);

    /**
     * @brief Mutable pointer of Projection used for the projection of the data between WGS84 and UTM coordinate systems
     */
    Projection::SharedPtr projection_;
    /**
     * @brief Logger instance
     */
    ILogger::SharedPtr logger_;
    /**
     * @brief Mutable pointer of MapService
     */
    std::shared_ptr<dbs_map::MapService> map_service_;
    /**
     * @brief Instance of MapDBReaderConfig
     */
    MapDBReaderConfig config_;
};

/**
 * @brief Create MapDBReaderConfig with the values of the environment variables
 * @param logger Logger for console messages
 * @return MapDBReaderConfig Config for the map reader
 */
inline MapDBReaderConfig read_db_map_reader_config_from_env(ILogger::SharedPtr logger)
{
    MapDBReaderConfig config{};

    if (const char* map_storage_dir = std::getenv("PERSISTENT_STORAGE_ROOT_DIR"))
    {
        config.db_map_persistent_storage_root_dir = map_storage_dir;
    }
    else
    {
        logger->error("PERSISTENT_STORAGE_ROOT_DIR not set.");
    }

    if (const char* map_catalog = std::getenv("DB_MAP_CATALOG"))
    {
        config.db_map_catalog = map_catalog;
    }
    else
    {
        logger->info("DB_MAP_CATALOG not set. Using default map catalog.");
    }

    if (const char* map_endpoint = std::getenv("DB_MAP_ENDPOINT"))
    {
        config.db_map_endpoint = map_endpoint;
    }
    else
    {
        logger->info("DB_MAP_ENDPOINT not set up. Map endpoint set by default.");
    }

    if (const char* map_env = std::getenv("DB_MAP_VERSION"))
    {
        std::istringstream iss{map_env};
        std::uint64_t map_version = 0;
        iss >> map_version;
        config.db_requested_map_version = map_version;
    }
    else
    {
        logger->info("DB_MAP_VERSION not set up. Local map is used, no over the air update is possible.");
    }

    return config;
}

/**
 * @brief Create map service from given config
 * @param config Config of the map db reader
 * @param logger Logger for console messages
 * @return std::shared_ptr<map_service::MapService> Constructed map service
 */
inline std::shared_ptr<dbs_map::MapService> create_map_service(
    const MapDBReaderConfig& config, ILogger::SharedPtr logger)
{
    auto map_service_config = dbs_map::GetDefaultConfig();

    map_service_config.map_local_path_ =
        (std::filesystem::path{config.db_map_persistent_storage_root_dir} / "hdmap").string();

    if (config.db_map_catalog)
    {
        map_service_config.catalog_ = config.db_map_catalog.value();
    }

    if (config.db_map_endpoint)
    {
        if (*config.db_map_endpoint == "DBMC")
        {
            logger->info("DB_MAP_ENDPOINT = DBMC");
            map_service_config.http_client_settings_ = dbs_map::download::GetDBClientSettings();
        }
        else if (*config.db_map_endpoint == "DB_LAN")
        {
            logger->info("DB_MAP_ENDPOINT = DB_LAN");
            map_service_config.http_client_settings_ = dbs_map::download::GetDBClientForCISettings();
        }
        else
        {
            logger->error("DB_MAP_ENDPOINT is invalid. Map endpoint set by default.");
        }
        map_service_config.http_client_settings_.verbose_ = true;
    }
    return std::make_shared<dbs_map::MapService>(map_service_config);
}

#endif // DSD_RAIL_HORIZON_CORE_MAP_DB_READER_H