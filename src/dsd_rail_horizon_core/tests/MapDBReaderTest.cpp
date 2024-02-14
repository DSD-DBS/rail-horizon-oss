/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "gmock/gmock.h"
#include "utils/GTestPrettyPrint.h" // IWYU pragma: keep // Enables more helpful output for custom types
#include "utils/LoggerMock.h"
#include "utils/LookupStructureHelper.h"

#include <dsd_rail_horizon_core/LookupStructures.h>
#include <dsd_rail_horizon_core/MapDBReader.h>
#include <dsd_rail_horizon_core/MapGeometryTypes.h>
#include <dsd_rail_horizon_core/MapModel.h>
#include <dsd_rail_horizon_core/Projection.h>

#include <dbs-map-api/MapServiceConfig.h>

#include <gtest/gtest.h>

#include <boost/geometry/index/rtree.hpp>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using testing::Eq;
using testing::HasSubstr;
using testing::Matcher;
using testing::Ne;

class TemporaryEnvironmentVariable
{
public:
    TemporaryEnvironmentVariable(const char* env_name) : env_name_{env_name}, saved_env_value_(std::getenv(env_name)) {}

    void set(const char* env_value)
    {
        setenv(env_name_, env_value, 1);
    }

    void unset()
    {
        unsetenv(env_name_);
    }

    ~TemporaryEnvironmentVariable()
    {
        setenv(env_name_, saved_env_value_, 1);
    }

private:
    const char* env_name_;
    const char* saved_env_value_;
};

class MapDBReaderFixture : public ::testing::Test
{
public:
    MapDBReaderFixture()
    {
        temp_map_dir = testing::TempDir();
        temp_map_dir += "map_data";
        std::filesystem::remove_all(temp_map_dir);

        auto map_data_src = map_service_map_data_dir / catalog / std::to_string(version);
        auto map_data_dest = temp_map_dir / "hdmap" / catalog / std::to_string(version);
        std::filesystem::create_directories(map_data_dest);

        std::filesystem::copy(map_data_src, map_data_dest, std::filesystem::copy_options::recursive);
        std::filesystem::create_directory_symlink(map_data_dest, map_data_dest.parent_path() / "current");
    }

    ~MapDBReaderFixture() override
    {
        std::filesystem::remove_all(temp_map_dir);
    }

    MapDBReaderConfig get_default_config()
    {
        if (const char* map_endpoint = std::getenv("DB_MAP_ENDPOINT"))
        {
            return MapDBReaderConfig{temp_map_dir.string(), catalog, map_endpoint, version};
        }
        return MapDBReaderConfig{temp_map_dir.string(), catalog, "DBMC", version};
    }

    void set_envs_according_to_config(const MapDBReaderConfig& config)
    {
        persistent_storage_root_dir_env.set(config.db_map_persistent_storage_root_dir.c_str());
        db_map_catalog_env.set(config.db_map_catalog->c_str());
        db_map_endpoint_env.set(config.db_map_endpoint->c_str());
        db_map_version_env.set(std::to_string(*config.db_requested_map_version).c_str());
    }

    MapDBReader create_map_reader(const MapDBReaderConfig& config)
    {
        auto projection_config = create_wsg84_to_utm_config(logger_, false);
        std::shared_ptr<Projection> projection = std::make_shared<Projection>(projection_config, logger_);
        return MapDBReader{config, projection, logger_};
    }

    dbs_map::model::ConsolidatedLayers::Ptr get_map_service_structures()
    {
        auto south_west = dbs_map::GeoCoordinates{map_roi_[0], map_roi_[1]};
        auto north_east = dbs_map::GeoCoordinates{map_roi_[2], map_roi_[3]};
        return create_map_service(get_default_config(), logger_)->GetLayersForRectangle({south_west, north_east});
    }


protected:
    std::shared_ptr<LoggerMock> logger_ = std::make_shared<LoggerMock>();
    std::vector<double> map_roi_ = {9.42564, 53.36550, 10.38914, 53.69209};

    TemporaryEnvironmentVariable persistent_storage_root_dir_env{"PERSISTENT_STORAGE_ROOT_DIR"};
    TemporaryEnvironmentVariable db_map_catalog_env{"DB_MAP_CATALOG"};
    TemporaryEnvironmentVariable db_map_endpoint_env{"DB_MAP_ENDPOINT"};
    TemporaryEnvironmentVariable db_map_version_env{"DB_MAP_VERSION"};

    std::filesystem::path map_service_map_data_dir{MAP_PATH}; // MAP_PATH is defined by us in cmake as a macro
    std::string catalog = "validate.s4r2.oss.4";
    std::size_t version = 63;
    std::filesystem::path temp_map_dir;
};

// Check that all environment envs are correctly read
TEST_F(MapDBReaderFixture, readConfigIfAllEnvsSet)
{
    // Arrange
    MapDBReaderConfig actual_config = get_default_config();
    set_envs_according_to_config(actual_config);

    // Act
    MapDBReaderConfig config = read_db_map_reader_config_from_env(logger_);

    // Assert
    ASSERT_THAT(config, Eq(actual_config));
}

// Checks whether there is an error if the persistent storage root dir env is not set
TEST_F(MapDBReaderFixture, readConfigIfPersistentStorageRootDirEnvNotSet)
{
    // Arrange
    MapDBReaderConfig actual_config = get_default_config();
    set_envs_according_to_config(actual_config);
    actual_config.db_map_persistent_storage_root_dir = "";
    persistent_storage_root_dir_env.unset();

    // Expect
    EXPECT_CALL(*logger_, error(Matcher<const std::string&>(HasSubstr("PERSISTENT_STORAGE_ROOT_DIR"))));

    // Act
    MapDBReaderConfig config = read_db_map_reader_config_from_env(logger_);

    // Assert
    ASSERT_THAT(config, Eq(actual_config));
}

// Checks whether there is a warning if the db map catalog env is not set
TEST_F(MapDBReaderFixture, readConfigIfDbMapCatalogEnvNotSet)
{
    // Arrange
    MapDBReaderConfig actual_config = get_default_config();
    set_envs_according_to_config(actual_config);
    actual_config.db_map_catalog = std::nullopt;
    db_map_catalog_env.unset();

    // Expect
    EXPECT_CALL(*logger_, info(Matcher<const std::string&>(testing::HasSubstr("DB_MAP_CATALOG"))));

    // Act
    MapDBReaderConfig config = read_db_map_reader_config_from_env(logger_);

    // Assert
    ASSERT_THAT(config, Eq(actual_config));
}

// Checks whether there is a warning if the db map endpoint is not set
TEST_F(MapDBReaderFixture, readConfigIfDbMapEndpointEnvNotSet)
{
    // Arrange
    MapDBReaderConfig actual_config = get_default_config();
    set_envs_according_to_config(actual_config);
    actual_config.db_map_endpoint = std::nullopt;
    db_map_endpoint_env.unset();

    // Expect
    EXPECT_CALL(*logger_, info(Matcher<const std::string&>(testing::HasSubstr("DB_MAP_ENDPOINT"))));

    // Act
    MapDBReaderConfig config = read_db_map_reader_config_from_env(logger_);

    // Assert
    ASSERT_THAT(config, Eq(actual_config));
}

// Checks whether there is a warning if the db map version is not set
TEST_F(MapDBReaderFixture, readConfigIfDbMapVersionEnvNotSet)
{
    // Arrange
    MapDBReaderConfig actual_config = get_default_config();
    set_envs_according_to_config(actual_config);
    actual_config.db_requested_map_version = std::nullopt;
    db_map_version_env.unset();

    // Expect
    EXPECT_CALL(*logger_, info(Matcher<const std::string&>(testing::HasSubstr("DB_MAP_VERSION"))));

    // Act
    MapDBReaderConfig config = read_db_map_reader_config_from_env(logger_);

    // Assert
    ASSERT_THAT(config, Eq(actual_config));
}


// Checks that no data is read if no local map is available
TEST_F(MapDBReaderFixture, errorIfNoLocalMapIsAvailable)
{
    // Arrange
    auto config = get_default_config();
    config.db_map_persistent_storage_root_dir = "";
    auto map_reader = create_map_reader(config);

    // Act
    LookupStructures::Ptr lookup_structures = map_reader.read(map_roi_);

    // Assert
    ASSERT_THAT(lookup_structures, Eq(nullptr));
}

// Checks that map over the air updates failes if no version is given, but the local map is used then
TEST_F(MapDBReaderFixture, overTheAirUpdateWithoutVersionFails)
{
    // Arrange
    auto config = get_default_config();
    config.db_requested_map_version = std::nullopt;
    auto map_reader = create_map_reader(config);

    // Act
    bool ova_sucessfull = map_reader.update_map_over_the_air();

    // Assert
    ASSERT_THAT(ova_sucessfull, Eq(false));
}

// Checks that map over the air updates is sucessful
// This test will work only with special access rights
TEST_F(MapDBReaderFixture, overTheAirUpdateIsSucessfull)
{
    // Arrange
    auto config = get_default_config();
    auto map_reader = create_map_reader(config);

    // Act
    bool ova_sucessfull = map_reader.update_map_over_the_air();

    // Assert
    ASSERT_THAT(ova_sucessfull, Eq(true));
}

// Checks that map over the air fails when we have not connection
// TEST_F(MapDBReaderFixture, noConnectionToMapEndpoint)
// {
//     // Arrange
//     auto config = get_default_config();
//     config.db_requested_map_version = 99999;
//     auto map_reader = create_map_reader(config);

//     // Act
//     bool ova_sucessfull = map_reader.update_map_over_the_air();

//     // Assert
//     ASSERT_THAT(ova_sucessfull, Eq(false));
// }


// Check if map service structures are parsed correctly and equal to original data when using DBMC Endpoint
TEST_F(MapDBReaderFixture, parseReadResultDBMC)
{
    // Arrange
    auto config = get_default_config();
    config.db_map_endpoint = "DBMC";
    auto map_reader = create_map_reader(config);

    // Act
    LookupStructures::Ptr lookup_structures = map_reader.read(map_roi_);

    // Assert
    StructureCounter counter = count_structures(lookup_structures);
    StructureCounter map_service_counter = count_map_service_structures(get_map_service_structures());

    display(map_service_counter, counter);

    ASSERT_THAT(counter, Eq(map_service_counter));
}

// Check if map service structures are parsed correctly and equal to original data when using DB Lan Endpoint
TEST_F(MapDBReaderFixture, parseReadResultDBLAN)
{
    // Arrange
    auto config = get_default_config();
    config.db_map_endpoint = "DB_LAN";
    auto map_reader = create_map_reader(config);

    // Act
    LookupStructures::Ptr lookup_structures = map_reader.read(map_roi_);

    // Assert
    StructureCounter counter = count_structures(lookup_structures);
    StructureCounter map_service_counter = count_map_service_structures(get_map_service_structures());

    display(map_service_counter, counter);

    ASSERT_THAT(counter, Eq(map_service_counter));
}

// Check if map service structures are parsed correctly and equal to original data when using Invalid Enpoint. In this
// case additionally an error should be logged
TEST_F(MapDBReaderFixture, parseReadResultInvalidEndpoint)
{
    // Arrange
    auto config = get_default_config();
    config.db_map_endpoint = "Invalid";

    // Expect
    EXPECT_CALL(*logger_, error(Matcher<const std::string&>(HasSubstr("DB_MAP_ENDPOINT"))));

    // Act
    auto map_reader = create_map_reader(config);
    LookupStructures::Ptr lookup_structures = map_reader.read(map_roi_);

    // Assert
    StructureCounter counter = count_structures(lookup_structures);
    StructureCounter map_service_counter = count_map_service_structures(get_map_service_structures());

    display(map_service_counter, counter);

    ASSERT_THAT(counter, Eq(map_service_counter));
}
