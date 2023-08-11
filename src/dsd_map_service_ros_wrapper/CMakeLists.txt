# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: CC0-1.0

# ######################################################################################################################
# Preamble
# ######################################################################################################################
cmake_minimum_required(VERSION 3.25)

project(
    dsd_map_service_ros_wrapper
    LANGUAGES CXX
    VERSION 1.0
)

# ######################################################################################################################
# Project Wide Setup
# ######################################################################################################################
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../cmake")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
include(Dependencies)
# ######################################################################################################################
# Dependencies
# ######################################################################################################################
# Ros defines by default an uninstall target that clashes with custom targets defined by map service dependencies. With
# this option we can turn this off.
set(AMENT_CMAKE_UNINSTALL_TARGET OFF)
find_package(ament_cmake REQUIRED)

add_submodule(
    NAME "internal/map-service"
    BINARY_BASE_DIR ${PROJECT_BINARY_DIR}
    OPTIONS "OLP_SDK_ENABLE_TESTING OFF" "ENABLE_TESTS OFF" "LEVELDB_BUILD_TESTS OFF" "LEVELDB_BUILD_BENCHMARKS OFF"
)
# ######################################################################################################################
# Main targets
# ######################################################################################################################
add_library(dsd_map_service_ros_wrapper INTERFACE)
add_library(dsd_map_service_ros_wrapper::dsd_map_service_ros_wrapper ALIAS dsd_map_service_ros_wrapper)
target_link_libraries(dsd_map_service_ros_wrapper INTERFACE dbs-map-api::dbs-map-api)

install(
    TARGETS dsd_map_service_ros_wrapper
    EXPORT dsd_map_service_ros_wrapperTargets
    INCLUDES
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
ament_export_dependencies(dbs-map-api)
ament_export_targets(dsd_map_service_ros_wrapperTargets HAS_LIBRARY_TARGET)
# ######################################################################################################################
# Tests
# ######################################################################################################################

# ######################################################################################################################
# Package
# ######################################################################################################################
ament_package()
