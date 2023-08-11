# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: CC0-1.0

# ######################################################################################################################
# Preamble
# ######################################################################################################################
cmake_minimum_required(VERSION 3.25)

project(
    dsd_rail_horizon_core
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
include(DeveloperSetup)
include(Dependencies)
# ######################################################################################################################
# Dependencies
# ######################################################################################################################
find_package(ament_cmake REQUIRED)
find_package(dsd_common_types REQUIRED)
find_package(dsd_map_service_ros_wrapper)
find_package(
    Boost 1.74
    COMPONENTS filesystem
    REQUIRED
) # Note: Changes here should also be done in cmake/configs/dsd_rail_horizon_coreConfigExtra.cmake
find_package(Proj REQUIRED) # Uses cmake/FindProj.cmake

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
    add_googletest_submodule()
endif()

# ######################################################################################################################
# Main targets
# ######################################################################################################################
# Note: It is common to have target definition and installation in the src folder, but ros requires ament_export_targets
# to be called in the same directory as ament_package

# Targets Targets
add_library(dsd_rail_horizon_core)
add_library(dsd_rail_horizon_core::dsd_rail_horizon_core ALIAS dsd_rail_horizon_core)
target_sources(dsd_rail_horizon_core PRIVATE "src/MapDBReader.cpp" "src/Projection.cpp")
target_include_directories(dsd_rail_horizon_core PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_link_libraries(
    dsd_rail_horizon_core PUBLIC dsd_common_types::dsd_common_types
                                 dsd_map_service_ros_wrapper::dsd_map_service_ros_wrapper Boost::filesystem PROJ::proj
)
ipm_target_dev_setup(dsd_rail_horizon_core)

# Installation
install(DIRECTORY "${PROJECT_SOURCE_DIR}/include/" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(
    TARGETS dsd_rail_horizon_core
    EXPORT dsd_rail_horizon_coreTargets
    INCLUDES
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
ament_export_dependencies(dsd_common_types)
ament_export_dependencies(dsd_map_service_ros_wrapper)
ament_export_targets(dsd_rail_horizon_coreTargets HAS_LIBRARY_TARGET)
# ######################################################################################################################
# Tests
# ######################################################################################################################
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
    add_subdirectory(tests)
endif()
# ######################################################################################################################
# Documentation
# ######################################################################################################################
add_subdirectory(docs)

# ######################################################################################################################
# Package
# ######################################################################################################################
ament_package(CONFIG_EXTRAS "cmake/configs/dsd_rail_horizon_coreConfigExtra.cmake")
