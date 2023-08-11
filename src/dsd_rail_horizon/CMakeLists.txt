# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: CC0-1.0

# ######################################################################################################################
# Preamble
# ######################################################################################################################
cmake_minimum_required(VERSION 3.25)

project(
    dsd_rail_horizon
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

find_package(dsd_rail_horizon_core REQUIRED)
find_package(dsd_ros_messages REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Proj REQUIRED) # Uses cmake/FindProj.cmake

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
    find_package(launch_testing_ament_cmake)
    add_googletest_submodule()
endif()
# ######################################################################################################################
# Main targets
# ######################################################################################################################
# Note(leon): It is common to have target definition and installation in the src folder, but ros requires
# ament_export_targets to be called in the same directory as ament_package

# Targets
add_library(dsd_rail_horizon_lib)
add_library(dsd_rail_horizon::dsd_rail_horizon_lib ALIAS dsd_rail_horizon_lib)
target_sources(
    dsd_rail_horizon_lib
    PRIVATE src/RailHorizonNode.cpp
            src/RailHorizonParameters.cpp
            src/publisher/RailHorizonPublisher.cpp
            src/publisher/AppStatusPublisher.cpp
            src/publisher/MarkerStructurePublisher.cpp
            src/subscriber/MissionProfileSubscriber.cpp
            src/subscriber/CoupledLocalizationSubscriber.cpp
)
target_include_directories(dsd_rail_horizon_lib PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(
    dsd_rail_horizon_lib PUBLIC dsd_rail_horizon_core::dsd_rail_horizon_core dsd_ros_messages::dsd_ros_messages
                                PROJ::proj rclcpp::rclcpp ${std_msgs_TARGETS} ${visualization_msgs_TARGETS}
)
ipm_target_dev_setup(dsd_rail_horizon_lib)

add_executable(dsd_rail_horizon)
target_sources(dsd_rail_horizon PRIVATE src/RailHorizonMain.cpp)
target_link_libraries(dsd_rail_horizon PRIVATE dsd_rail_horizon::dsd_rail_horizon_lib)
ipm_target_compile_warnings(dsd_rail_horizon)

# Install
install(DIRECTORY "${PROJECT_SOURCE_DIR}/config/" DESTINATION config/${PROJECT_NAME})
install(DIRECTORY "${PROJECT_SOURCE_DIR}/launch/" DESTINATION share/${PROJECT_NAME})
install(TARGETS dsd_rail_horizon DESTINATION lib/${PROJECT_NAME})

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
ament_package()
