# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: CC0-1.0

add_executable(dsd_rail_horizon_core_unit_tests MapDBReaderTest.cpp ProjectionTests.cpp)
target_link_libraries(
    dsd_rail_horizon_core_unit_tests PRIVATE GTest::gmock_main dsd_rail_horizon_core::dsd_rail_horizon_core
)
add_compile_definitions(
    dsd_rail_horizon_core_unit_tests PRIVATE
    MAP_PATH="${PROJECT_SOURCE_DIR}/../../submodules/internal/map-service/dbs-map-api/hdmap"
)

include(GoogleTest)
gtest_discover_tests(dsd_rail_horizon_core_unit_tests PROPERTIES LABELS "UnitTests")
