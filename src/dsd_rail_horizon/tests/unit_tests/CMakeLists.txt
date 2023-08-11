# SPDX-FileCopyrightText: Copyright DB Netz AG
# SPDX-License-Identifier: CC0-1.0

add_executable(dsd_rail_horizon_unit_tests)
target_sources(
    dsd_rail_horizon_unit_tests
    PRIVATE tests/publisher/AppStatusPublisherTests.cpp
            tests/publisher/RailHorizonPublisherTests.cpp
            tests/publisher/MarkerStructurePublisherTests.cpp
            tests/subscriber/CoupledLocalizationSubscriberTests.cpp
            tests/subscriber/MissionProfileSubscriberTests.cpp
            tests/utils/CalculationHelperTests.cpp
            tests/utils/MessageHelperTests.cpp
            tests/interfaces/implementations/RosClockTests.cpp
            tests/interfaces/implementations/RosPublisherTests.cpp
            tests/interfaces/implementations/RosLoggerTests.cpp
            tests/RailHorizonNodeTests.cpp
)
target_include_directories(dsd_rail_horizon_unit_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(dsd_rail_horizon_unit_tests PRIVATE GTest::gmock_main dsd_rail_horizon::dsd_rail_horizon_lib)

include(GoogleTest)
gtest_discover_tests(dsd_rail_horizon_unit_tests PROPERTIES LABELS "UnitTests")
