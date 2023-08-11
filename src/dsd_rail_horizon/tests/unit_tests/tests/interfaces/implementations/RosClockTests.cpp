/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "utils/CustomMatcher.h"
#include "utils/CustomPrintDefinitions.h"

#include <dsd_rail_horizon/interfaces/implementations/RosClock.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <boost/format.hpp>

#include <rclcpp/clock.hpp>

using testing::Eq;

class RosClockFixture : public ::testing::Test
{
};

TEST_F(RosClockFixture, NowReturnsCurrentTime)
{
    // Arrange
    auto clock = std::make_shared<rclcpp::Clock>();
    RosClock ros_clock(clock);

    // Act
    auto time = ros_clock.now();

    // Assert
    auto expected_time = clock->now();
    ASSERT_THAT(time, TimeNear(expected_time));
}