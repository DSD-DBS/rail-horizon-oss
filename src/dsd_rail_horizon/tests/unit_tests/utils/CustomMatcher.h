/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <gmock/gmock.h>

/**
 * @brief GMock Matcher to compare if two rclpp::Time objects are close to each other
 */
MATCHER_P(TimeNear, expected_time, "")
{
    return std::abs(arg.seconds() - expected_time.seconds()) <= 0.0001;
}

/**
 * @brief GMock Matcher to compare if two point objects are close to each other
 */
MATCHER(PointNear, "")
{
    const auto actual_point = std::get<0>(arg);
    const auto expected_point = std::get<1>(arg);
    const double tolerance = 0.001;
    bool x_near = std::abs(actual_point.template get<0>() - expected_point.template get<0>()) <= tolerance;
    bool y_near = std::abs(actual_point.template get<1>() - expected_point.template get<1>()) <= tolerance;
    bool z_near = std::abs(actual_point.template get<2>() - expected_point.template get<2>()) <= tolerance;
    return x_near && y_near && z_near;
}
