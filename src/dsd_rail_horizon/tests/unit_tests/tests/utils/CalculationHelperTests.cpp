/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "utils/CustomMatcher.h"
#include "utils/CustomPrintDefinitions.h" // IWYU pragma: keep // Enables more helpful output for custom types

#include <dsd_rail_horizon/utils/CalculationHelper.h>

#include <dsd_common_types/GeometryTypes.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using testing::Eq;

class CalculationHelperFixture : public ::testing::Test
{
};

// Test transformation of points
TEST_F(CalculationHelperFixture, transform)
{
    // Arrange
    auto rotation_matrix = calculate_rotation_matrix(PointXyz{0.2, 0.5, 1.0});
    GeometryTransformation transformation{PointXyz{2, 3, 4}, rotation_matrix};
    std::vector<PointXyz> test_points = {PointXyz{1, 2, 3}, PointXyz{4, 5, 6}, PointXyz{7, 8, 9}};

    // Act
    auto transformed_points = transformation.transform(test_points);

    std::vector<PointXyz> expected_points = {PointXyz{-0.12197, -1.63618, -0.555013},
        PointXyz{0.243941, 3.27236, 1.11003}, PointXyz{0.609852, 8.1809, 2.77506}};
    ASSERT_THAT(transformed_points, testing::Pointwise(PointNear(), expected_points));
}
