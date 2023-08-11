/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "gmock/gmock.h"
#include "utils/LoggerMock.h"

#include <dsd_rail_horizon_core/interfaces/ILogger.h>
#include <dsd_rail_horizon_core/Projection.h>

#include <dsd_common_types/GeoGeometryTypes.h>
#include <dsd_common_types/GeometryTypes.h>

#include <gtest/gtest.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include <ext/alloc_traits.h>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

using ::testing::Eq;
using ::testing::Ne;

namespace boost::geometry::model
{
void PrintTo(const PointXyz& point, std::ostream* os)
{
    *os << "(" << point.get<0>() << ", " << point.get<1>() << ", " << point.get<2>() << ")";
}
} // namespace boost::geometry::model

namespace
{
MATCHER_P(PointNear, expected_point, "")
{
    const double tolerance = 0.001;
    bool x_near = std::abs(arg.template get<0>() - expected_point.template get<0>()) <= tolerance;
    bool y_near = std::abs(arg.template get<1>() - expected_point.template get<1>()) <= tolerance;
    bool z_near = std::abs(arg.template get<2>() - expected_point.template get<2>()) <= tolerance;
    return x_near && y_near && z_near;
}
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
} // namespace

class ProjectionFixture : public ::testing::Test
{
public:
    Projection create_projection()
    {
        return Projection{create_wsg84_to_utm_config(logger_, false), logger_};
    }

    Projection create_geoid_projection()
    {
        return Projection{create_wsg84_to_utm_config(logger_, true), logger_};
    }

    Projection create_invalid_projection()
    {
        return Projection{ProjectionConfig{"invalid", "invalid"}, logger_};
    }


protected:
    std::shared_ptr<LoggerMock> logger_ = std::make_shared<LoggerMock>();

    // Create tests points (Eifeltower, Brandenburger Tor, Big Ben) as WGS84 (GPS) points (longitude, latitude)
    std::vector<PointXyzWgs84> wgs84_points{PointXyzWgs84{2.294481, 48.858370, 0.0},
        PointXyzWgs84{13.377702, 52.51628, 0.0}, PointXyzWgs84{-0.124584, 51.50069, 0.0}};
    // Corresponding converted points in WGS84-UTM Zone 32N (eastern, northern)
    std::vector<PointXyz> utm_points{PointXyz{8304.094, 5433426.188, 0.0}, PointXyz{796987.105, 5827473.637, 0.0},
        PointXyz{-132742.417, 5745085.758, 0.0}};
};

// Checks there is no error on construction during the creation of the projects with the normal config
TEST_F(ProjectionFixture, noErrorOnConstruction)
{
    // Act
    Projection projection = create_projection();

    // Assert
    EXPECT_THAT(projection.get_error_status(), Eq(""));
}

// Checks there is no error on construction during the creation of the projects with the geoid config
TEST_F(ProjectionFixture, noErrorOnConstructionGeoidConfig)
{
    // Act
    Projection projection = create_geoid_projection();

    // Assert
    EXPECT_THAT(projection.get_error_status(), Eq(""));
}

// Checks that point is sucessfully converted from wsgs84 to utm
TEST_F(ProjectionFixture, transformPointSucessfully)
{
    // Arrange
    Projection projection = create_projection();

    // Act
    PointXyz utm_point = projection.transform_point_wgs84_to_utm(wgs84_points[0]);

    // Assert
    EXPECT_THAT(utm_point, PointNear(utm_points[0]));
}

// Checks that original point is returned and error is set if projections cannot be set up
TEST_F(ProjectionFixture, transformPointWithInvalidProjection)
{
    // Arrange
    Projection projection = create_invalid_projection();

    // Act
    PointXyz utm_point = projection.transform_point_wgs84_to_utm(wgs84_points[0]);

    // Assert
    EXPECT_THAT(projection.get_error_status(), Ne(""));
    EXPECT_THAT(utm_point, PointNear(wgs84_points[0]));
}

// Checks that linestring is sucessfully converted from wsgs84 to utm
TEST_F(ProjectionFixture, transformLineSucessfully)
{
    // Arrange
    Projection projection = create_projection();
    LineStringXyzWgs84 wsg84_line_string{wgs84_points.begin(), wgs84_points.end()};

    // Act
    LineStringXyz utm_line_string = projection.transform_linestring_wgs84_to_utm(wsg84_line_string);

    // Assert
    EXPECT_THAT(utm_line_string, testing::Pointwise(PointNear(), utm_points));
}

// Checks that original linestring is returned and error is set if projections cannot be set up
TEST_F(ProjectionFixture, transformLineWithInvalidProjection)
{
    // Arrange
    Projection projection = create_invalid_projection();
    LineStringXyzWgs84 wsg84_line_string{wgs84_points.begin(), wgs84_points.end()};

    // Act
    LineStringXyz utm_line_string = projection.transform_linestring_wgs84_to_utm(wsg84_line_string);

    // Assert
    EXPECT_THAT(projection.get_error_status(), Ne(""));
    EXPECT_THAT(utm_line_string, testing::Pointwise(PointNear(), wsg84_line_string));
}

// Checks that polygon is sucessfully converted from wsgs84 to utm
TEST_F(ProjectionFixture, transformPolygonSucessfully)
{
    // Arrange
    Projection projection = create_projection();
    PolygonXyzWgs84 wsg84_polygon{};
    boost::geometry::append(wsg84_polygon.outer(), wgs84_points);

    // Act
    PolygonXyz utm_polygon = projection.transform_polygon_wgs84_to_utm(wsg84_polygon);

    // Assert
    EXPECT_THAT(utm_polygon.outer(), testing::Pointwise(PointNear(), utm_points));
}

// Checks that original polygon is returned and error is set if projections cannot be set up
TEST_F(ProjectionFixture, transformPolygonWithInvalidProjection)
{
    // Arrange
    Projection projection = create_invalid_projection();
    PolygonXyzWgs84 wsg84_polygon{};
    boost::geometry::append(wsg84_polygon.outer(), wgs84_points);

    // Act
    PolygonXyz utm_polygon = projection.transform_polygon_wgs84_to_utm(wsg84_polygon);

    // Assert
    EXPECT_THAT(projection.get_error_status(), Ne(""));
    EXPECT_THAT(utm_polygon.outer(), testing::Pointwise(PointNear(), wsg84_polygon.outer()));
}

// Checks that nothing happens on projection recreation if there has been no error
TEST_F(ProjectionFixture, createProjectionOnNoError)
{
    // Arrange
    Projection projection = create_projection();

    // Act
    projection.create_projections_on_error();

    // Assert
    EXPECT_THAT(projection.get_error_status(), Eq(""));
}

// Checks that projection are tried to be recreated on previous error. In this case, it should fail again and report the
// same eror as before
TEST_F(ProjectionFixture, createProjectionOnError)
{
    // Arrange
    Projection projection = create_invalid_projection();

    // Expect
    auto current_error = projection.get_error_status();
    EXPECT_CALL(*logger_, error(current_error));

    // Act
    projection.create_projections_on_error();

    // Assert
    EXPECT_THAT(projection.get_error_status(), Eq(current_error));
}