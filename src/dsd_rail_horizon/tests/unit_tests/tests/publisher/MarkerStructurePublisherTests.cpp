/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "utils/CustomPrintDefinitions.h" // IWYU pragma: keep // Enables more helpful output for custom types
#include "utils/data/StructureData.h"
#include "utils/mocks/ClockMock.h"
#include "utils/mocks/PublisherMock.h"

#include <dsd_rail_horizon/publisher/MarkerStructurePublisher.h>
#include <dsd_rail_horizon/utils/MessageHelper.h>

#include <dsd_common_types/GeometryTypes.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <visualization_msgs/msg/marker_array.hpp>

using ::testing::Eq;
using ::testing::Ne;

namespace
{
visualization_msgs::msg::Marker create_marker_message(
    const MarkerStructurePublisherConfig& config, const Id& id, int32_t type, const std_msgs::msg::ColorRGBA& color)
{
    visualization_msgs::msg::Marker marker{};
    marker.header.frame_id = config.marker_reference_frame;
    marker.header.stamp = rclcpp::Time{100};
    marker.ns = "default";
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale = create_scale_message(0.2, 0.2, 0.2);
    marker.color = color;
    return marker;
}
} // namespace

class MarkerStructurePublisherExposed : public MarkerStructurePublisher
{
public:
    using MarkerStructurePublisher::MarkerStructurePublisher;

    using MarkerStructurePublisher::body_structure_marker_publisher_;
    using MarkerStructurePublisher::horizontal_structure_marker_publisher_;
    using MarkerStructurePublisher::plane_structure_marker_publisher_;
    using MarkerStructurePublisher::position_marker_publisher_;
    using MarkerStructurePublisher::vertical_structure_marker_publisher_;
};

class MarkerStructurePublisherFixture : public ::testing::Test
{
public:
    MarkerStructurePublisherFixture()
    {
        rclcpp::init(0, nullptr);
        parent_node_ = std::make_shared<rclcpp::Node>("test_node");
        position_marker_mock_ = std::make_shared<PublisherMock<visualization_msgs::msg::Marker>>();
        vertical_structure_marker_mock_ = std::make_shared<PublisherMock<visualization_msgs::msg::MarkerArray>>();
        horizontal_structure_marker_mock_ = std::make_shared<PublisherMock<visualization_msgs::msg::MarkerArray>>();
        plane_structure_marker_mock_ = std::make_shared<PublisherMock<visualization_msgs::msg::MarkerArray>>();
        body_structure_marker_mock_ = std::make_shared<PublisherMock<visualization_msgs::msg::MarkerArray>>();

        clock_ = std::make_shared<ClockMock>();
        rclcpp::Time time{100};
        ON_CALL(*clock_, now()).WillByDefault(testing::Return(time));
    }

    std::shared_ptr<MarkerStructurePublisherExposed> create_publisher(const MarkerStructurePublisherConfig& config_)
    {
        auto publisher = std::make_shared<MarkerStructurePublisherExposed>(*parent_node_, config_, clock_);
        publisher->position_marker_publisher_ = position_marker_mock_;
        publisher->vertical_structure_marker_publisher_ = vertical_structure_marker_mock_;
        publisher->horizontal_structure_marker_publisher_ = horizontal_structure_marker_mock_;
        publisher->plane_structure_marker_publisher_ = plane_structure_marker_mock_;
        publisher->body_structure_marker_publisher_ = body_structure_marker_mock_;
        return publisher;
    }

protected:
    MarkerStructurePublisherConfig config_{"t1", "t2", "t3", "t4", "t5", 10, "test_reference_frame", true, true};
    std::shared_ptr<rclcpp::Node> parent_node_;
    std::shared_ptr<ClockMock> clock_;

    std::shared_ptr<PublisherMock<visualization_msgs::msg::Marker>> position_marker_mock_;
    std::shared_ptr<PublisherMock<visualization_msgs::msg::MarkerArray>> vertical_structure_marker_mock_;
    std::shared_ptr<PublisherMock<visualization_msgs::msg::MarkerArray>> horizontal_structure_marker_mock_;
    std::shared_ptr<PublisherMock<visualization_msgs::msg::MarkerArray>> plane_structure_marker_mock_;
    std::shared_ptr<PublisherMock<visualization_msgs::msg::MarkerArray>> body_structure_marker_mock_;

    StructureContainer structures_ = create_structures();
    // TODO(leon): Have meaningful transformation here
    GeometryTransformation local_transformation_{PointXyz{0, 0, 0}, boost::qvm::identity_mat<double, 4>()};
};


TEST_F(MarkerStructurePublisherFixture, publishPositionMarker)
{
    // Arrange
    auto marker_structure_publisher = create_publisher(config_);

    // Expect
    auto expected_message = create_marker_message(
        config_, 0, visualization_msgs::msg::Marker::CUBE, create_color_message(1.0, 0.0, 0.0, 1.0));
    EXPECT_CALL(*position_marker_mock_, publish(expected_message));

    // Act
    marker_structure_publisher->publish(structures_, local_transformation_);
}

TEST_F(MarkerStructurePublisherFixture, publishVerticalStructureMarker)
{
    // Arrange
    auto marker_structure_publisher = create_publisher(config_);

    visualization_msgs::msg::Marker m1 = create_marker_message(
        config_, 10, visualization_msgs::msg::Marker::LINE_LIST, create_color_message(0.0, 1.0, 0.0));
    m1.points = geometry_to_point_vector<LineStringXyz>({{6, 5, 4}, {3, 2, 1}});

    visualization_msgs::msg::Marker m2 = create_marker_message(
        config_, 20, visualization_msgs::msg::Marker::LINE_LIST, create_color_message(0.0, 1.0, 0.0));
    m2.points = geometry_to_point_vector<LineStringXyz>({{12, 10, 8}, {6, 4, 2}});

    // Expect
    auto expected_message = visualization_msgs::msg::MarkerArray{};
    expected_message.markers = {m1, m2};
    EXPECT_CALL(*vertical_structure_marker_mock_, publish(expected_message));

    // Act
    marker_structure_publisher->publish(structures_, local_transformation_);
}

visualization_msgs::msg::MarkerArray create_expected_horizontal_marker_message(
    const MarkerStructurePublisherConfig& config)
{
    auto message = visualization_msgs::msg::MarkerArray{};

    visualization_msgs::msg::Marker mc1 = create_marker_message(
        config, 30, visualization_msgs::msg::Marker::LINE_STRIP, create_color_message(0.0, 0.0, 1.0));
    mc1.ns = "track_center";
    mc1.points = geometry_to_point_vector<LineStringXyz>({{6, 5, 4}, {3, 2, 1}});

    visualization_msgs::msg::Marker ml1 = create_marker_message(
        config, 30, visualization_msgs::msg::Marker::LINE_STRIP, create_color_message(0.0, 0.6, 0.9));
    ml1.ns = "track_left";
    ml1.points = geometry_to_point_vector<LineStringXyz>({{12, 11, 10}, {9, 8, 7}});

    visualization_msgs::msg::Marker mr1 = create_marker_message(
        config, 30, visualization_msgs::msg::Marker::LINE_STRIP, create_color_message(0.0, 1.0, 1.0));
    mr1.ns = "track_right";
    mr1.points = geometry_to_point_vector<LineStringXyz>({{15, 14, 13}, {12, 11, 10}});

    visualization_msgs::msg::Marker mc2 = create_marker_message(
        config, 100, visualization_msgs::msg::Marker::LINE_STRIP, create_color_message(0.0, 0.0, 1.0));
    mc2.ns = "track_center";
    mc2.points = geometry_to_point_vector<LineStringXyz>({{12, 10, 8}, {6, 4, 2}});


    visualization_msgs::msg::Marker ml2 = create_marker_message(
        config, 100, visualization_msgs::msg::Marker::LINE_STRIP, create_color_message(0.0, 0.6, 0.9));
    ml2.ns = "track_left";
    ml2.points = geometry_to_point_vector<LineStringXyz>({{24, 22, 20}, {18, 16, 14}});

    visualization_msgs::msg::Marker mr2 = create_marker_message(
        config, 100, visualization_msgs::msg::Marker::LINE_STRIP, create_color_message(0.0, 1.0, 1.0));
    mr2.ns = "track_right";
    mr2.points = geometry_to_point_vector<LineStringXyz>({{36, 34, 32}, {30, 28, 26}});

    if (config.rail_horizon_virtual_centerline && config.rail_horizon_left_right_rail)
    {
        message.markers = {mc1, ml1, mr1, mc2, ml2, mr2};
    }
    else if (config.rail_horizon_virtual_centerline)
    {
        message.markers = {mc1, mc2};
    }
    else if (config.rail_horizon_left_right_rail)
    {
        message.markers = {ml1, mr1, ml2, mr2};
    }
    return message;
}

class HorizontalMarkerStructurePublisherFixture : public MarkerStructurePublisherFixture,
                                                  public testing::WithParamInterface<MarkerStructurePublisherConfig>
{
};

TEST_P(HorizontalMarkerStructurePublisherFixture, publishHorizontalStructureMarker)
{
    // Arrange
    auto config = GetParam();
    auto marker_structure_publisher = create_publisher(config);

    // Expect
    auto expected_message = create_expected_horizontal_marker_message(config);
    EXPECT_CALL(*horizontal_structure_marker_mock_, publish(expected_message));

    // Act
    marker_structure_publisher->publish(structures_, local_transformation_);
}

INSTANTIATE_TEST_SUITE_P(HorizontalMarkerTestSuite, HorizontalMarkerStructurePublisherFixture,
    testing::Values(
        MarkerStructurePublisherConfig{"t1", "t2", "t3", "t4", "t5", 10, "test_reference_frame", true, true},
        MarkerStructurePublisherConfig{"t1", "t2", "t3", "t4", "t5", 10, "test_reference_frame", true, false},
        MarkerStructurePublisherConfig{"t1", "t2", "t3", "t4", "t5", 10, "test_reference_frame", false, true},
        MarkerStructurePublisherConfig{"t1", "t2", "t3", "t4", "t5", 10, "test_reference_frame", false, false}),
    [](const testing::TestParamInfo<HorizontalMarkerStructurePublisherFixture::ParamType>& info) {
    std::string name = "";
    if (info.param.rail_horizon_virtual_centerline && info.param.rail_horizon_left_right_rail)
    {
        return "enableVirtualCenterlineAndLeftRightRail";
    }
    if (info.param.rail_horizon_virtual_centerline)
    {
        return "enableVirtualCenterline";
    }
    if (info.param.rail_horizon_left_right_rail)
    {
        return "enableLeftRightRail";
    }
    return "withoutTracks";
});

visualization_msgs::msg::MarkerArray create_expected_plane_marker_message(const MarkerStructurePublisherConfig& config)
{
    auto p1 = create_marker_message(
        config, 40, visualization_msgs::msg::Marker::LINE_STRIP, create_color_message(0.6, 0.3, 0.0));
    p1.ns = "plane_other";
    p1.points = geometry_to_point_vector<LineStringXyz>({{{9, 8, 7}, {6, 5, 4}, {3, 2, 1}}});

    auto p2 = create_marker_message(
        config, 50, visualization_msgs::msg::Marker::LINE_STRIP, create_color_message(1.0, 0.0, 0.0));
    p2.ns = "plane_risk_zone";
    p2.points = geometry_to_point_vector<LineStringXyz>({{{18, 16, 14}, {12, 10, 8}, {6, 4, 2}}});

    auto message = visualization_msgs::msg::MarkerArray{};
    message.markers = {p1, p2};
    return message;
}

TEST_F(MarkerStructurePublisherFixture, publishPlaneStructureMarker)
{
    // Arrange
    auto marker_structure_publisher = create_publisher(config_);

    // Expect
    auto expected_message = create_expected_plane_marker_message(config_);
    EXPECT_CALL(*plane_structure_marker_mock_, publish(expected_message));

    // Act
    marker_structure_publisher->publish(structures_, local_transformation_);
}

visualization_msgs::msg::MarkerArray create_expected_body_marker_message(const MarkerStructurePublisherConfig& config)
{
    auto b1 = create_marker_message(
        config, 60, visualization_msgs::msg::Marker::LINE_STRIP, create_color_message(1.0, 0.0, 1.0));
    b1.points = geometry_to_point_vector<LineStringXyz>({{{9, 8, 7}, {6, 5, 4}, {3, 2, 1}}});


    auto message = visualization_msgs::msg::MarkerArray{};
    message.markers = {b1};
    return message;
}

TEST_F(MarkerStructurePublisherFixture, publishBodyStructureMarker)
{
    // Arrange
    auto marker_structure_publisher = create_publisher(config_);

    // Expect
    auto expected_message = create_expected_body_marker_message(config_);
    EXPECT_CALL(*body_structure_marker_mock_, publish(expected_message));

    // Act
    marker_structure_publisher->publish(structures_, local_transformation_);
}