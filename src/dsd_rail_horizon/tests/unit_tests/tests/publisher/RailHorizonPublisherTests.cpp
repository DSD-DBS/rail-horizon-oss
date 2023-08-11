/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "utils/CustomPrintDefinitions.h" // IWYU pragma: keep // Enables more helpful output for custom types
#include "utils/data/StructureData.h"
#include "utils/mocks/ClockMock.h"
#include "utils/mocks/LoggerMock.h"
#include "utils/mocks/PublisherMock.h"

#include <dsd_rail_horizon/publisher/RailHorizonPublisher.h>
#include <dsd_rail_horizon/utils/MessageHelper.h>

#include <dsd_rail_horizon_core/MapModel.h>

#include <dsd_common_types/GeoGeometryTypes.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using ::testing::Eq;
using ::testing::Ne;

namespace
{
template <typename Point>
dsd_ros_messages::msg::GNSSLocalization create_gnss_localization(
    const Point& position, double roll, double pitch, double yaw)
{
    dsd_ros_messages::msg::GNSSLocalization localization;
    localization.position.position.push_back(position.template get<0>());
    localization.position.position.push_back(position.template get<1>());
    localization.position.position.push_back(position.template get<2>());
    localization.orientation.angles.push_back(roll);
    localization.orientation.angles.push_back(pitch);
    localization.orientation.angles.push_back(yaw);
    return localization;
}

dsd_ros_messages::msg::CoupledLocalizationStamped create_coupled_localization_stamped_message()
{
    dsd_ros_messages::msg::CoupledLocalizationStamped coupled_localization_message{};
    coupled_localization_message.header.stamp = rclcpp::Time{50};
    coupled_localization_message.coupled_localization = dsd_ros_messages::msg::CoupledLocalization{};
    coupled_localization_message.coupled_localization.coupled_gnss_localization =
        create_gnss_localization<PointXyz>({1, 2, 3}, 3, 2, 1);
    return coupled_localization_message;
}

dsd_ros_messages::msg::RailHorizonStamped create_rail_horizon_stamped_message(const RailHorizonPublisherConfig& config)
{
    dsd_ros_messages::msg::RailHorizonStamped message{};
    message.header.stamp = rclcpp::Time{50};
    message.header.frame_id = "test_frame_id";
    message.rail_horizon.optional_attributes.data.push_back(dsd_ros_messages::msg::KeyValuePair{});
    message.rail_horizon.optional_attributes.data[0].key = "map_version";
    message.rail_horizon.optional_attributes.data[0].value = "10";
    message.rail_horizon.reference_position.coupled_gnss_localization =
        create_gnss_localization(PointXyz{1, 2, 3}, 3, 2, 1);

    if (config.publish_global_horizon)
    {
        message.rail_horizon.global_horizon.landmarks = {
            create_vertical_landmark<PointXyzWgs84>(10, VerticalStructure::Type::LIGHT, {1, 2, 3}, 1, {4, 5, 6}, 2),
            create_vertical_landmark<PointXyzWgs84>(20, VerticalStructure::Type::SIGN, {2, 4, 6}, 3, {8, 10, 12}, 4),

            create_body_landmark<LineStringXyzWgs84>(
                60, BodyStructure::Type::CONTAINER, {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}}, 100),

            create_plane_landmark<LineStringXyzWgs84>(
                40, PlaneStructure::Type::TUNNEL, {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}}),
        };

        message.rail_horizon.global_horizon.zones = {
            create_zone_landmark<LineStringXyzWgs84>(50, PlaneStructure::Type::ZONE, PlaneStructure::SubType::RISK_ZONE,
                {{2, 4, 6}, {8, 10, 12}, {14, 16, 18}}),
        };

        if (config.rail_horizon_virtual_centerline && config.rail_horizon_left_right_rail)
        {
            message.rail_horizon.global_horizon.tracks = {
                create_track<PointXyzWgs84>(
                    30, 1, 10, {{1, 2, 3}, {4, 5, 6}}, {{7, 8, 9}, {10, 11, 12}}, {{13, 14, 15}, {16, 17, 18}}, false),
                create_track<PointXyzWgs84>(100, 2, 20, {{2, 4, 6}, {8, 10, 12}}, {{14, 16, 18}, {20, 22, 24}},
                    {{26, 28, 30}, {32, 34, 36}}, true),
            };
        }
        else if (config.rail_horizon_virtual_centerline)
        {
            message.rail_horizon.global_horizon.tracks = {
                create_track<PointXyzWgs84>(30, 1, 10, {{1, 2, 3}, {4, 5, 6}}, {}, {}, false),
                create_track<PointXyzWgs84>(100, 2, 20, {{2, 4, 6}, {8, 10, 12}}, {}, {}, true),
            };
        }
        else if (config.rail_horizon_left_right_rail)
        {
            message.rail_horizon.global_horizon.tracks = {
                create_track<PointXyzWgs84>(
                    30, 1, 10, {}, {{7, 8, 9}, {10, 11, 12}}, {{13, 14, 15}, {16, 17, 18}}, false),
                create_track<PointXyzWgs84>(
                    100, 2, 20, {}, {{14, 16, 18}, {20, 22, 24}}, {{26, 28, 30}, {32, 34, 36}}, true),
            };
        }
        else
        {
            message.rail_horizon.global_horizon.tracks = {
                create_track<PointXyzWgs84>(30, 1, 10, {}, {}, {}, false),
                create_track<PointXyzWgs84>(100, 2, 20, {}, {}, {}, true),
            };
        }
        message.rail_horizon.global_horizon.driving_path = {create_id_message(100)};
    }

    if (config.publish_local_horizon)
    {
        message.rail_horizon.local_horizon.landmarks = {
            create_vertical_landmark<PointXyzWgs84>(10, VerticalStructure::Type::LIGHT, {6, 5, 4}, 1, {3, 2, 1}, 2),
            create_vertical_landmark<PointXyzWgs84>(20, VerticalStructure::Type::SIGN, {12, 10, 8}, 3, {6, 4, 2}, 4),

            create_body_landmark<LineStringXyzWgs84>(
                60, BodyStructure::Type::CONTAINER, {{9, 8, 7}, {6, 5, 4}, {3, 2, 1}}, 100),

            create_plane_landmark<LineStringXyzWgs84>(
                40, PlaneStructure::Type::TUNNEL, {{9, 8, 7}, {6, 5, 4}, {3, 2, 1}}),
        };

        message.rail_horizon.local_horizon.zones = {
            create_zone_landmark<LineStringXyzWgs84>(50, PlaneStructure::Type::ZONE, PlaneStructure::SubType::RISK_ZONE,
                {{18, 16, 14}, {12, 10, 8}, {6, 4, 2}}),
        };

        if (config.rail_horizon_virtual_centerline && config.rail_horizon_left_right_rail)
        {
            message.rail_horizon.local_horizon.tracks = {
                create_track<PointXyzWgs84>(
                    30, 1, 10, {{6, 5, 4}, {3, 2, 1}}, {{12, 11, 10}, {9, 8, 7}}, {{15, 14, 13}, {12, 11, 10}}, false),
                create_track<PointXyzWgs84>(100, 2, 20, {{12, 10, 8}, {6, 4, 2}}, {{24, 22, 20}, {18, 16, 14}},
                    {{36, 34, 32}, {30, 28, 26}}, true),
            };
        }
        else if (config.rail_horizon_virtual_centerline)
        {
            message.rail_horizon.local_horizon.tracks = {
                create_track<PointXyzWgs84>(30, 1, 10, {{6, 5, 4}, {3, 2, 1}}, {}, {}, false),
                create_track<PointXyzWgs84>(100, 2, 20, {{12, 10, 8}, {6, 4, 2}}, {}, {}, true),
            };
        }
        else if (config.rail_horizon_left_right_rail)
        {
            message.rail_horizon.local_horizon.tracks = {
                create_track<PointXyzWgs84>(
                    30, 1, 10, {}, {{12, 11, 10}, {9, 8, 7}}, {{15, 14, 13}, {12, 11, 10}}, false),
                create_track<PointXyzWgs84>(
                    100, 2, 20, {}, {{24, 22, 20}, {18, 16, 14}}, {{36, 34, 32}, {30, 28, 26}}, true),
            };
        }
        else
        {
            message.rail_horizon.local_horizon.tracks = {
                create_track<PointXyzWgs84>(30, 1, 10, {}, {}, {}, false),
                create_track<PointXyzWgs84>(100, 2, 20, {}, {}, {}, true),
            };
        }
        message.rail_horizon.local_horizon.driving_path = {create_id_message(100)};
    }

    return message;
}
} // namespace

class RailHorizonPublisherExposed : public RailHorizonPublisher
{
public:
    using RailHorizonPublisher::RailHorizonPublisher;

    using RailHorizonPublisher::rail_horizon_publisher_;
};

class RailHorizonPublisherFixture : public ::testing::TestWithParam<RailHorizonPublisherConfig>
{
public:
    RailHorizonPublisherFixture()
    {
        rclcpp::init(0, nullptr);
        parent_node_ = std::make_shared<rclcpp::Node>("test_node");
        publisher_mock_ = std::make_shared<PublisherMock<dsd_ros_messages::msg::RailHorizonStamped>>();
    }

    std::shared_ptr<RailHorizonPublisherExposed> create_publisher(const RailHorizonPublisherConfig& config_)
    {
        auto publisher = std::make_shared<RailHorizonPublisherExposed>(*parent_node_, config_);
        publisher->rail_horizon_publisher_ = publisher_mock_;
        return publisher;
    }

protected:
    std::shared_ptr<rclcpp::Node> parent_node_;
    std::shared_ptr<PublisherMock<dsd_ros_messages::msg::RailHorizonStamped>> publisher_mock_;

    RailHorizonPublisherConfig config_ =
        RailHorizonPublisherConfig{"test_topic", 3, "test_frame_id", false, false, true, true};

    std::vector<Id> track_ids_ = {100};
    std::string map_version_ = "10";
    // TODO(leon): Have meaningful transformation here
    GeometryTransformation local_transformation{PointXyz{0, 0, 0}, boost::qvm::identity_mat<double, 4>()};
    StructureContainer structures_ = create_structures();
    dsd_ros_messages::msg::CoupledLocalizationStamped coupled_localization_message_ =
        create_coupled_localization_stamped_message();
};

// Test publish method without global or local horizon
TEST_F(RailHorizonPublisherFixture, publishNoHorizon)
{
    // ARRANGE
    auto rail_horizon_publisher = create_publisher(config_);

    // EXPECT
    auto expected_message = create_rail_horizon_stamped_message(config_);
    EXPECT_CALL(*publisher_mock_, publish(expected_message));

    // ACT
    rail_horizon_publisher->publish(
        structures_, local_transformation, coupled_localization_message_, track_ids_, map_version_);
}

// Test publish method with only global horizon
TEST_F(RailHorizonPublisherFixture, publishGlobalHorizon)
{
    // ARRANGE
    config_.publish_global_horizon = true;
    auto rail_horizon_publisher = create_publisher(config_);

    // EXPECT
    auto expected_message = create_rail_horizon_stamped_message(config_);
    EXPECT_CALL(*publisher_mock_, publish(expected_message));

    // ACT
    rail_horizon_publisher->publish(
        structures_, local_transformation, coupled_localization_message_, track_ids_, map_version_);
}

// Test publish method with only local horizon
TEST_F(RailHorizonPublisherFixture, publishLocalHorizon)
{
    // ARRANGE
    config_.publish_local_horizon = true;
    auto rail_horizon_publisher = create_publisher(config_);

    // EXPECT
    auto expected_message = create_rail_horizon_stamped_message(config_);
    EXPECT_CALL(*publisher_mock_, publish(expected_message));

    // ACT
    rail_horizon_publisher->publish(
        structures_, local_transformation, coupled_localization_message_, track_ids_, map_version_);
}

// Test publish method without centerline track
TEST_F(RailHorizonPublisherFixture, publishWithoutCenterlineTrack)
{
    config_.publish_local_horizon = true;
    config_.publish_global_horizon = true;
    config_.rail_horizon_virtual_centerline = false;
    // ARRANGE
    auto rail_horizon_publisher = create_publisher(config_);

    // EXPECT
    auto expected_message = create_rail_horizon_stamped_message(config_);
    EXPECT_CALL(*publisher_mock_, publish(expected_message));

    // ACT
    rail_horizon_publisher->publish(
        structures_, local_transformation, coupled_localization_message_, track_ids_, map_version_);
}

// Test publish method without left and right track
TEST_F(RailHorizonPublisherFixture, publishWithourLeftRightTrack)
{
    config_.publish_local_horizon = true;
    config_.publish_global_horizon = true;
    config_.rail_horizon_left_right_rail = false;
    // ARRANGE
    auto rail_horizon_publisher = create_publisher(config_);

    // EXPECT
    auto expected_message = create_rail_horizon_stamped_message(config_);
    EXPECT_CALL(*publisher_mock_, publish(expected_message));

    // ACT
    rail_horizon_publisher->publish(
        structures_, local_transformation, coupled_localization_message_, track_ids_, map_version_);
}

// Test publish method without centerline, left and right track
TEST_F(RailHorizonPublisherFixture, publishWithoutTrack)
{
    config_.publish_local_horizon = true;
    config_.publish_global_horizon = true;
    config_.rail_horizon_left_right_rail = false;
    config_.rail_horizon_virtual_centerline = false;
    // ARRANGE
    auto rail_horizon_publisher = create_publisher(config_);

    // EXPECT
    auto expected_message = create_rail_horizon_stamped_message(config_);
    EXPECT_CALL(*publisher_mock_, publish(expected_message));

    // ACT
    rail_horizon_publisher->publish(
        structures_, local_transformation, coupled_localization_message_, track_ids_, map_version_);
}
