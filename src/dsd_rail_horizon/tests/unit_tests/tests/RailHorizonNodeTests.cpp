/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "utils/CustomPrintDefinitions.h" // IWYU pragma: keep // Enables more helpful output for custom types
#include "utils/mocks/LoggerMock.h"
#include "utils/TestSubscriber.h"

#include <dsd_rail_horizon/interfaces/implementations/RosPublisher.h>
#include <dsd_rail_horizon/RailHorizonNode.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <visualization_msgs/msg/marker_array.hpp>

using testing::Eq;
using testing::HasSubstr;
using testing::Matcher;
using testing::Ne;

class RailHorizonNodeExposed : public RailHorizonNode
{
public:
    using RailHorizonNode::RailHorizonNode;

    using RailHorizonNode::rail_horizon_periodic_broadcast_callback;

    using RailHorizonNode::logger_;
    using RailHorizonNode::parameters_;
};

class RailHorizonNodeFixture : public ::testing::Test
{
public:
    RailHorizonNodeFixture()
    {
        rclcpp::init(0, nullptr);
    }

    std::shared_ptr<RailHorizonNodeExposed> get_rail_horizon_node()
    {
        auto node = std::make_shared<RailHorizonNodeExposed>(node_options_);
        auto params = node->parameters_;
        rail_horizon_subscriber_ = std::make_shared<TestSubscriber<dsd_ros_messages::msg::RailHorizonStamped>>(
            *node, params->rail_horizon_namespace_ + params->rail_horizon_topic_, params->dds_qos_);
        position_marker_subscriber_ = std::make_shared<TestSubscriber<visualization_msgs::msg::Marker>>(
            *node, params->rail_horizon_namespace_ + params->rail_horizon_marker_position_topic_, params->dds_qos_);
        vertical_structure_marker_subscriber_ = std::make_shared<TestSubscriber<visualization_msgs::msg::MarkerArray>>(
            *node, params->rail_horizon_namespace_ + params->rail_horizon_marker_vertical_structure_topic_,
            params->dds_qos_);
        horizontal_structure_marker_subscriber_ =
            std::make_shared<TestSubscriber<visualization_msgs::msg::MarkerArray>>(*node,
                params->rail_horizon_namespace_ + params->rail_horizon_marker_horizontal_structure_topic_,
                params->dds_qos_);
        plane_structure_marker_subscriber_ =
            std::make_shared<TestSubscriber<visualization_msgs::msg::MarkerArray>>(*node,
                params->rail_horizon_namespace_ + params->rail_horizon_marker_plane_structure_topic_, params->dds_qos_);
        body_structure_marker_subscriber_ =
            std::make_shared<TestSubscriber<visualization_msgs::msg::MarkerArray>>(*node,
                params->rail_horizon_namespace_ + params->rail_horizon_marker_body_structure_topic_, params->dds_qos_);

        coupled_localization_publisher_ =
            std::make_shared<RosPublisher<dsd_ros_messages::msg::CoupledLocalizationStamped>>(
                *node, params->coupled_localization_topic_, params->dds_qos_);

        return node;
    }

protected:
    rclcpp::NodeOptions node_options_{};
    std::shared_ptr<LoggerMock> logger_ = std::make_shared<LoggerMock>();

    std::shared_ptr<TestSubscriber<dsd_ros_messages::msg::RailHorizonStamped>> rail_horizon_subscriber_;
    std::shared_ptr<TestSubscriber<visualization_msgs::msg::Marker>> position_marker_subscriber_;
    std::shared_ptr<TestSubscriber<visualization_msgs::msg::MarkerArray>> vertical_structure_marker_subscriber_;
    std::shared_ptr<TestSubscriber<visualization_msgs::msg::MarkerArray>> horizontal_structure_marker_subscriber_;
    std::shared_ptr<TestSubscriber<visualization_msgs::msg::MarkerArray>> plane_structure_marker_subscriber_;
    std::shared_ptr<TestSubscriber<visualization_msgs::msg::MarkerArray>> body_structure_marker_subscriber_;

    IPublisher<dsd_ros_messages::msg::CoupledLocalizationStamped>::SharedPtr coupled_localization_publisher_;
};

TEST_F(RailHorizonNodeFixture, periodicCallbackNoLocalizationAvailable)
{
    // Arrange
    auto rail_horizon_node = get_rail_horizon_node();
    rail_horizon_node->logger_ = logger_;

    // Expect
    EXPECT_CALL(*logger_, info(Matcher<const std::string&>(HasSubstr("Received no position update"))));

    // Act
    rail_horizon_node->rail_horizon_periodic_broadcast_callback();
    rclcpp::spin_some(rail_horizon_node);

    // Assert
    EXPECT_THAT(rail_horizon_subscriber_->last_message, Eq(nullptr));
    EXPECT_THAT(position_marker_subscriber_->last_message, Eq(nullptr));
    EXPECT_THAT(vertical_structure_marker_subscriber_->last_message, Eq(nullptr));
    EXPECT_THAT(horizontal_structure_marker_subscriber_->last_message, Eq(nullptr));
    EXPECT_THAT(plane_structure_marker_subscriber_->last_message, Eq(nullptr));
    EXPECT_THAT(body_structure_marker_subscriber_->last_message, Eq(nullptr));
}

TEST_F(RailHorizonNodeFixture, periodicCallbackWithAvailableData)
{
    // Arrange
    auto rail_horizon_node = get_rail_horizon_node();

    coupled_localization_publisher_->publish(dsd_ros_messages::msg::CoupledLocalizationStamped{});
    rclcpp::spin_some(rail_horizon_node);

    // Act
    rail_horizon_node->rail_horizon_periodic_broadcast_callback();
    rclcpp::spin_some(rail_horizon_node);

    // Assert
    EXPECT_THAT(rail_horizon_subscriber_->last_message, Ne(nullptr));
    EXPECT_THAT(position_marker_subscriber_->last_message, Ne(nullptr));
    EXPECT_THAT(vertical_structure_marker_subscriber_->last_message, Ne(nullptr));
    EXPECT_THAT(horizontal_structure_marker_subscriber_->last_message, Ne(nullptr));
    EXPECT_THAT(plane_structure_marker_subscriber_->last_message, Ne(nullptr));
    EXPECT_THAT(body_structure_marker_subscriber_->last_message, Ne(nullptr));
}

TEST_F(RailHorizonNodeFixture, periodicCallbackWithAvailableDataNoMarkers)
{
    // Arrange
    node_options_.append_parameter_override("rail_horizon_markers", false);
    auto rail_horizon_node = get_rail_horizon_node();

    coupled_localization_publisher_->publish(dsd_ros_messages::msg::CoupledLocalizationStamped{});
    rclcpp::spin_some(rail_horizon_node);

    // Act
    rail_horizon_node->rail_horizon_periodic_broadcast_callback();
    rclcpp::spin_some(rail_horizon_node);

    // Assert
    EXPECT_THAT(rail_horizon_subscriber_->last_message, Ne(nullptr));
    EXPECT_THAT(position_marker_subscriber_->last_message, Eq(nullptr));
    EXPECT_THAT(vertical_structure_marker_subscriber_->last_message, Eq(nullptr));
    EXPECT_THAT(horizontal_structure_marker_subscriber_->last_message, Eq(nullptr));
    EXPECT_THAT(plane_structure_marker_subscriber_->last_message, Eq(nullptr));
    EXPECT_THAT(body_structure_marker_subscriber_->last_message, Eq(nullptr));
}

TEST_F(RailHorizonNodeFixture, periodicCallbackWithSupervisionAppStandby)
{
    // Arrange
    node_options_.append_parameter_override("mission_control_supervision", true);
    auto rail_horizon_node = get_rail_horizon_node();

    coupled_localization_publisher_->publish(dsd_ros_messages::msg::CoupledLocalizationStamped{});
    rclcpp::spin_some(rail_horizon_node);

    // Act
    rail_horizon_node->rail_horizon_periodic_broadcast_callback();
    rclcpp::spin_some(rail_horizon_node);

    // Assert
    EXPECT_THAT(rail_horizon_subscriber_->last_message, Eq(nullptr));
}