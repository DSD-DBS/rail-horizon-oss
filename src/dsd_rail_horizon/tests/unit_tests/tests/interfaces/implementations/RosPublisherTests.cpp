/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "utils/TestSubscriber.h"

#include <dsd_rail_horizon/interfaces/implementations/RosPublisher.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

using testing::Eq;

class RosPublisherFixture : public ::testing::Test
{
public:
    RosPublisherFixture()
    {
        rclcpp::init(0, nullptr);
        parent_node_ = std::make_shared<rclcpp::Node>("test_node");
    }

protected:
    std::shared_ptr<rclcpp::Node> parent_node_;
};

TEST_F(RosPublisherFixture, publishesMessage)
{
    // Arrange
    RosPublisher<std_msgs::msg::String> publisher{*parent_node_, "test_topic", rclcpp::QoS(10)};
    TestSubscriber<std_msgs::msg::String> subscriber{*parent_node_, "test_topic", rclcpp::QoS(10)};

    std_msgs::msg::String test_message{};
    test_message.data = "Test";

    // Act
    publisher.publish(test_message);
    rclcpp::spin_some(parent_node_);

    // Assert
    ASSERT_THAT(subscriber.last_message, ::testing::NotNull());
    ASSERT_THAT(*subscriber.last_message, Eq(test_message));
}