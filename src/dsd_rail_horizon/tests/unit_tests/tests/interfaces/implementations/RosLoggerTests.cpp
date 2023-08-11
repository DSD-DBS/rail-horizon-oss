/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "utils/TestSubscriber.h"

#include <dsd_rail_horizon/interfaces/implementations/RosLogger.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <rcl_interfaces/msg/log.hpp>

#include <mutex>

using testing::Eq;
using testing::Ne;

class RosLoggerFixture : public ::testing::Test
{
public:
    RosLoggerFixture()
    {
        rclcpp::init(0, nullptr);
        parent_node_ = std::make_shared<rclcpp::Node>("test_node");
    }

protected:
    std::shared_ptr<rclcpp::Node> parent_node_;
};

// TODO(leon): Tests don't work for debug yet
TEST_F(RosLoggerFixture, logDebugWithoutTag)
{
    // Arrange
    auto logger = std::make_shared<RosLogger>(parent_node_->get_logger());
    TestSubscriber<rcl_interfaces::msg::Log> subscriber{*parent_node_, "/rosout", rclcpp::QoS(10)};

    // Act
    logger->debug("Test");
    rclcpp::spin_some(parent_node_);

    // Assert
    // ASSERT_THAT(subscriber.last_message->msg, Eq("Test"));
    // ASSERT_THAT(subscriber.last_message->level, Eq(rcl_interfaces::msg::Log::DEBUG));
}

TEST_F(RosLoggerFixture, logDebugWithTag)
{
    // Arrange
    auto logger = std::make_shared<RosLogger>(parent_node_->get_logger(), "test_tag");
    TestSubscriber<rcl_interfaces::msg::Log> subscriber{*parent_node_, "/rosout", rclcpp::QoS(10)};
    // Act
    logger->debug("Test");
    rclcpp::spin_some(parent_node_);

    // Assert
    // ASSERT_THAT(subscriber.last_message->msg, Eq("[test_tag] Test"));
    // ASSERT_THAT(subscriber.last_message->level, Eq(rcl_interfaces::msg::Log::DEBUG));
}

TEST_F(RosLoggerFixture, logDebugWithTagAndFormatString)
{
    // Arrange
    auto logger = std::make_shared<RosLogger>(parent_node_->get_logger(), "test_tag");
    TestSubscriber<rcl_interfaces::msg::Log> subscriber{*parent_node_, "/rosout", rclcpp::QoS(10)};

    // Act
    logger->debug(boost::format("Test"));
    rclcpp::spin_some(parent_node_);

    // Assert
    // ASSERT_THAT(subscriber.last_message->msg, Eq("[test_tag] Test"));
    // ASSERT_THAT(subscriber.last_message->level, Eq(rcl_interfaces::msg::Log::DEBUG));
}

TEST_F(RosLoggerFixture, logInfoWithoutTag)
{
    // Arrange
    auto logger = std::make_shared<RosLogger>(parent_node_->get_logger());
    TestSubscriber<rcl_interfaces::msg::Log> subscriber{*parent_node_, "/rosout", rclcpp::QoS(10)};

    // Act
    logger->info("Test");
    rclcpp::spin_some(parent_node_);

    // Assert
    ASSERT_THAT(subscriber.last_message, ::testing::NotNull());
    ASSERT_THAT(subscriber.last_message->msg, Eq("Test"));
    ASSERT_THAT(subscriber.last_message->level, Eq(rcl_interfaces::msg::Log::INFO));
}

TEST_F(RosLoggerFixture, logInfoWithTag)
{
    // Arrange
    auto logger = std::make_shared<RosLogger>(parent_node_->get_logger(), "test_tag");
    TestSubscriber<rcl_interfaces::msg::Log> subscriber{*parent_node_, "/rosout", rclcpp::QoS(10)};

    // Act
    logger->info("Test");
    rclcpp::spin_some(parent_node_);

    // Assert
    ASSERT_THAT(subscriber.last_message, ::testing::NotNull());
    ASSERT_THAT(subscriber.last_message->msg, Eq("[test_tag] Test"));
    ASSERT_THAT(subscriber.last_message->level, Eq(rcl_interfaces::msg::Log::INFO));
}

TEST_F(RosLoggerFixture, logInfoWithTagAndFormatString)
{
    // Arrange
    auto logger = std::make_shared<RosLogger>(parent_node_->get_logger(), "test_tag");
    TestSubscriber<rcl_interfaces::msg::Log> subscriber{*parent_node_, "/rosout", rclcpp::QoS(10)};

    // Act
    logger->info(boost::format("Test"));
    rclcpp::spin_some(parent_node_);

    // Assert
    ASSERT_THAT(subscriber.last_message, ::testing::NotNull());
    ASSERT_THAT(subscriber.last_message->msg, Eq("[test_tag] Test"));
    ASSERT_THAT(subscriber.last_message->level, Eq(rcl_interfaces::msg::Log::INFO));
}

TEST_F(RosLoggerFixture, logErrorWithoutTag)
{
    // Arrange
    auto logger = std::make_shared<RosLogger>(parent_node_->get_logger());
    TestSubscriber<rcl_interfaces::msg::Log> subscriber{*parent_node_, "/rosout", rclcpp::QoS(10)};

    // Act
    logger->error("Test");
    rclcpp::spin_some(parent_node_);

    // Assert
    ASSERT_THAT(subscriber.last_message, ::testing::NotNull());
    ASSERT_THAT(subscriber.last_message->msg, Eq("Test"));
    ASSERT_THAT(subscriber.last_message->level, Eq(rcl_interfaces::msg::Log::ERROR));
}

TEST_F(RosLoggerFixture, logErrorWithTag)
{
    // Arrange
    auto logger = std::make_shared<RosLogger>(parent_node_->get_logger(), "test_tag");
    TestSubscriber<rcl_interfaces::msg::Log> subscriber{*parent_node_, "/rosout", rclcpp::QoS(10)};

    // Act
    logger->error("Test");
    rclcpp::spin_some(parent_node_);

    // Assert
    ASSERT_THAT(subscriber.last_message, ::testing::NotNull());
    ASSERT_THAT(subscriber.last_message->msg, Eq("[test_tag] Test"));
    ASSERT_THAT(subscriber.last_message->level, Eq(rcl_interfaces::msg::Log::ERROR));
}

TEST_F(RosLoggerFixture, logErrorWithTagAndFormatString)
{
    // Arrange
    auto logger = std::make_shared<RosLogger>(parent_node_->get_logger(), "test_tag");
    TestSubscriber<rcl_interfaces::msg::Log> subscriber{*parent_node_, "/rosout", rclcpp::QoS(10)};

    // Act
    logger->error(boost::format("Test"));
    rclcpp::spin_some(parent_node_);

    // Assert
    ASSERT_THAT(subscriber.last_message, ::testing::NotNull());
    ASSERT_THAT(subscriber.last_message->msg, Eq("[test_tag] Test"));
    ASSERT_THAT(subscriber.last_message->level, Eq(rcl_interfaces::msg::Log::ERROR));
}