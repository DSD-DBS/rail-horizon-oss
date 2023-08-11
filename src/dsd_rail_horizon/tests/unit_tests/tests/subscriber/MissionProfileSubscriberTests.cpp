/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "utils/CustomPrintDefinitions.h" // IWYU pragma: keep // Enables more helpful output for custom types
#include "utils/mocks/LoggerMock.h"

#include <dsd_rail_horizon/subscriber/MissionProfileSubscriber.h>

#include <dsd_common_types/GeometryTypes.h>

#include <dsd_ros_messages/msg/id.hpp>
#include <dsd_ros_messages/msg/mission_profile.hpp>
#include <dsd_ros_messages/msg/mission_profile_stamped.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

using ::testing::Eq;

class MissionProfileSubscriberFixture : public ::testing::Test
{
public:
    using Publisher = rclcpp::Publisher<MissionProfileSubscriber::MissionProfileStampedMessage>;
    using Subscriber = MissionProfileSubscriber;
    using Config = MissionProfileSubscriberConfig;

    MissionProfileSubscriberFixture()
    {
        rclcpp::init(0, nullptr);
        parent_node_ = std::make_shared<rclcpp::Node>("test_node");
    }

    Config create_default_config()
    {
        return Config{"test", 1};
    }

    std::tuple<Publisher::SharedPtr, std::shared_ptr<Subscriber>> create_pub_sub_with_config(const Config& config)
    {
        auto publisher = parent_node_->create_publisher<MissionProfileSubscriber::MissionProfileStampedMessage>(
            config.topic_name, config.qos);
        auto subscriber = std::make_shared<MissionProfileSubscriber>(*parent_node_, config, logger_);
        return {publisher, subscriber};
    }

    MissionProfileSubscriber::MissionProfileStampedMessage create_mission_profile_stamped_message()
    {
        dsd_ros_messages::msg::MissionProfile mission_profile_message{};
        mission_profile_message.track_ids = {create_id_message(10), create_id_message(30), create_id_message(20)};

        MissionProfileSubscriber::MissionProfileStampedMessage message{};
        message.seq = 20;
        message.stamp = rclcpp::Time();
        message.mission_profile = mission_profile_message;
        return message;
    }

    inline dsd_ros_messages::msg::ID create_id_message(size_t id)
    {
        dsd_ros_messages::msg::ID id_message;
        id_message.id = id;
        return id_message;
    }

protected:
    std::shared_ptr<rclcpp::Node> parent_node_;

    std::shared_ptr<LoggerMock> logger_ = std::make_shared<LoggerMock>();
};

// Checks that no data is available at startup or when publisher has not sent anything and thus a info message is logged
// and track ids are empty
TEST_F(MissionProfileSubscriberFixture, noTrackIdsAvailableAtStartup)
{
    // Arrange
    auto [_, subscriber] = create_pub_sub_with_config(create_default_config());

    // Expect
    EXPECT_CALL(*logger_, info("Received no mission profile update ..."));

    // Act
    auto result = subscriber->get_track_ids();

    // Assert
    EXPECT_THAT(result, Eq(std::vector<Id>{}));
}

// Checks that data is received after publisher has sent message and tracks id are extracted correctly
TEST_F(MissionProfileSubscriberFixture, trackIdsAvailableAfterPublishedMessage)
{
    // Arrange
    auto [publisher, subscriber] = create_pub_sub_with_config(create_default_config());
    auto message = create_mission_profile_stamped_message();

    // Act
    publisher->publish(message);
    rclcpp::spin_some(parent_node_);

    // Assert
    EXPECT_THAT(subscriber->get_track_ids(), Eq(std::vector<Id>{10, 30, 20}));
}