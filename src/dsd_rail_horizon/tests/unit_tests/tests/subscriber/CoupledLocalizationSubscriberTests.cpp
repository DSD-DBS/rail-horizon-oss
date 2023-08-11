/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "utils/CustomPrintDefinitions.h" // IWYU pragma: keep // Enables more helpful output for custom types
#include "utils/mocks/ClockMock.h"
#include "utils/mocks/LoggerMock.h"

#include <dsd_rail_horizon/subscriber/CoupledLocalizationSubscriber.h>

#include <dsd_ros_messages/msg/coupled_localization.hpp>
#include <dsd_ros_messages/msg/coupled_localization_stamped.hpp>
#include <dsd_ros_messages/msg/gnss_localization.hpp>
#include <dsd_ros_messages/msg/orientation.hpp>
#include <dsd_ros_messages/msg/position.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <boost/geometry/geometries/geometries.hpp>

#include <rclcpp/rclcpp.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/header.hpp>

#include <array>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

using ::testing::Eq;

class CoupledLocalizationSubscriberFixture : public ::testing::Test
{
public:
    using Publisher = rclcpp::Publisher<dsd_ros_messages::msg::CoupledLocalizationStamped>;
    using Subscriber = CoupledLocalizationSubscriber;
    using Config = CoupledLocalizationSubscriberConfig;

    CoupledLocalizationSubscriberFixture()
    {
        rclcpp::init(0, nullptr);
        parent_node_ = std::make_shared<rclcpp::Node>("test_node");
    }

    Config create_default_config()
    {
        return {"test", 1, false, {}, {}, false};
    }

    Config create_swap_and_spoof_config()
    {
        return {"test", 1, true, {1.0, 2.0, 3.0}, {0.1, 0.2, 0.3}, true};
    }

    std::tuple<Publisher::SharedPtr, std::shared_ptr<Subscriber>> create_pub_sub_with_config(const Config& config)
    {
        auto publisher = parent_node_->create_publisher<dsd_ros_messages::msg::CoupledLocalizationStamped>(
            config.topic_name, config.qos);
        auto subscriber = std::make_shared<CoupledLocalizationSubscriber>(*parent_node_, config, logger_, clock_);
        return {publisher, subscriber};
    }

    dsd_ros_messages::msg::CoupledLocalizationStamped create_coupled_localization_stamped_message()
    {
        dsd_ros_messages::msg::GNSSLocalization localization_message{};
        localization_message.position.position = {4.0, 5.0, 6.0};
        localization_message.orientation.angles = {0.0, 1.0, 2.0};

        dsd_ros_messages::msg::CoupledLocalizationStamped message{};
        message.header.stamp.sec = 123;
        message.header.stamp.nanosec = 456;
        message.coupled_localization.coupled_gnss_localization = localization_message;

        return message;
    }

    dsd_ros_messages::msg::CoupledLocalizationStamped create_malformed_coupled_localization_stamped_message()
    {
        auto message = create_coupled_localization_stamped_message();
        message.coupled_localization.coupled_gnss_localization = dsd_ros_messages::msg::GNSSLocalization{};
        return message;
    }

protected:
    std::shared_ptr<rclcpp::Node> parent_node_;

    std::shared_ptr<LoggerMock> logger_ = std::make_shared<LoggerMock>();
    std::shared_ptr<ClockMock> clock_ = std::make_shared<ClockMock>();
};

// Checks that no data is available at startup or when publisher has not sent anything
TEST_F(CoupledLocalizationSubscriberFixture, noDataAvailableAtStartup)
{
    // Arrange
    auto [_, subscriber] = create_pub_sub_with_config(create_default_config());

    // Assert
    EXPECT_THAT(subscriber->is_data_available(), Eq(false));
}

// Checks that data is received after publisher has published some data
TEST_F(CoupledLocalizationSubscriberFixture, dataAvailableAfterPublishedMessage)
{
    // Arrange
    auto [publisher, subscriber] = create_pub_sub_with_config(create_default_config());
    auto message = create_coupled_localization_stamped_message();

    // Act
    publisher->publish(message);
    rclcpp::spin_some(parent_node_);

    // Assert
    EXPECT_THAT(subscriber->is_data_available(), Eq(true));
    EXPECT_THAT(subscriber->get_message(), Eq(message));
}

// Checks that data is received and modified properly when spoofing and longitude / latitude swapping is enabled
TEST_F(CoupledLocalizationSubscriberFixture, modifiedDataAvailableAfterPublishedMessage)
{
    // Arrange
    auto config = create_swap_and_spoof_config();
    auto [publisher, subscriber] = create_pub_sub_with_config(config);
    auto message = create_coupled_localization_stamped_message();
    rclcpp::Time time{100};
    ON_CALL(*clock_, now()).WillByDefault(testing::Return(time));

    // Act
    publisher->publish(message);
    rclcpp::spin_some(parent_node_);

    // Assert
    message.header.stamp = time;
    // spoofed and swapped position
    message.coupled_localization.coupled_gnss_localization.position.position = {
        config.gnss_position[1], config.gnss_position[0], config.gnss_position[2]};
    message.coupled_localization.coupled_gnss_localization.orientation.angles =
        std::vector<float>(config.gnss_rotation.begin(), config.gnss_rotation.end());

    EXPECT_THAT(subscriber->is_data_available(), Eq(true));
    EXPECT_THAT(subscriber->get_message(), Eq(message));
}

// Checks that no data is available and error is logged when the received gnss position is malformed and spoofing is not
// enabled
TEST_F(CoupledLocalizationSubscriberFixture, noDataAvailableIfMessageMalformed)
{
    // Arrange
    auto [publisher, subscriber] = create_pub_sub_with_config(create_default_config());
    auto message = create_malformed_coupled_localization_stamped_message();

    // Expect
    EXPECT_CALL(*logger_, error("Received malformed position update."));

    // Act
    publisher->publish(message);
    rclcpp::spin_some(parent_node_);

    // Assert
    EXPECT_THAT(subscriber->is_data_available(), Eq(false));
}

// Checks that gnss orientation is extracted correctly from coupled localization message
TEST_F(CoupledLocalizationSubscriberFixture, getCurrentGnssOrientation)
{
    // Arrange
    auto message = create_coupled_localization_stamped_message();

    // Act
    auto orientation = get_current_gnss_orientation(message);

    // Assert
    // TODO(leon): No operator==? Perhaps we can add it
    // EXPECT_THAT(orientation, Eq(PointXyz{0.0, 1.0, 2.0}));
    EXPECT_THAT(orientation.get<0>(), Eq(0.0));
    EXPECT_THAT(orientation.get<1>(), Eq(1.0));
    EXPECT_THAT(orientation.get<2>(), Eq(2.0));
}

// Checks that gnss position is extracted correctly from coupled localization message
TEST_F(CoupledLocalizationSubscriberFixture, getCurrentGnssPosition)
{
    // Arrange
    auto message = create_coupled_localization_stamped_message();

    // Act
    auto position = get_current_gnss_position(message);

    // Assert
    // EXPECT_THAT(position, Eq(PointXyzWgs84{4.0, 5.0, 6.0}));
    EXPECT_THAT(position.get<0>(), Eq(4.0));
    EXPECT_THAT(position.get<1>(), Eq(5.0));
    EXPECT_THAT(position.get<2>(), Eq(6.0));
}