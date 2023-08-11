/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "utils/CustomPrintDefinitions.h" // IWYU pragma: keep // Enables more helpful output for custom types
#include "utils/mocks/ClockMock.h"
#include "utils/mocks/LoggerMock.h"
#include "utils/mocks/PublisherMock.h"

#include <dsd_rail_horizon/interfaces/IClock.h>
#include <dsd_rail_horizon/interfaces/IPublisher.h>
#include <dsd_rail_horizon/publisher/AppStatusPublisher.h>
#include <dsd_rail_horizon/utils/MessageHelper.h>

#include <dsd_rail_horizon_core/interfaces/ILogger.h>

#include <dsd_ros_messages/msg/application_configuration.hpp>
#include <dsd_ros_messages/msg/application_configuration_stamped.hpp>
#include <dsd_ros_messages/msg/application_status_stamped.hpp>
#include <dsd_ros_messages/msg/application_status_types.hpp>
#include <dsd_ros_messages/msg/detail/application_configuration_stamped__struct.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <string>

using ::testing::Eq;
using ::testing::Ne;

class AppStatusPublisherExposed : public AppStatusPublisher
{
public:
    using AppStatusPublisher::AppStatusPublisher;

    using AppStatusPublisher::app_configuration_;
    using AppStatusPublisher::app_configuration_callback;
    using AppStatusPublisher::app_status_periodic_broadcast_callback;

    using AppStatusPublisher::app_status_publisher_;
    using AppStatusPublisher::current_app_status_;
    using AppStatusPublisher::error_status_;
    using AppStatusPublisher::running_status_;

    using AppStatusPublisher::AppModeTypes;
    using AppStatusPublisher::AppStatusTypes;
};

class AppStatusPublisherTest : public ::testing::Test
{
public:
    AppStatusPublisherTest()
    {
        rclcpp::init(0, nullptr);
        parent_node_ = std::make_shared<rclcpp::Node>("test_node");

        app_status_publisher_ = std::make_shared<AppStatusPublisherExposed>(*parent_node_, config_, logger_, clock_);
        publisher_mock_ = std::make_shared<PublisherMock<dsd_ros_messages::msg::ApplicationStatusStamped>>();
        app_status_publisher_->app_status_publisher_ = publisher_mock_;
    }

protected:
    std::shared_ptr<rclcpp::Node> parent_node_;

    std::shared_ptr<LoggerMock> logger_ = std::make_shared<LoggerMock>();
    std::shared_ptr<ClockMock> clock_ = std::make_shared<ClockMock>();

    AppStatusPublisherConfig config_{"test_publisher_topic", "test_subscription_topic", 3, "AppId", "MissionId"};

    std::shared_ptr<AppStatusPublisherExposed> app_status_publisher_;
    std::shared_ptr<PublisherMock<dsd_ros_messages::msg::ApplicationStatusStamped>> publisher_mock_;
};

/****** app_configuration_callback ******/

TEST_F(AppStatusPublisherTest, checkApplicationConfigurationSetInCallback)
{
    // Arrange
    auto message = std::make_shared<dsd_ros_messages::msg::ApplicationConfigurationStamped>();

    // Act
    app_status_publisher_->app_configuration_callback(message);

    // Assert
    EXPECT_THAT(app_status_publisher_->app_configuration_, Ne(nullptr));
    EXPECT_THAT(*app_status_publisher_->app_configuration_, Eq(*message));
}

/****** app_status_periodic_broadcast_callback ******/

TEST_F(AppStatusPublisherTest, checkStandbyMessagePublished)
{
    // Arrange
    auto message = create_app_status_stamped_message(
        rclcpp::Time(), create_app_status_message({config_.mission_control_node_id}, {config_.mission_control_app_id},
                            {dsd_ros_messages::msg::ApplicationStatusTypes::APPLICATION_STATUS_TYPE_STANDBY},
                            {dsd_ros_messages::msg::ApplicationStatusTypes::APPLICATION_STATUS_SUB_TYPE_UNKNOWN}, {}));

    EXPECT_CALL(*publisher_mock_, publish(message));

    // Act
    app_status_publisher_->app_status_periodic_broadcast_callback();
}

TEST_F(AppStatusPublisherTest, checkErrorMessagePublished)
{
    // Arrange
    app_status_publisher_->current_app_status_ =
        AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_ERROR;
    app_status_publisher_->error_status_ = "Some error message";

    auto message = create_app_status_stamped_message(
        rclcpp::Time(), create_app_status_message({config_.mission_control_node_id}, {config_.mission_control_app_id},
                            {dsd_ros_messages::msg::ApplicationStatusTypes::APPLICATION_STATUS_TYPE_ERROR},
                            {dsd_ros_messages::msg::ApplicationStatusTypes::APPLICATION_STATUS_SUB_TYPE_UNKNOWN},
                            {app_status_publisher_->error_status_}));

    EXPECT_CALL(*publisher_mock_, publish(message));

    // Act
    app_status_publisher_->app_status_periodic_broadcast_callback();
}

TEST_F(AppStatusPublisherTest, checkRunningMessagePublished)
{
    // Arrange
    app_status_publisher_->current_app_status_ =
        AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_RUNNING;

    auto message = create_app_status_stamped_message(
        rclcpp::Time(), create_app_status_message({config_.mission_control_node_id}, {config_.mission_control_app_id},
                            {dsd_ros_messages::msg::ApplicationStatusTypes::APPLICATION_STATUS_TYPE_RUNNING},
                            {dsd_ros_messages::msg::ApplicationStatusTypes::APPLICATION_STATUS_SUB_TYPE_UNKNOWN}, {}));

    EXPECT_CALL(*publisher_mock_, publish(message));

    // Act
    app_status_publisher_->app_status_periodic_broadcast_callback();
}

/****** app_status_periodic_broadcast_callback : configuration processing ******/

TEST_F(AppStatusPublisherTest, appConfigurationRecievedWithoutRailHorizonUpdate)
{
    // Arrange
    app_status_publisher_->current_app_status_ =
        AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_RUNNING;
    app_status_publisher_->app_configuration_ =
        std::make_shared<dsd_ros_messages::msg::ApplicationConfigurationStamped>();

    // Act
    app_status_publisher_->app_status_periodic_broadcast_callback();

    // Assert
    // TO DO: Could be random that app status didn't change
    EXPECT_THAT(app_status_publisher_->current_app_status_,
        Eq(AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_RUNNING));
}

TEST_F(AppStatusPublisherTest, appConfigurationRecievedWithRailHorzionInStandby)
{
    // Arrange
    app_status_publisher_->current_app_status_ =
        AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_RUNNING;
    app_status_publisher_->app_configuration_ =
        std::make_shared<dsd_ros_messages::msg::ApplicationConfigurationStamped>();
    app_status_publisher_->app_configuration_->application_configuration.application_id = {
        "Other Id", config_.mission_control_app_id, "Other Id2"};
    app_status_publisher_->app_configuration_->application_configuration.mode = {
        AppStatusPublisherExposed::AppModeTypes::APPLICATION_MODE_TYPE_RUNNING,
        AppStatusPublisherExposed::AppModeTypes::APPLICATION_MODE_TYPE_STANDBY,
        AppStatusPublisherExposed::AppModeTypes::APPLICATION_MODE_TYPE_RUNNING};

    // Act
    app_status_publisher_->app_status_periodic_broadcast_callback();

    // Assert
    EXPECT_THAT(app_status_publisher_->current_app_status_,
        Eq(AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_STANDBY));
}

TEST_F(AppStatusPublisherTest, appConfigurationRecievedWithRailHorzionInRunning)
{
    // Arrange
    app_status_publisher_->current_app_status_ =
        AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_STANDBY;
    app_status_publisher_->app_configuration_ =
        std::make_shared<dsd_ros_messages::msg::ApplicationConfigurationStamped>();
    app_status_publisher_->app_configuration_->application_configuration.application_id = {
        "Other Id", config_.mission_control_app_id, "Other Id2"};
    app_status_publisher_->app_configuration_->application_configuration.mode = {
        AppStatusPublisherExposed::AppModeTypes::APPLICATION_MODE_TYPE_STANDBY,
        AppStatusPublisherExposed::AppModeTypes::APPLICATION_MODE_TYPE_RUNNING,
        AppStatusPublisherExposed::AppModeTypes::APPLICATION_MODE_TYPE_STANDBY};

    // Act
    app_status_publisher_->app_status_periodic_broadcast_callback();

    // Assert
    EXPECT_THAT(app_status_publisher_->current_app_status_,
        Eq(AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_RUNNING));
}


/****** isAppRunning ******/

TEST_F(AppStatusPublisherTest, appRunningIfStatusIsRunning)
{
    // Arrange
    app_status_publisher_->current_app_status_ =
        AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_RUNNING;

    // Act
    bool is_app_running = app_status_publisher_->is_app_running();

    EXPECT_THAT(is_app_running, Eq(true));
}

TEST_F(AppStatusPublisherTest, appNotRunningIfStatusIsNotRunning)
{
    // Arrange
    app_status_publisher_->current_app_status_ =
        AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_ERROR;

    // Act
    bool is_app_running = app_status_publisher_->is_app_running();

    // Assert
    EXPECT_THAT(is_app_running, Eq(false));
}

TEST_F(AppStatusPublisherTest, setRunningStatusInRunningMode)
{
    // Arrange
    app_status_publisher_->current_app_status_ =
        AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_RUNNING;
    std::string running_status_msg = "Some running status message";

    // Act
    app_status_publisher_->set_running_status(running_status_msg);

    // Assert
    EXPECT_THAT(app_status_publisher_->running_status_, Eq(running_status_msg));
    EXPECT_THAT(app_status_publisher_->current_app_status_,
        Eq(AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_RUNNING));
}

/****** setErrorStatus ******/
TEST_F(AppStatusPublisherTest, setErrorInStandbyMode)
{
    // Arrange
    app_status_publisher_->current_app_status_ =
        AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_STANDBY;
    std::string error_message = "Some error ";

    // Act
    app_status_publisher_->set_error_status(error_message);

    // Assert
    EXPECT_THAT(app_status_publisher_->error_status_, Eq(error_message));
    EXPECT_THAT(app_status_publisher_->current_app_status_,
        Eq(AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_STANDBY));
}

TEST_F(AppStatusPublisherTest, setErrorInRunningMode)
{
    // Arrange
    app_status_publisher_->current_app_status_ =
        AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_RUNNING;
    std::string error_message = "Some error";

    // Act
    app_status_publisher_->set_error_status(error_message);

    // Assert
    EXPECT_THAT(app_status_publisher_->error_status_, Eq(error_message));
    EXPECT_THAT(app_status_publisher_->current_app_status_,
        Eq(AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_ERROR));
}

TEST_F(AppStatusPublisherTest, setErrorInErrorMode)
{
    // Arrange
    app_status_publisher_->current_app_status_ =
        AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_ERROR;
    std::string error_message = "Some error";

    // Act
    app_status_publisher_->set_error_status(error_message);

    // Assert
    EXPECT_THAT(app_status_publisher_->error_status_, Eq(error_message));
    EXPECT_THAT(app_status_publisher_->current_app_status_,
        Eq(AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_ERROR));
}

TEST_F(AppStatusPublisherTest, setEmptyErrorInErrorMode)
{
    // Arrange
    app_status_publisher_->current_app_status_ =
        AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_ERROR;
    std::string error_message{};

    // Act
    app_status_publisher_->set_error_status(error_message);

    // Assert
    EXPECT_THAT(app_status_publisher_->error_status_, Eq(error_message));
    EXPECT_THAT(app_status_publisher_->current_app_status_,
        Eq(AppStatusPublisherExposed::AppStatusTypes::APPLICATION_STATUS_TYPE_RUNNING));
}
