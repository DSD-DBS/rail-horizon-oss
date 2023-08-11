/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dsd_rail_horizon/publisher/AppStatusPublisher.h"

#include <dsd_rail_horizon/interfaces/IClock.h>
#include <dsd_rail_horizon/interfaces/implementations/RosPublisher.h>
#include <dsd_rail_horizon/interfaces/IPublisher.h>
#include <dsd_rail_horizon/utils/MessageHelper.h>

#include <dsd_ros_messages/msg/application_configuration.hpp>
#include <dsd_ros_messages/msg/application_status.hpp>
#include <dsd_ros_messages/msg/application_status_types.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <memory>

using namespace std::chrono_literals;

AppStatusPublisher::AppStatusPublisher(rclcpp::Node& parent_node, const AppStatusPublisherConfig& config,
    ILogger::SharedPtr logger, IClock::ConstSharedPtr clock)
: config_{config}, logger_{logger}, clock_{clock}
{
    app_status_publisher_ =
        std::make_shared<RosPublisher<AppStatusMessage>>(parent_node, config.publisher_topic_name, config.qos);

    auto publisher_callback = std::bind(&AppStatusPublisher::app_status_periodic_broadcast_callback, this);
    timer_app_status_callback_ = parent_node.create_wall_timer(1s, publisher_callback);

    auto subscriber_callback = std::bind(&AppStatusPublisher::app_configuration_callback, this, std::placeholders::_1);
    app_configuration_subscriber_ = parent_node.create_subscription<AppConfigurationMessage>(
        config.subscriber_topic_name, config.qos, subscriber_callback);
}

void AppStatusPublisher::set_error_status(const std::string& error_status)
{
    this->error_status_ = error_status;

    if (current_app_status_ != AppStatusTypes::APPLICATION_STATUS_TYPE_STANDBY)
    {
        if (error_status_.empty())
        {
            current_app_status_ = AppStatusTypes::APPLICATION_STATUS_TYPE_RUNNING;
        }
        else
        {
            current_app_status_ = AppStatusTypes::APPLICATION_STATUS_TYPE_ERROR;
        }
    }
}

void AppStatusPublisher::set_running_status(const std::string& running_status)
{
    this->running_status_ = running_status;
}
bool AppStatusPublisher::is_app_running() const
{
    return current_app_status_ == AppStatusTypes::APPLICATION_STATUS_TYPE_RUNNING;
}

void AppStatusPublisher::app_status_periodic_broadcast_callback()
{
    app_status_publisher_->publish(create_message());


    std::lock_guard<std::mutex> lock(app_config_mutex_);

    if (!app_configuration_)
    {
        logger_->info("Received no application configuration update ...");
        return;
    }

    std::vector<std::string> app_config_application_id = app_configuration_->application_configuration.application_id;

    // Check if rail_horizon is listed in the message
    auto it =
        std::find(app_config_application_id.begin(), app_config_application_id.end(), config_.mission_control_app_id);
    if (it == app_config_application_id.end())
    {
        logger_->info("No Rail Horizon status update ...");
        return;
    }

    // TODO(leon): Should status be reset every time?
    size_t index = std::distance(app_config_application_id.begin(), it);
    switch (app_configuration_->application_configuration.mode[index])
    {
        case AppModeTypes::APPLICATION_MODE_TYPE_STANDBY:
            current_app_status_ = AppStatusTypes::APPLICATION_STATUS_TYPE_STANDBY;
            break;
        case AppModeTypes::APPLICATION_MODE_TYPE_RUNNING:
            current_app_status_ = AppStatusTypes::APPLICATION_STATUS_TYPE_RUNNING;
            break;
    }
}

AppStatusPublisher::AppStatusMessage AppStatusPublisher::create_message()
{
    std::vector<std::string> comment = {};
    // if (current_app_status_ == AppStatusTypes::APPLICATION_STATUS_TYPE_ERROR)
    // {
    //     comment.push_back(error_status_);
    // }

    std::vector<uint8_t> app_status_type = {};
    switch (current_app_status_)
    {
        case AppStatusTypes::APPLICATION_STATUS_TYPE_STANDBY:
            app_status_type.push_back(dsd_ros_messages::msg::ApplicationStatusTypes::APPLICATION_STATUS_TYPE_STANDBY);
            break;
        case AppStatusTypes::APPLICATION_STATUS_TYPE_RUNNING:
            app_status_type.push_back(dsd_ros_messages::msg::ApplicationStatusTypes::APPLICATION_STATUS_TYPE_RUNNING);
            if (!running_status_.empty())
            {
                comment.push_back(running_status_);
            }
            break;
        case AppStatusTypes::APPLICATION_STATUS_TYPE_ERROR:
            app_status_type.push_back(dsd_ros_messages::msg::ApplicationStatusTypes::APPLICATION_STATUS_TYPE_ERROR);
            comment.push_back(error_status_);
            break;
    }

    auto app_status_message =
        create_app_status_message({config_.mission_control_node_id}, {config_.mission_control_app_id}, app_status_type,
            {dsd_ros_messages::msg::ApplicationStatusTypes::APPLICATION_STATUS_SUB_TYPE_UNKNOWN}, comment);

    return create_app_status_stamped_message(clock_->now(), app_status_message);
}

void AppStatusPublisher::app_configuration_callback(const AppConfigurationMessage::SharedPtr application_configuration)
{
    logger_->info("Application configuration update.");

    std::lock_guard<std::mutex> lock(app_config_mutex_);
    app_configuration_ = application_configuration;
}