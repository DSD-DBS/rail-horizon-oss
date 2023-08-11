/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_APP_STATUS_PUBLISHER_H
#define DSD_RAIL_HORIZON_APP_STATUS_PUBLISHER_H

#include <dsd_rail_horizon/interfaces/IClock.h>
#include <dsd_rail_horizon/interfaces/IPublisher.h>

#include <dsd_rail_horizon_core/interfaces/ILogger.h>

#include <dsd_ros_messages/msg/application_configuration_stamped.hpp>
#include <dsd_ros_messages/msg/application_status_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

/**
 * @brief Config parameters for the application status publisher
 */
struct AppStatusPublisherConfig
{
    /**
     * @brief Name of the topic the publisher is publishing to
     */
    std::string publisher_topic_name;
    /**
     * @brief Name of the topic the subscriber is listening to
     */
    std::string subscriber_topic_name;
    /**
     * @brief Quality of service setting for the publisher and subscriber
     */
    rclcpp::QoS qos;
    /**
     * @brief Application identifier
     */
    std::string mission_control_app_id;
    /**
     * @brief Node identifier
     */
    std::string mission_control_node_id;
};

/**
 * @brief Application status publisher for publishing application status messages
 */
class AppStatusPublisher
{
public:
    /**
     * @brief Constructor for application status publisher
     * @param parent_node The parent node this publisher is associated with.
     * @param config Config for the application status pubisher (see \ref AppStatusPublisherConfig)
     * @param logger Logger instance for logging messages
     * @param clock Clock instance to get the current time for spoofing
     */
    AppStatusPublisher(rclcpp::Node& parent_node, const AppStatusPublisherConfig& config, ILogger::SharedPtr logger,
        IClock::ConstSharedPtr clock);

    /**
     * @brief Sets the error status and current app mode
     * @param error_status Error status text
     */
    void set_error_status(const std::string& error_status);

    /**
     * @brief Sets the running status needed as a comment
     * @param running_status Running status text
     */
    void set_running_status(const std::string& running_status);

    /**
     * @brief Checks if the publisher is currently in APPLICATION_MODE_TYPE_RUNNING
     */
    bool is_app_running() const;

protected:
    /**
     * @brief Alias name for application status message type
     */
    using AppStatusMessage = dsd_ros_messages::msg::ApplicationStatusStamped;
    /**
     * @brief Alias name for application configuration message type
     */
    using AppConfigurationMessage = dsd_ros_messages::msg::ApplicationConfigurationStamped;

    /**
     * @brief Periodically broadcasts app status and updates app status
     */
    void app_status_periodic_broadcast_callback();

    /**
     * @brief Reciever for app configuration message updates
     * @param application_configuration shared pointer to AppConfigurationMessage
     */
    void app_configuration_callback(AppConfigurationMessage::SharedPtr application_configuration);

    /**
     * @brief Constructs ApplicationStatusStamped Message for periodic broadcast
     */
    AppStatusMessage create_message();
    /**
     * @brief Enumeration AppModeTypes that contains potential application modes
     */
    enum AppModeTypes
    {
        APPLICATION_MODE_TYPE_STANDBY = 0,
        APPLICATION_MODE_TYPE_RUNNING,
        APPLICATION_MODE_TYPE_PREPARE_POWER_OFF,
    };
    /**
     * @brief Enumeration AppStatusTypes that contains potential application statuses
     */
    enum class AppStatusTypes
    {
        APPLICATION_STATUS_TYPE_STANDBY = 0,
        // APPLICATION_STATUS_TYPE_BUSY,
        APPLICATION_STATUS_TYPE_RUNNING,
        APPLICATION_STATUS_TYPE_ERROR,
        // APPLICATION_STATUS_TYPE_REQUIRES_RESTART,
    };

    /**
     * @brief Config of the publisher
     */
    AppStatusPublisherConfig config_;

    /**
     * @brief Logger
     */
    ILogger::SharedPtr logger_;
    /**
     * @brief Clock for getting the current time for spoofing
     */
    IClock::ConstSharedPtr clock_;
    /**
     * @brief Current application status (initially set on APPLICATION_STATUS_TYPE_STANDBY)
     */
    AppStatusTypes current_app_status_{AppStatusTypes::APPLICATION_STATUS_TYPE_STANDBY};
    /**
     * @brief Error status with description of the error state
     */
    std::string error_status_;
    /**
     * @brief Running status with description of the running state
     */
    std::string running_status_;

    /**
     * @brief The ros publisher for publishing the application status message
     */
    IPublisher<AppStatusMessage>::SharedPtr app_status_publisher_;
    /**
     * @brief Timer periodically calls app_status_periodic_broadcast_callback
     *
     */
    rclcpp::TimerBase::SharedPtr timer_app_status_callback_;

    /**
     * @brief The ros subscriber for receiving the application configuration message
     */
    rclcpp::Subscription<AppConfigurationMessage>::SharedPtr app_configuration_subscriber_;
    /**
     * @brief Last received application configuration message.
     * Read / Write Access should be protected by the mutex \ref app_config_mutex_
     */
    AppConfigurationMessage::SharedPtr app_configuration_;
    /**
     * @brief Mutex for protecting read / write to \ref app_configuration_ as the \ref app_configuration_callback is
     * called async from ROS2
     */
    mutable std::mutex app_config_mutex_;
};

#endif // DSD_RAIL_HORIZON_APP_STATUS_PUBLISHER_H
