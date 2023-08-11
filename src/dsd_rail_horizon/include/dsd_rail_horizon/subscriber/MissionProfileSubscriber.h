/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_MISSION_PROFILE_SUBSCRIBER_H
#define DSD_RAIL_HORIZON_MISSION_PROFILE_SUBSCRIBER_H

#include <dsd_rail_horizon_core/interfaces/ILogger.h>

#include <dsd_common_types/GeometryTypes.h>

#include <dsd_ros_messages/msg/mission_profile_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

/**
 * @brief Config parameters for the mission profile subscriber
 */
struct MissionProfileSubscriberConfig
{
    /**
     * @brief Name of the topic the subscriber is listening to
     */
    std::string topic_name;
    /**
     * @brief Quality of service setting for the subscriber
     */
    rclcpp::QoS qos;
};

/**
 * @brief Mission profile subscriber for receiving mission profile messages
 */
class MissionProfileSubscriber
{
public:
    /**
     * @brief Alias name for mission profile stamped message type
     */
    using MissionProfileStampedMessage = dsd_ros_messages::msg::MissionProfileStamped;

    /**
     * @brief Constructor for mission profile subscriber
     *
     * @param parent_node The parent node this subscriber is associated with. Needed to create the ros subscriber
     * @param config Config for the mission profile subscriber (see \ref MissionProfileSubscriberConfig)
     * @param logger Logger instance for logging messages
     */
    MissionProfileSubscriber(
        rclcpp::Node& parent_node, const MissionProfileSubscriberConfig& config, ILogger::SharedPtr logger);

    /**
     * @brief Get track ids from mission profile message
     *
     * @return std::vector<ID> Vector of track ids
     */
    std::vector<Id> get_track_ids() const;

private:
    /**
     * @brief Receive mission profile message
     *
     * @param mission_profile Received mission profile message
     */
    void mission_profile_callback(MissionProfileStampedMessage::SharedPtr mission_profile);

    /**
     * @brief Logger
     */
    ILogger::SharedPtr logger_;

    /**
     * @brief The ros subscriber for receiving the mission profile message
     */
    rclcpp::Subscription<MissionProfileStampedMessage>::SharedPtr mission_profile_subscriber_;

    /**
     * @brief Last received mission profile message.
     *
     * Read / Write Access should be protected by the mutex \ref mission_profile_mutex_
     */
    MissionProfileStampedMessage::SharedPtr mission_profile_;

    /**
     * @brief Mutex for protecting read / write to \ref mission_profile_ as the \ref mission_profile_callback is called
     * async from ROS2
     */
    mutable std::mutex mission_profile_mutex_;
};

#endif // DSD_RAIL_HORIZON_MISSION_PROFILE_SUBSCRIBER_H