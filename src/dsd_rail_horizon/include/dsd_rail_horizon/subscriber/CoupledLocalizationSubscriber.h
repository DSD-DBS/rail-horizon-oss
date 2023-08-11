/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_COUPLED_LOCALIZATION_SUBSCRIBER_H
#define DSD_RAIL_HORIZON_COUPLED_LOCALIZATION_SUBSCRIBER_H

#include <dsd_rail_horizon/interfaces/IClock.h>

#include <dsd_rail_horizon_core/interfaces/ILogger.h>

#include <dsd_common_types/GeoGeometryTypes.h>
#include <dsd_common_types/GeometryTypes.h>

#include <dsd_ros_messages/msg/coupled_localization.hpp>
#include <dsd_ros_messages/msg/coupled_localization_stamped.hpp>
#include <dsd_ros_messages/msg/gnss_localization.hpp>
#include <dsd_ros_messages/msg/orientation.hpp>
#include <dsd_ros_messages/msg/position.hpp>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

/**
 * @brief Config parameters for the coupled localization subscriber
 */
struct CoupledLocalizationSubscriberConfig
{
    /**
     * @brief Name of the topic the subscriber is listening to
     */
    std::string topic_name;
    /**
     * @brief Quality of service setting for the subscriber
     */
    rclcpp::QoS qos;
    /**
     * @brief Whether the gnss position and rotation should be spoofed
     */
    bool spoof_gnss_position;
    /**
     * @brief Replacement position for gnss if spoof_gnss_position is true
     */
    std::vector<double> gnss_position;
    /**
     * @brief Replacement rotation for gnss if spoof_gnss_position is true
     */
    std::vector<float> gnss_rotation;
    /**
     * @brief Whether latitute and longitute values of the gnss positions should be swapped (first & second value of
     * position)
     */
    bool gnss_swap_lat_lon;
};

/**
 * @brief Coupled localization subscriber for receiving coupled localization messages
 */
class CoupledLocalizationSubscriber
{
public:
    /**
     * @brief Alias name for couples localization message type
     */
    using CoupledLocalizationStampedMessage = dsd_ros_messages::msg::CoupledLocalizationStamped;

    /**
     * @brief Constructor for coupled localization subscriber
     *
     * @param parent_node The parent node this subscriber is associated with. Needed to create the ros subscriber
     * @param config Config for the coupled localization subscriber (see \ref CoupledLocalizationSubscriberConfig)
     * @param logger Logger instance for logging messages
     * @param clock Clock instance to get the current time for spoofing
     */
    CoupledLocalizationSubscriber(rclcpp::Node& parent_node, const CoupledLocalizationSubscriberConfig& config,
        ILogger::SharedPtr logger, IClock::SharedPtr clock);

    /**
     * @brief Checks whether the subscriber has received any valid data since startup
     *
     * @return bool Whether the subscriber has received any valid data since startup
     */
    bool is_data_available() const
    {
        return coupled_localization_ != nullptr;
    }

    /**
     * @brief Get the latest received message by the subscriber
     *
     * For performance reason this method is unsafe. The method \ref is_data_available can be used to check whether data
     * is available.
     *
     * @return CoupledLocalizationStampedMessage The received message
     */
    CoupledLocalizationStampedMessage get_message() const
    {
        return *coupled_localization_;
    }

private:
    /**
     * @brief Receive couple localisation message
     *
     * Depending on the config of the subscriber the message will also be altered (e.g. spoofing)
     *
     * @param coupled_localization Received coupled localization message
     */
    void coupled_localization_callback(CoupledLocalizationStampedMessage::SharedPtr coupled_localization);

    /**
     * @brief Config of the subscriber
     */
    CoupledLocalizationSubscriberConfig config_;

    /**
     * @brief Logger
     */
    ILogger::SharedPtr logger_;

    /**
     * @brief Clock for getting the current time for spoofing
     */
    IClock::SharedPtr clock_;

    /**
     * @brief The ros subscriber for receiving the coupled localization message
     */
    rclcpp::Subscription<CoupledLocalizationStampedMessage>::SharedPtr coupled_localization_subscriber_;

    /**
     * @brief Last received coupled localization message.
     *
     * Read / Write Access should be protected by the mutex \ref coupled_localization_mutex_
     */
    CoupledLocalizationStampedMessage::ConstSharedPtr coupled_localization_;

    /**
     * @brief Mutex for protecting read / write to \ref coupled_localization_ as the \ref coupled_localization_callback
     * is called async from ROS2
     */
    mutable std::mutex coupled_localization_mutex_;
};

/**
 * @brief Convenience method to get the current gnss orientation from the coupled localization message
 *
 * @param coupled_localization Coupled localization message
 * @return PointXyz Current GNSS Orientation
 */
inline PointXyz get_current_gnss_orientation(
    const dsd_ros_messages::msg::CoupledLocalizationStamped& coupled_localization)
{
    const auto orientation = coupled_localization.coupled_localization.coupled_gnss_localization.orientation.angles;
    return {orientation[0], orientation[1], orientation[2]};
}

/**
 * @brief  Convenience method to get the current gnss position from the coupled localization message
 *
 * @param coupled_localization Coupled localization message
 * @return PointXyzWgs84 Current GNSS Position in WSGS84 coordinates
 */
inline PointXyzWgs84 get_current_gnss_position(
    const dsd_ros_messages::msg::CoupledLocalizationStamped& coupled_localization)
{
    const auto position = coupled_localization.coupled_localization.coupled_gnss_localization.position.position;
    return {position[0], position[1], position[2]};
}

#endif // DSD_RAIL_HORIZON_COUPLED_LOCALIZATION_SUBSCRIBER_H