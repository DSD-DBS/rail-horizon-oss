/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dsd_rail_horizon/subscriber/CoupledLocalizationSubscriber.h"

#include <dsd_rail_horizon/interfaces/IClock.h>

#include <std_msgs/msg/header.hpp>

#include <array>
#include <functional>
#include <utility>

CoupledLocalizationSubscriber::CoupledLocalizationSubscriber(rclcpp::Node& parent_node,
    const CoupledLocalizationSubscriberConfig& config, ILogger::SharedPtr logger, IClock::SharedPtr clock)
: config_{config}, logger_{logger}, clock_{clock}
{
    auto subscriber_callback =
        std::bind(&CoupledLocalizationSubscriber::coupled_localization_callback, this, std::placeholders::_1);
    coupled_localization_subscriber_ = parent_node.create_subscription<CoupledLocalizationStampedMessage>(
        config.topic_name, config.qos, subscriber_callback);
}

void CoupledLocalizationSubscriber::coupled_localization_callback(
    const CoupledLocalizationStampedMessage::SharedPtr coupled_localization)
{
    // logger_->info("Received position update."); //commented due to overhead
    auto* coupled_localization_position =
        &(coupled_localization->coupled_localization.coupled_gnss_localization.position);
    auto* coupled_localization_orientation =
        &(coupled_localization->coupled_localization.coupled_gnss_localization.orientation);

    if (config_.spoof_gnss_position)
    {
        coupled_localization->header.stamp = clock_->now();
        coupled_localization_position->position = config_.gnss_position;
        coupled_localization_orientation->angles = config_.gnss_rotation;
        logger_->info("Spoofed GNSS position and timestamp.");
    }
    else if (coupled_localization_position->position.size() != 3)
    {
        logger_->error("Received malformed position update.");
        return;
    }

    if (config_.gnss_swap_lat_lon)
    {
        // Swap LAT / LON according to interface definition
        std::swap(coupled_localization_position->position[0], coupled_localization_position->position[1]);
        // logger_->info("Swapped LAT/LON."); //commented due to overhead
    }

    std::lock_guard<std::mutex> lock(coupled_localization_mutex_);
    coupled_localization_ = coupled_localization;
}