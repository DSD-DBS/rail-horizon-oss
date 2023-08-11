/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dsd_rail_horizon/subscriber/MissionProfileSubscriber.h"

#include <dsd_common_types/GeometryTypes.h>

#include <dsd_ros_messages/msg/mission_profile.hpp>

#include <algorithm>
#include <array>
#include <functional>
#include <iterator>

MissionProfileSubscriber::MissionProfileSubscriber(
    rclcpp::Node& parent_node, const MissionProfileSubscriberConfig& config, ILogger::SharedPtr logger)
: logger_{logger}
{
    auto subscriber_callback =
        std::bind(&MissionProfileSubscriber::mission_profile_callback, this, std::placeholders::_1);
    mission_profile_subscriber_ = parent_node.create_subscription<MissionProfileStampedMessage>(
        config.topic_name, config.qos, subscriber_callback);
}

std::vector<Id> MissionProfileSubscriber::get_track_ids() const
{
    std::lock_guard<std::mutex> lock(mission_profile_mutex_);
    if (!mission_profile_)
    {
        logger_->info("Received no mission profile update ...");
        return {};
    }

    std::vector<Id> track_ids{};
    std::transform(begin(mission_profile_->mission_profile.track_ids), end(mission_profile_->mission_profile.track_ids),
        std::back_inserter(track_ids), [&](const auto& track_id) {
            return track_id.id;
        });
    return track_ids;
}

void MissionProfileSubscriber::mission_profile_callback(const MissionProfileStampedMessage::SharedPtr mission_profile)
{
    logger_->info("Mission profile update.");

    std::lock_guard<std::mutex> lock(mission_profile_mutex_);
    mission_profile_ = mission_profile;
}