/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_ROS_CLOCK
#define DSD_ROS_CLOCK

#include <dsd_rail_horizon/interfaces/IClock.h>

#include <rclcpp/clock.hpp>

/**
 * @brief ROS Clock class
 */
class RosClock : public IClock
{
public:
    /**
     * @brief ROS Clock constructor
     * @param clock Shared pointer to the ROS2 Clock
     */
    RosClock(rclcpp::Clock::SharedPtr clock) : clock_{clock} {}
    /**
     * @brief Function to get the current time
     */
    [[nodiscard]] rclcpp::Time now() const override
    {
        return clock_->now();
    }

private:
    /**
     * @brief Shared Pointer of the rclcpp Clock
     */
    rclcpp::Clock::SharedPtr clock_;
};

#endif // DSD_ROS_CLOCK