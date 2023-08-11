/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_ROS_LOGGER_H
#define DSD_RAIL_HORIZON_ROS_LOGGER_H

#include <dsd_rail_horizon_core/interfaces/ILogger.h>

#include <boost/format.hpp>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <string>

/**
 * @brief ROS Logger class
 */
class RosLogger : public ILogger
{
public:
    /**
     * @brief ROS Logger constructor without a tag
     * @param logger ROS2 Logger
     */
    RosLogger(const rclcpp::Logger& logger) : logger_{logger}, tag_{""} {}
    /**
     * @brief ROS Logger constructor with a tag
     * @param logger ROS2 Logger
     * @param tag constant reference to std::string
     */
    RosLogger(const rclcpp::Logger& logger, const std::string& tag)
    : logger_{logger}, tag_{str(boost::format("[%1%]") % tag)}
    {
    }
    /**
     * @brief Function to show the debug message of type std::string
     * @param message constant reference to std::string
     */
    void debug(const std::string& message) override
    {
        RCLCPP_DEBUG(logger_, "%s", tagged_message(message).c_str());
    }
    /**
     * @brief Function to show the debug message of type boost::format
     * @param message constant reference to boost::format
     */
    void debug(const boost::format& message) override
    {
        debug(message.str());
    }
    /**
     * @brief Function to show the info message of type std::string
     * @param message constant reference to std::string
     */
    void info(const std::string& message) override
    {
        RCLCPP_INFO(logger_, "%s", tagged_message(message).c_str());
    }
    /**
     * @brief Function to show the info message of type boost::format
     * @param message constant reference to boost::format
     */
    void info(const boost::format& message) override
    {
        info(message.str());
    }
    /**
     * @brief Function to show the error message of type std::string
     * @param message constant reference to std::string
     */
    void error(const std::string& message) override
    {
        RCLCPP_ERROR(logger_, "%s", tagged_message(message).c_str());
    }
    /**
     * @brief Function to show the error message of type boost::format
     * @param message constant reference to boost::format
     */
    void error(const boost::format& message) override
    {
        error(message.str());
    }


private:
    /**
     * @brief Function that adds a tag infront of the message
     * @param tag constant reference to std::string
     */
    std::string tagged_message(const std::string& message)
    {
        if (tag_.empty())
        {
            return message;
        }
        return (tag_ + " " + message);
    }
    /**
     * @brief Logger instance of rclcpp
     */
    rclcpp::Logger logger_;
    /**
     * @brief Tag of type std::string
     */
    std::string tag_;
};

#endif // DSD_RAIL_HORIZON_ROS_LOGGER_H