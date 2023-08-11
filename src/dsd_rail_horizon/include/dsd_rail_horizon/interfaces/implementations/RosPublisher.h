/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_ROS_PUBLISHER_H
#define DSD_RAIL_HORIZON_ROS_PUBLISHER_H

#include <dsd_rail_horizon/interfaces/IPublisher.h>

#include <rclcpp/rclcpp.hpp>

/**
 * @brief ROS Publisher class
 */
template <typename MessageType>
class RosPublisher : public IPublisher<MessageType>
{
public:
    /**
     * @brief ROS Publisher constructor
     * @param parent_node The parent node this publisher is associated with.
     * @param topic_name The topic on which this publisher is publishing a message
     * @param qos Quality of service setting for the publisher
     */
    RosPublisher(rclcpp::Node& parent_node, const std::string& topic_name, const rclcpp::QoS& qos)
    : publisher_{parent_node.create_publisher<MessageType>(topic_name, qos)}
    {
    }
    /**
     * @brief Function to publish specified message type
     * @param message Message of certain MessageType (see template)
     */
    void publish(const MessageType& message) const override
    {
        publisher_->publish(message);
    }

private:
    /**
     * @brief The ROS2 publisher used for publishing certain message
     */
    typename rclcpp::Publisher<MessageType>::SharedPtr publisher_;
};

#endif // DSD_RAIL_HORIZON_ROS_PUBLISHER_H