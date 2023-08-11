/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_TEST_SUBSCRIBER_H
#define DSD_RAIL_HORIZON_TEST_SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>

#include <string>

template <typename Message>
class TestSubscriber
{
public:
    TestSubscriber(rclcpp::Node& parent_node, const std::string& topic_name, const rclcpp::QoS& qos)
    : subscriber_{parent_node.create_subscription<Message>(topic_name, qos, [this](typename Message::SharedPtr msg) {
          this->last_message = msg;
      })}
    {
    }

    typename Message::SharedPtr last_message = nullptr;

private:
    typename rclcpp::Subscription<Message>::SharedPtr subscriber_;
};


#endif // DSD_RAIL_HORIZON_TEST_SUBSCRIBER_H