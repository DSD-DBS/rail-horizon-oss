/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_PUBLISHER_MOCK_H
#define DSD_RAIL_HORIZON_PUBLISHER_MOCK_H

#include <dsd_rail_horizon/interfaces/IPublisher.h>

#include <gmock/gmock.h>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>


template <typename MessageType>
class PublisherMock : public IPublisher<MessageType>
{
public:
    MOCK_METHOD(void, publish, ( const MessageType& ), (const, override));
};

#endif // DSD_RAIL_HORIZON_PUBLISHER_MOCK_H
