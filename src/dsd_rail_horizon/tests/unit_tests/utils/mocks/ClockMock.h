/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_CLOCK_MOCK_H
#define DSD_RAIL_HORIZON_CLOCK_MOCK_H

#include <dsd_rail_horizon/interfaces/IClock.h>

#include <gmock/gmock.h>

class ClockMock : public IClock
{
public:
    MOCK_METHOD(rclcpp::Time, now, (), (const, override));
};


#endif // DSD_RAIL_HORIZON_CLOCK_MOCK_H