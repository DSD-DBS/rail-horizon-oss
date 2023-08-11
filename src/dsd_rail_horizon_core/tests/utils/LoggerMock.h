/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_CORE_LOGGER_MOCK_H
#define DSD_RAIL_HORIZON_CORE_LOGGER_MOCK_H

#include <dsd_rail_horizon_core/interfaces/ILogger.h>

#include <gmock/gmock.h>

class LoggerMock : public ILogger
{
public:
    MOCK_METHOD(void, debug, (const std::string& message), (override));
    MOCK_METHOD(void, debug, (const boost::format& message), (override));
    MOCK_METHOD(void, info, (const std::string& message), (override));
    MOCK_METHOD(void, info, (const boost::format& message), (override));
    MOCK_METHOD(void, error, (const std::string& message), (override));
    MOCK_METHOD(void, error, (const boost::format& message), (override));
};


#endif // DSD_RAIL_HORIZON_CORE_LOGGER_MOCK_H