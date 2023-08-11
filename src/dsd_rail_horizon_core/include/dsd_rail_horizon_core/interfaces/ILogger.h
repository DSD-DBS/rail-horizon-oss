/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_CORE_ILOGGER_H
#define DSD_RAIL_HORIZON_CORE_ILOGGER_H

#include <boost/format/format_fwd.hpp>

#include <memory>
#include <string>

/**
 * @brief Logger interface containing logging functions that should be redefined in the derived classes
 */
class ILogger
{
public:
    /**
     * @brief Destructor of the Logger
     */
    virtual ~ILogger() = default;
    /**
     * @brief Function to show the debug message of type std::string
     * @param message constant reference to std::string
     */
    virtual void debug(const std::string& message) = 0;
    /**
     * @brief Function to show the debug message of type boost::format
     * @param message constant reference to boost::format
     */
    virtual void debug(const boost::format& message) = 0;
    /**
     * @brief Function to show the info message of type std::string
     * @param message constant reference to std::string
     */
    virtual void info(const std::string& message) = 0;
    /**
     * @brief Function to show the info message of type boost::format
     * @param message constant reference to boost::format
     */
    virtual void info(const boost::format& message) = 0;
    /**
     * @brief Function to show the error message of type std::string
     * @param message constant reference to std::string
     */
    virtual void error(const std::string& message) = 0;
    /**
     * @brief Function to show the error message of type boost::format
     * @param message constant reference to boost::format
     */
    virtual void error(const boost::format& message) = 0;

    /**
     * @brief Type alias for shared ptr on a mutable object of this class
     */
    using SharedPtr = std::shared_ptr<ILogger>;
    /**
     * @brief Type alias for shared ptr on a const object of this class
     */
    using ConstSharedPtr = std::shared_ptr<const ILogger>;
};

#endif // DSD_RAIL_HORIZON_CORE_ILOGGER_H
