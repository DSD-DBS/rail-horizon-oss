/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_ICLOCK_H
#define DSD_RAIL_HORIZON_ICLOCK_H

#include <rclcpp/time.hpp>

/**
 * @brief Clock interface containing current time function that should be redefined in the derived classes
 */
class IClock
{
public:
    /**
     * @brief Destructor of the Clock
     */
    virtual ~IClock() = default;

    /**
     * @brief Function to get the current time
     */
    [[nodiscard]] virtual rclcpp::Time now() const = 0;

    /**
     * @brief Type alias for shared ptr on a mutable object of this class
     */
    using SharedPtr = std::shared_ptr<IClock>;
    /**
     * @brief Type alias for shared ptr on a const object of this class
     */
    using ConstSharedPtr = std::shared_ptr<const IClock>;
};

#endif // DSD_RAIL_HORIZON_ICLOCK_H