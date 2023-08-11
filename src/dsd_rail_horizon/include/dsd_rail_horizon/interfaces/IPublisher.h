/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_IPUBLISHER_H
#define DSD_RAIL_HORIZON_IPUBLISHER_H

#include <memory>

/**
 * @brief Publisher interface containing publish function that should be redefined in the derived classes
 */
template <typename MessageType>
class IPublisher
{
public:
    /**
     * @brief Function to publish specified message type
     * @param message Message of certain MessageType (see template)
     */
    virtual void publish(const MessageType& message) const = 0;

    /**
     * @brief Type alias for shared ptr on a mutable object of this class
     */
    using SharedPtr = std::shared_ptr<IPublisher<MessageType>>;
    /**
     * @brief Type alias for shared ptr on a const object of this class
     */
    using ConstSharedPtr = std::shared_ptr<const IPublisher<MessageType>>;
};

#endif // DSD_RAIL_HORIZON_IPUBLISHER_H