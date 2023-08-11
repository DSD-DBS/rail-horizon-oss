/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_CUSTOM_PRINT_DEFINITIONS_H
#define DSD_RAIL_HORIZON_CUSTOM_PRINT_DEFINITIONS_H

#include <dsd_common_types/GeometryTypes.h>

#include <dsd_ros_messages/msg/application_status_stamped.hpp>
#include <dsd_ros_messages/msg/coupled_localization_stamped.hpp>
#include <dsd_ros_messages/msg/id.hpp>
#include <dsd_ros_messages/msg/key_value_pair.hpp>
#include <dsd_ros_messages/msg/mission_profile.hpp>
#include <dsd_ros_messages/msg/mission_profile_stamped.hpp>
#include <dsd_ros_messages/msg/object.hpp>
#include <dsd_ros_messages/msg/object_base.hpp>
#include <dsd_ros_messages/msg/object_classification.hpp>
#include <dsd_ros_messages/msg/object_tracking.hpp>
#include <dsd_ros_messages/msg/rail_horizon_data.hpp>
#include <dsd_ros_messages/msg/rail_horizon_stamped.hpp>
#include <dsd_ros_messages/msg/shape.hpp>

#include <boost/format.hpp>

#include <rclcpp/clock.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ostream>

namespace rclcpp
{
inline void PrintTo(const Time& time, std::ostream* os)
{
    *os << boost::format{"{seconds: %1%, nanoseconds: %2%}"} % time.seconds() % time.nanoseconds();
}
} // namespace rclcpp

namespace boost::geometry::model
{
inline void PrintTo(const PointXyz& point, std::ostream* os)
{
    *os << "(" << point.get<0>() << ", " << point.get<1>() << ", " << point.get<2>() << ")";
}
} // namespace boost::geometry::model

namespace dsd_ros_messages::msg
{

inline std::ostream& operator<<(std::ostream& os, const ID& id)
{
    return os << to_yaml(id);
}

inline std::ostream& operator<<(std::ostream& os, const MissionProfile& mission_profile)
{
    return os << to_yaml(mission_profile);
}

inline std::ostream& operator<<(std::ostream& os, const MissionProfileStamped& mission_profile_stamped)
{
    return os << to_yaml(mission_profile_stamped);
}

inline std::ostream& operator<<(std::ostream& os, const Shape& shape)
{
    return os << to_yaml(shape);
}

inline std::ostream& operator<<(std::ostream& os, const ObjectBase& object_base)
{
    return os << to_yaml(object_base);
}

inline std::ostream& operator<<(std::ostream& os, const ObjectClassification& object_classification)
{
    return os << to_yaml(object_classification);
}

inline std::ostream& operator<<(std::ostream& os, const ObjectTracking& object_tracking)
{
    return os << to_yaml(object_tracking);
}

inline std::ostream& operator<<(std::ostream& os, const KeyValuePair& key_value_pair)
{
    return os << to_yaml(key_value_pair);
}

inline std::ostream& operator<<(std::ostream& os, const KeyValueMap& key_value_map)
{
    return os << to_yaml(key_value_map);
}


inline std::ostream& operator<<(std::ostream& os, const Object& object)
{
    return os << to_yaml(object);
}

inline std::ostream& operator<<(std::ostream& os, const RailHorizonData& rail_horizon_data)
{
    return os << to_yaml(rail_horizon_data);
}

inline std::ostream& operator<<(std::ostream& os, const ApplicationStatusStamped& application_status_stamped)
{
    return os << to_yaml(application_status_stamped);
}

inline std::ostream& operator<<(
    std::ostream& os, const CoupledLocalizationStamped& coupled_localization_stamped_message)
{
    return os << to_yaml(coupled_localization_stamped_message);
}

inline std::ostream& operator<<(std::ostream& os, const RailHorizonStamped& rail_horizon_stamped_message)
{
    return os << to_yaml(rail_horizon_stamped_message);
}

} // namespace dsd_ros_messages::msg

namespace visualization_msgs::msg
{
inline std::ostream& operator<<(std::ostream& os, const Marker& marker_message)
{
    return os << to_yaml(marker_message);
}
inline std::ostream& operator<<(std::ostream& os, const MarkerArray& marker_array_message)
{
    return os << to_yaml(marker_array_message);
}
} // namespace visualization_msgs::msg

#endif // DSD_RAIL_HORIZON_CUSTOM_PRINT_DEFINITIONS_H