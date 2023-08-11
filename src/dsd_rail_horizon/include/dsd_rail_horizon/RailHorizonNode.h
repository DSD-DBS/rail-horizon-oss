/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DSD_RAIL_HORIZON_RAIL_RAIL_HORIZON_NODE_H
#define DSD_RAIL_HORIZON_RAIL_RAIL_HORIZON_NODE_H

#include "dsd_rail_horizon/publisher/MarkerStructurePublisher.h"
#include "dsd_rail_horizon/publisher/RailHorizonPublisher.h"
#include "dsd_rail_horizon/utils/CalculationHelper.h"

#include <dsd_rail_horizon/interfaces/IClock.h>
#include <dsd_rail_horizon/publisher/AppStatusPublisher.h>
#include <dsd_rail_horizon/RailHorizonParameters.h>
#include <dsd_rail_horizon/subscriber/CoupledLocalizationSubscriber.h>
#include <dsd_rail_horizon/subscriber/MissionProfileSubscriber.h>

#include <dsd_rail_horizon_core/interfaces/ILogger.h>
#include <dsd_rail_horizon_core/LookupStructures.h>
#include <dsd_rail_horizon_core/Projection.h>

#include <dsd_common_types/GeometryTypes.h>

#include <dsd_ros_messages/msg/coupled_localization_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>


using namespace std::chrono_literals;
/**
 * @brief Rail Horizon ROS Node
 */
class RailHorizonNode : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Rail Horizon Publisher object
     *
     * @param options Optionally pass ROS2 options such as parameters
     */
    RailHorizonNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

protected:
    /**
     * @brief Creates all publisher and subscribers
     *
     */
    void setup_publishers_and_subscribers();

    /**
     * @brief Selects the map provider and reads data from it
     *
     */
    void init_lookup_structures();

    /**
     * @brief Publishes RailHorizon data and optionally marker data
     *
     */
    void rail_horizon_periodic_broadcast_callback();

    /**
     * @brief Extracts current gnss position from CoupledLocalizationStamped message
     */
    PointXyz extract_current_gnss_position(
        const dsd_ros_messages::msg::CoupledLocalizationStamped& coupled_localization_message);
    /**
     * @brief Creates rotation matrix by extracting the orientation from CoupledLocalizationStamped message
     */
    RotationMatrix extract_rotation_matrix(
        const dsd_ros_messages::msg::CoupledLocalizationStamped& coupled_localization_message);
    /**
     * @brief Get track ids from mission profile subscriber
     * @return std::vector<ID> Vector of track ids
     */
    std::vector<Id> get_track_ids();

    /**
     * @brief Rail Horizon parameters from configuration file
     */
    std::shared_ptr<RailHorizonParameters> parameters_;

    /**
     * @brief Logger
     */
    ILogger::SharedPtr logger_;
    /**
     * @brief Clock
     */
    IClock::SharedPtr clock_;
    /**
     * @brief Projection of the gnss localization
     */
    Projection::SharedPtr gnss_projection_;
    /**
     * @brief Projection of the map
     */
    Projection::SharedPtr map_projection_;

    /**
     * @brief Timer periodically calls rail_horizon_periodic_broadcast_callback
     */
    rclcpp::TimerBase::SharedPtr timer_rh_callback_;

    /**
     * @brief Rail Horizon publisher
     */
    std::shared_ptr<RailHorizonPublisher> rail_horizon_publisher_;
    /**
     * @brief Marker publisher (relevant for rviz)
     */
    std::shared_ptr<MarkerStructurePublisher> marker_publisher_;
    /**
     * @brief Manages state of app and publishes it
     */
    std::shared_ptr<AppStatusPublisher> app_status_publisher_;
    /**
     * @brief Coupled Localization Subscriber for receiving CoupledLocalizationStamped message
     */
    std::shared_ptr<CoupledLocalizationSubscriber> coupled_localization_subscriber_;
    /**
     * @brief Mission Profile Subscriber for receiving MissionProfileStamped message
     */
    std::shared_ptr<MissionProfileSubscriber> mission_profile_subscriber_;

    /**
     * @brief Lookup structures from digital map
     */
    LookupStructures::Ptr lookup_structures_;
    /**
     * @brief Version of the map used by Rail Horizon
     */
    std::string map_version_{};
};

#endif // DSD_RAIL_HORIZON_RAIL_RAIL_HORIZON_NODE_H
