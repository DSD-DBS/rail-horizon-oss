/*
 * SPDX-FileCopyrightText: Copyright DB Netz AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <dsd_rail_horizon/RailHorizonNode.h>

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RailHorizonNode>());
    rclcpp::shutdown();

    return 0;
}
