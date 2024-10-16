// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_sensor_service/localization_fusion.hpp"

namespace qrb_ros::sensor_service
{

LocalizationFusion::LocalizationFusion(const rclcpp::NodeOptions & options)
  : robot_localization::RosFilter<robot_localization::Ekf>(
        rclcpp::NodeOptions(options).arguments({ "localization_fusion" }))
{
  RCLCPP_INFO(get_logger(), "localization_fusion start..");

  work_thread_ = std::make_shared<std::thread>(&LocalizationFusion::initialize, this);
}

LocalizationFusion::~LocalizationFusion()
{
  RCLCPP_INFO(get_logger(), "localization_fusion stop..");

  if (work_thread_ && work_thread_->joinable()) {
    work_thread_->join();
  }
}

}  // namespace qrb_ros::sensor_service

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::sensor_service::LocalizationFusion)
