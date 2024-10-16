// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_SENSOR_SERVICE__LOCALIZATION_FUSION_HPP_
#define QRB_ROS_SENSOR_SERVICE__LOCALIZATION_FUSION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "robot_localization/ekf.hpp"
#include "robot_localization/ros_filter.hpp"

namespace qrb_ros::sensor_service
{

class LocalizationFusion : public robot_localization::RosFilter<robot_localization::Ekf>
{
public:
  explicit LocalizationFusion(const rclcpp::NodeOptions & options);
  ~LocalizationFusion();

private:
  std::shared_ptr<std::thread> work_thread_;
};

}  // namespace qrb_ros::sensor_service

#endif  // QRB_ROS_SENSOR_SERVICE__LOCALIZATION_FUSION_HPP_
