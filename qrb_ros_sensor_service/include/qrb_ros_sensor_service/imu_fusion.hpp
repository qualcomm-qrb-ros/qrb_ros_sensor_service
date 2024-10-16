// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_SENSOR_SERVICE__IMU_FUSION_HPP_
#define QRB_ROS_SENSOR_SERVICE__IMU_FUSION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace qrb_ros::sensor_service
{

class ImuFusion : public rclcpp::Node
{
public:
  explicit ImuFusion(const rclcpp::NodeOptions & options);
  ~ImuFusion();

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_raw_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_pub_;

  void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg);
  double angle_normalize(double angle);

  double roll_{ 0 };
  double pitch_{ 0 };
  double yaw_{ 0 };
  double last_ts_{ 0 };

  // factor when filter with gyro and accel data
  float gyro_factor_{ 0.95 };
};

}  // namespace qrb_ros::sensor_service

#endif  // QRB_ROS_SENSOR_SERVICE__IMU_FUSION_HPP_
