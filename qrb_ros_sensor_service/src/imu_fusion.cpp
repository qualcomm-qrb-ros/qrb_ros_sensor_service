// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_sensor_service/imu_fusion.hpp"

#include "tf2/LinearMath/Quaternion.h"

namespace qrb_ros::sensor_service
{

ImuFusion::ImuFusion(const rclcpp::NodeOptions & options) : Node("imu_fusion", options)
{
  RCLCPP_INFO(get_logger(), "imu_fusion start..");

  gyro_factor_ = this->declare_parameter<float>("gyro_factor", 0.95);

  if (gyro_factor_ < 0 || gyro_factor_ > 1) {
    throw std::runtime_error("gyro_factor need in range [0, 1.0]");
  }

  imu_raw_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&ImuFusion::imu_callback, this, std::placeholders::_1));

  imu_data_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
}

ImuFusion::~ImuFusion()
{
  RCLCPP_INFO(get_logger(), "imu_fusion stop..");
}

void ImuFusion::imu_callback(sensor_msgs::msg::Imu::SharedPtr msg)
{
  double ts = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;

  // skip first frame
  if (last_ts_ == 0) {
    last_ts_ = ts;
    return;
  }

  double dt = ts - last_ts_;

  // calculate roll and pitch with accel
  double roll_acc = std::atan2(msg->linear_acceleration.y, msg->linear_acceleration.z);
  double pitch_acc = std::atan2(msg->linear_acceleration.x * -1,
      std::sqrt(msg->linear_acceleration.y * msg->linear_acceleration.y +
                msg->linear_acceleration.z * msg->linear_acceleration.z));

  // calculate roll, pitch and yaw with gyro
  roll_ += msg->angular_velocity.x * dt;
  pitch_ += msg->angular_velocity.y * dt;
  yaw_ += msg->angular_velocity.z * dt;

  // filter data with accel and gyro
  roll_ = gyro_factor_ * roll_ + (1 - gyro_factor_) * roll_acc;
  pitch_ = gyro_factor_ * pitch_ + (1 - gyro_factor_) * pitch_acc;

  // normalize angle to [-pi, pi]
  roll_ = angle_normalize(roll_);
  pitch_ = angle_normalize(pitch_);
  yaw_ = angle_normalize(yaw_);

  tf2::Quaternion quaternion;
  quaternion.setRPY(roll_, pitch_, yaw_);

  auto imu_data_msg = std::make_unique<sensor_msgs::msg::Imu>();

  imu_data_msg->header = msg->header;
  imu_data_msg->header.frame_id = "imu_link";
  imu_data_msg->linear_acceleration = msg->linear_acceleration;
  imu_data_msg->angular_velocity = msg->angular_velocity;
  imu_data_msg->linear_acceleration_covariance = msg->linear_acceleration_covariance;
  imu_data_msg->angular_velocity_covariance = msg->angular_velocity_covariance;

  imu_data_msg->orientation.x = quaternion.x();
  imu_data_msg->orientation.y = quaternion.y();
  imu_data_msg->orientation.z = quaternion.z();
  imu_data_msg->orientation.w = quaternion.w();
  imu_data_msg->orientation_covariance = { 0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01 };

  imu_data_pub_->publish(std::move(imu_data_msg));

  last_ts_ = ts;
}

double ImuFusion::angle_normalize(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
};

}  // namespace qrb_ros::sensor_service

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::sensor_service::ImuFusion)
