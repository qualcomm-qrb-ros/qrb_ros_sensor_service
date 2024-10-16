// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_sensor_service/rollover_detect.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace qrb_ros::sensor_service
{

RolloverDetect::RolloverDetect(const rclcpp::NodeOptions & options)
  : Node("rollover_detect", options)
{
  RCLCPP_INFO(get_logger(), "rollover_detect start..");

  roll_threshold_ = this->declare_parameter<double>("roll_threshold", 60);
  pitch_threshold_ = this->declare_parameter<double>("pitch_threshold", 60);
  event_interval_ = this->declare_parameter<double>("event_interval", 1);

  if (roll_threshold_ < 0 || roll_threshold_ > 90) {
    throw std::runtime_error("roll_threshold need in range [0, 90.0]");
  }
  if (pitch_threshold_ < 0 || pitch_threshold_ > 90) {
    throw std::runtime_error("pitch_threshold need in range [0, 90.0]");
  }

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 10, std::bind(&RolloverDetect::imu_callback, this, std::placeholders::_1));

  rollover_event_pub_ =
      this->create_publisher<qrb_ros_sensor_service_interfaces::msg::RolloverEvent>(
          "rollover_event", 10);
}

RolloverDetect::~RolloverDetect()
{
  RCLCPP_INFO(get_logger(), "rollover_detect stop..");
}

void RolloverDetect::imu_callback(sensor_msgs::msg::Imu::SharedPtr msg)
{
  double roll = 0;
  double pitch = 0;
  double yaw = 0;

  auto ts = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
  if (ts - last_event_ts_ < 1e9 * event_interval_) {
    return;
  }

  tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  if (std::abs(roll) * 180 < roll_threshold_ * M_PI &&
      std::abs(pitch) * 180 < pitch_threshold_ * M_PI) {
    return;
  }

  // trigger rollover event
  auto event_msg = std::make_unique<qrb_ros_sensor_service_interfaces::msg::RolloverEvent>();
  event_msg->header = msg->header;
  event_msg->header.frame_id = "rollover_detect";
  event_msg->roll_angle = roll * 180 / M_PI;
  event_msg->pitch_angle = pitch * 180 / M_PI;
  event_msg->yaw_angle = yaw * 180 / M_PI;
  rollover_event_pub_->publish(std::move(event_msg));

  last_event_ts_ = ts;
}

}  // namespace qrb_ros::sensor_service

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::sensor_service::RolloverDetect)
