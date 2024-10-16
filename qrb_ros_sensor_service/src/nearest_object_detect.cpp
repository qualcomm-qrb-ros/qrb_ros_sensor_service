// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_sensor_service/nearest_object_detect.hpp"

namespace qrb_ros::sensor_service
{

NearestObjectDetect::NearestObjectDetect(const rclcpp::NodeOptions & options)
  : Node("nearest_object_detect", options)
{
  RCLCPP_INFO(get_logger(), "nearest_object_detect start..");

  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&NearestObjectDetect::laser_callback, this, std::placeholders::_1));

  nearest_obj_pub_ = this->create_publisher<qrb_ros_sensor_service_interfaces::msg::NearestObject>(
      "nearest_object", 10);
}

NearestObjectDetect::~NearestObjectDetect()
{
  RCLCPP_INFO(get_logger(), "nearest_object_detect stop..");
}

void NearestObjectDetect::laser_callback(sensor_msgs::msg::LaserScan::SharedPtr laser_msg)
{
  float min_distance = std::numeric_limits<float>::infinity();
  int min_index = -1;

  for (std::size_t i = 0; i < laser_msg->ranges.size(); i++) {
    if (std::isfinite(laser_msg->ranges[i]) && laser_msg->ranges[i] < min_distance) {
      min_distance = laser_msg->ranges[i];
      min_index = i;
    }
  }

  if (min_index >= 0) {
    float angle = laser_msg->angle_min + min_index * laser_msg->angle_increment;

    auto msg = std::make_unique<qrb_ros_sensor_service_interfaces::msg::NearestObject>();
    msg->header = laser_msg->header;
    msg->header.frame_id = "";
    msg->distance = min_distance;
    msg->angle = angle;

    nearest_obj_pub_->publish(std::move(msg));
  }
}

}  // namespace qrb_ros::sensor_service

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::sensor_service::NearestObjectDetect)
