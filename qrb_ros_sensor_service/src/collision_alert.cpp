// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_sensor_service/collision_alert.hpp"

namespace qrb_ros::sensor_service
{

CollisionAlert::CollisionAlert(const rclcpp::NodeOptions & options)
  : Node("collision_alert", options)
{
  RCLCPP_INFO(get_logger(), "collision_alert start..");

  distance_threshold_ = this->declare_parameter<double>("distance_threshold", 0.1);
  event_interval_ = this->declare_parameter<double>("event_interval", 1.0);

  if (distance_threshold_ <= 0) {
    throw std::runtime_error{ "distance_threshold need > 0" };
  }

  nearest_obj_sub_ =
      this->create_subscription<qrb_ros_sensor_service_interfaces::msg::NearestObject>(
          "nearest_object", 10,
          std::bind(&CollisionAlert::nearest_object_callback, this, std::placeholders::_1));

  collision_event_pub_ =
      this->create_publisher<qrb_ros_sensor_service_interfaces::msg::CollisionAlertEvent>(
          "collision_alert_event", 10);
}

CollisionAlert::~CollisionAlert()
{
  RCLCPP_INFO(get_logger(), "collision_alert stop..");
}

void CollisionAlert::nearest_object_callback(
    qrb_ros_sensor_service_interfaces::msg::NearestObject::SharedPtr msg)
{
  auto ts = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
  if (ts - last_event_ts_ < 1e9 * event_interval_) {
    return;
  }

  if (msg->distance > distance_threshold_) {
    return;
  }

  auto event_msg = std::make_unique<qrb_ros_sensor_service_interfaces::msg::CollisionAlertEvent>();
  event_msg->header = msg->header;
  event_msg->distance = msg->distance;
  event_msg->angle = msg->angle;
  collision_event_pub_->publish(std::move(event_msg));

  last_event_ts_ = ts;
}

}  // namespace qrb_ros::sensor_service

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::sensor_service::CollisionAlert)
