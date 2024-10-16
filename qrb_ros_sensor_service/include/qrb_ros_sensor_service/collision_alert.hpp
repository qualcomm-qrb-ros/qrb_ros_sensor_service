// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_SENSOR_SERVICE__COLLISION_ALERT_HPP_
#define QRB_ROS_SENSOR_SERVICE__COLLISION_ALERT_HPP_

#include "qrb_ros_sensor_service_interfaces/msg/collision_alert_event.hpp"
#include "qrb_ros_sensor_service_interfaces/msg/nearest_object.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qrb_ros::sensor_service
{

class CollisionAlert : public rclcpp::Node
{
public:
  explicit CollisionAlert(const rclcpp::NodeOptions & options);
  ~CollisionAlert();

private:
  rclcpp::Subscription<qrb_ros_sensor_service_interfaces::msg::NearestObject>::SharedPtr
      nearest_obj_sub_;
  rclcpp::Publisher<qrb_ros_sensor_service_interfaces::msg::CollisionAlertEvent>::SharedPtr
      collision_event_pub_;

  void nearest_object_callback(
      qrb_ros_sensor_service_interfaces::msg::NearestObject::SharedPtr msg);

  double distance_threshold_{ 0.1 };

  // control event publish frequency
  double event_interval_{ 1.0 };
  double last_event_ts_{ 0 };
};

}  // namespace qrb_ros::sensor_service

#endif  // QRB_ROS_SENSOR_SERVICE__COLLISION_ALERT_HPP_
