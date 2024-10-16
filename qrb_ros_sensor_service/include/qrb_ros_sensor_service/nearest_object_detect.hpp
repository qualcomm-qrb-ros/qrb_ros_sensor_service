// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_SENSOR_SERVICE__NEAREST_OBJECT_DETECT_HPP_
#define QRB_ROS_SENSOR_SERVICE__NEAREST_OBJECT_DETECT_HPP_

#include "qrb_ros_sensor_service_interfaces/msg/nearest_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace qrb_ros::sensor_service
{

class NearestObjectDetect : public rclcpp::Node
{
public:
  explicit NearestObjectDetect(const rclcpp::NodeOptions & options);
  ~NearestObjectDetect();

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<qrb_ros_sensor_service_interfaces::msg::NearestObject>::SharedPtr
      nearest_obj_pub_;

  void laser_callback(sensor_msgs::msg::LaserScan::SharedPtr msg);
};

}  // namespace qrb_ros::sensor_service

#endif  // QRB_ROS_SENSOR_SERVICE__NEAREST_OBJECT_DETECT_HPP_
