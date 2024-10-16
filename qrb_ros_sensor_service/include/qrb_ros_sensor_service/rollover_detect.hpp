// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_SENSOR_SERVICE__ROLLOVER_ALERT_HPP_
#define QRB_ROS_SENSOR_SERVICE__ROLLOVER_ALERT_HPP_

#include "qrb_ros_sensor_service_interfaces/msg/rollover_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace qrb_ros::sensor_service
{

class RolloverDetect : public rclcpp::Node
{
public:
  explicit RolloverDetect(const rclcpp::NodeOptions & options);
  ~RolloverDetect();

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Publisher<qrb_ros_sensor_service_interfaces::msg::RolloverEvent>::SharedPtr
      rollover_event_pub_;

  double roll_threshold_{ 60 };
  double pitch_threshold_{ 60 };

  // control event publish frequency
  double event_interval_{ 1 };
  double last_event_ts_{ 0 };
};

}  // namespace qrb_ros::sensor_service

#endif  // QRB_ROS_SENSOR_SERVICE__ROLLOVER_ALERT_HPP_
