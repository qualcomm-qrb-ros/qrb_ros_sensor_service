// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_sensor_service/sensor_service.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto sensor_service = std::make_shared<qrb_ros::sensor_service::SensorService>(options);

  exec.add_node(sensor_service);
  exec.spin();
  exec.remove_node(sensor_service);

  rclcpp::shutdown();

  return 0;
}
