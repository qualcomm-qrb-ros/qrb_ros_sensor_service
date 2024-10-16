// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_SENSOR_SERVICE__SENSOR_SERVICE_HPP_
#define QRB_ROS_SENSOR_SERVICE__SENSOR_SERVICE_HPP_

#include <map>

#include "qrb_ros_sensor_service_interfaces/srv/sensor_task.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qrb_ros::sensor_service
{

class SensorService : public rclcpp::Node
{
public:
  explicit SensorService(const rclcpp::NodeOptions & options);
  ~SensorService();

private:
  using SensorTask = qrb_ros_sensor_service_interfaces::srv::SensorTask;

  rclcpp::CallbackGroup::SharedPtr callback_group_{ nullptr };
  rclcpp::Service<SensorTask>::SharedPtr task_server_;

  std::mutex task_management_lock_;
  void task_service_callback(const SensorTask::Request::SharedPtr request,
      SensorTask::Response::SharedPtr response);
  bool start_task(uint32_t task_id, SensorTask::Response::SharedPtr response);
  bool stop_task(uint32_t task_id, SensorTask::Response::SharedPtr response);

  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::map<uint32_t, rclcpp::Node::SharedPtr> components_;
  std::shared_ptr<std::thread> executor_thread_;

  bool check_task_dependencies_ready(uint32_t task_id);
  bool check_task_could_stop(uint32_t task_id);
  const std::map<uint32_t, std::set<uint32_t>> dependency_map_{
    { SensorTask::Request::REGISTER_COLLISION_ALERT,
        { SensorTask::Request::REGISTER_NEAREST_OBJECT_DETECTION } },
    { SensorTask::Request::REGISTER_ROLLOVER_DETECTION,
        { SensorTask::Request::REGISTER_IMU_FUSION } },
    { SensorTask::Request::REGISTER_LOCALIZATION_FUSION,
        { SensorTask::Request::REGISTER_IMU_FUSION } },
  };
};

}  // namespace qrb_ros::sensor_service

#endif  // QRB_ROS_SENSOR_SERVICE__SENSOR_SERVICE_HPP_
