// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_sensor_service/sensor_service.hpp"

#include "qrb_ros_sensor_service/collision_alert.hpp"
#include "qrb_ros_sensor_service/imu_fusion.hpp"
#include "qrb_ros_sensor_service/localization_fusion.hpp"
#include "qrb_ros_sensor_service/nearest_object_detect.hpp"
#include "qrb_ros_sensor_service/rollover_detect.hpp"

using namespace std::placeholders;

namespace qrb_ros::sensor_service
{

SensorService::SensorService(const rclcpp::NodeOptions & options) : Node("sensor_service", options)
{
  RCLCPP_INFO(get_logger(), "sensor_service start..");

  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  task_server_ = this->create_service<SensorTask>("sensor_task",
      std::bind(&SensorService::task_service_callback, this, _1, _2),
      rmw_qos_profile_services_default, callback_group_);

  executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();

  executor_thread_ = std::make_unique<std::thread>(
      std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, executor_));
}

SensorService::~SensorService()
{
  RCLCPP_INFO(get_logger(), "sensor_service stop..");

  if (executor_thread_->joinable()) {
    executor_thread_->join();
  }
}

bool SensorService::check_task_dependencies_ready(uint32_t task_id)
{
  if (dependency_map_.find(task_id) == dependency_map_.end()) {
    return true;
  }
  auto task_depends = dependency_map_.at(task_id);
  for (auto depend : task_depends) {
    if (components_.find(depend) == components_.end()) {
      return false;
    }
  }
  return true;
}

bool SensorService::check_task_could_stop(uint32_t task_id)
{
  // if any running task depends this task, could not stop
  for (auto running_task : components_) {
    if (dependency_map_.find(running_task.first) == dependency_map_.end()) {
      continue;
    }

    // if depends
    auto running_depends = dependency_map_.at(running_task.first);
    if (running_depends.find(task_id) != running_depends.end()) {
      return false;
    }
  }

  return true;
}

bool SensorService::start_task(uint32_t task_id, SensorTask::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lg(task_management_lock_);

  if (!check_task_dependencies_ready(task_id)) {
    response->err_info = "dependency tasks (id: ";
    for (int id : dependency_map_.at(task_id)) {
      response->err_info += std::to_string(id) + ",";
    }
    response->err_info += ") not enabled";
    return false;
  }

  if (components_.find(task_id) != components_.end()) {
    response->success = true;
    return true;
  }

  rclcpp::Node::SharedPtr node_ptr = nullptr;
  rclcpp::NodeOptions options;

  try {
    if (task_id == SensorTask::Request::REGISTER_NEAREST_OBJECT_DETECTION) {
      node_ptr = std::make_shared<NearestObjectDetect>(options);
    } else if (task_id == SensorTask::Request::REGISTER_IMU_FUSION) {
      node_ptr = std::make_shared<ImuFusion>(options);
    } else if (task_id == SensorTask::Request::REGISTER_LOCALIZATION_FUSION) {
      node_ptr = std::make_shared<LocalizationFusion>(options);
    } else if (task_id == SensorTask::Request::REGISTER_ROLLOVER_DETECTION) {
      node_ptr = std::make_shared<RolloverDetect>(options);
    } else if (task_id == SensorTask::Request::REGISTER_COLLISION_ALERT) {
      node_ptr = std::make_shared<CollisionAlert>(options);
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "task id: " << task_id << " unknown");
      response->err_info = "unknown task id";
      return false;
    }

    components_.emplace(task_id, node_ptr);
    executor_->add_node(node_ptr);

    RCLCPP_INFO_STREAM(get_logger(), "load task success");
    response->success = true;
    return true;

  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "load task failed: " << e.what());
    response->err_info = e.what();
    return false;
  }
}

bool SensorService::stop_task(uint32_t task_id, SensorTask::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lg(task_management_lock_);

  auto start_task_id = task_id - 1;
  if (components_.find(start_task_id) == components_.end()) {
    response->success = true;
    return true;
  }

  if (!check_task_could_stop(start_task_id)) {
    response->err_info = "can not stop, running tasks depend on it";
    return false;
  }

  auto node = components_.at(start_task_id);
  executor_->remove_node(node);
  components_.erase(start_task_id);

  response->success = true;
  return true;
}

void SensorService::task_service_callback(const SensorTask::Request::SharedPtr request,
    SensorTask::Response::SharedPtr response)
{
  auto task_id = request->task_id;

  if (task_id == request->REGISTER_NEAREST_OBJECT_DETECTION ||
      task_id == request->REGISTER_IMU_FUSION || task_id == request->REGISTER_LOCALIZATION_FUSION ||
      task_id == request->REGISTER_ROLLOVER_DETECTION ||
      task_id == request->REGISTER_COLLISION_ALERT) {
    start_task(task_id, response);
  } else if (task_id == request->UNREGISTER_NEAREST_OBJECT_DETECTION ||
             task_id == request->UNREGISTER_IMU_FUSION ||
             task_id == request->UNREGISTER_LOCALIZATION_FUSION ||
             task_id == request->UNREGISTER_ROLLOVER_DETECTION ||
             task_id == request->UNREGISTER_COLLISION_ALERT) {
    stop_task(task_id, response);
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "task id is invalid: " << task_id);
    response->success = false;
    response->err_info = "unknow task_id";
  }
}

}  // namespace qrb_ros::sensor_service

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::sensor_service::SensorService)
