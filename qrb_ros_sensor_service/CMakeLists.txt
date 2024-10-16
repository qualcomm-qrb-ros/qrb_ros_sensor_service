cmake_minimum_required(VERSION 3.8)
project(qrb_ros_sensor_service VERSION 1.0.0)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# components for sensor releated functions
ament_auto_add_library(${PROJECT_NAME}_nearest_object_detect SHARED
  src/nearest_object_detect.cpp
)
rclcpp_components_register_nodes(${PROJECT_NAME}_nearest_object_detect
  qrb_ros::sensor_service::NearestObjectDetect
)

ament_auto_add_library(${PROJECT_NAME}_collision_alert SHARED
  src/collision_alert.cpp
)
rclcpp_components_register_nodes(${PROJECT_NAME}_collision_alert
  qrb_ros::sensor_service::CollisionAlert
)

ament_auto_add_library(${PROJECT_NAME}_imu_fusion SHARED
  src/imu_fusion.cpp
)
rclcpp_components_register_nodes(${PROJECT_NAME}_imu_fusion
  qrb_ros::sensor_service::ImuFusion
)

ament_auto_add_library(${PROJECT_NAME}_localization_fusion SHARED
  src/localization_fusion.cpp
)
target_link_libraries(${PROJECT_NAME}_localization_fusion
  ${robot_localization_LIBRARIES}
)
rclcpp_components_register_nodes(${PROJECT_NAME}_localization_fusion
  qrb_ros::sensor_service::LocalizationFusion
)

ament_auto_add_library(${PROJECT_NAME}_rollover_detect SHARED
  src/rollover_detect.cpp
)
rclcpp_components_register_nodes(${PROJECT_NAME}_rollover_detect
  qrb_ros::sensor_service::RolloverDetect
)

# sensor service component
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/sensor_service.cpp
)
rclcpp_components_register_nodes(${PROJECT_NAME} qrb_ros::sensor_service::SensorService)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch config)
