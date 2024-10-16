# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('qrb_ros_sensor_service'),
        'config',
        'default.yaml'
    )

    return LaunchDescription([
        Node(
            package='qrb_ros_sensor_service',
            executable='qrb_ros_sensor_service_node',
            parameters=[config],
            output='screen',
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=["0", "0", "0.1", "0", "0", "0", "base_link", "imu_link"]
        )
    ])
