# QRB ROS Sensor Service

## Overview

`qrb_ros_sensor_service` is a ROS package for management robot sensor tasks.

Included Features
- Nearest Object Detection: publish nearest object and distance with Lidar data.
- Imu Fusion: fuse angular velocities, accelerations, from a generic IMU device into an orientation.
- Localization Fusion: fuse odom from IMU and Odometry.
- Collision Alert: alert collision with 2D lidar sensors.
- Rollover Detection: alert robot rollover event (with IMU gyro data).

## Documentation
Please refer to the [QRB ROS Sensor Service](https://quic-qrb-ros.github.io/packages/qrb_ros_sensor_service/index.html) for more documents.
- [Overview](https://quic-qrb-ros.github.io/packages/qrb_ros_sensor_service/index.html#overview)
- [Run](https://quic-qrb-ros.github.io/packages/qrb_ros_sensor_service/index.html#run)
- [Updates](https://quic-qrb-ros.github.io/packages/qrb_ros_sensor_service/index.html#updates)

## Contributing

We would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](./CONTRIBUTING.md) and [code of conduct](./CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](../../issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](../../issues)


## Authors

* **Peng Wang** - *Initial work* - [penww](https://github.com/penww)

See also the list of [contributors](https://github.com/quic-qrb-ros/qrb_ros_sensor_service/graphs/contributors) who participated in this project.


## License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.
