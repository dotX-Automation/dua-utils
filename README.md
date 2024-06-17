# dua-utils

Common utilities for software packages based on the Distributed Unified Architecture.

## Contents

This repository contains the following ROS 2 utilities:

- [x] [`dua_app_management`](src/dua_app_management/README.md): Collection of software modules and source files for the management of applications and processes.
- [x] [`dua_interfaces`](src/dua_interfaces/README.md): ROS 2 interfaces for the Distributed Unified Architecture.
- [x] [`dua_node`](src/dua_node/README.md): C++ library to implement a ROS 2 node leveraging the capabilities of the Distributed Unified Architecture.
- [x] [`dua_qos`](src/dua_qos/README.md): C++ library to implement common ROS 2 Quality of Service (QoS) profiles for DUA modules.
- [x] [`dua_qos_py`](src/dua_qos_py/README.md): Python library to implement common ROS 2 Quality of Service (QoS) profiles for DUA modules.
- [x] [`dua_structures`](src/dua_structures/README.md): C++ library to implement common data structures.
- [x] [`dynamic_systems`](src/dynamic_systems/README.md): C++ library to implement modular dynamic systems.
- [x] [`params_manager`](src/params_manager/README.md): C++ library to easily manage ROS 2 node parameters with the `rclcpp` API.
- [x] [`polynomial_kit`](src/polynomial_kit/README.md): C++ library based on `Eigen` to represent polynomial function.
- [x] [`pose_kit`](src/pose_kit/README.md): Collection of C++ libraries to store and manipulate the state of a rigid body in 2D/3D space.
- [x] [`simple_actionclient`](src/simple_actionclient/README.md): C++ library that wraps the `rclcpp` action client providing a simpler interface.
- [x] [`simple_actionclient_py`](src/simple_actionclient_py/README.md): Python library that wraps the `rclpy` action client providing a simpler interface.
- [x] [`simple_serviceclient`](src/simple_serviceclient/README.md): C++ library that wraps the `rclcpp` service client providing a simpler interface.
- [x] [`simple_serviceclient_py`](src/simple_serviceclient_py/README.md): Python library that wraps the `rclpy` service client providing a simpler interface.
- [x] [`theora_wrappers`](src/theora_wrappers/README.md): C++ library to implement auto-resetting `image_transport` publishers and subscribers for Theora-encoded image streams.
- [x] [`transitions_ros`](src/transitions_ros/README.md): Python library that extends the `transitions` library to implement finite-state machines with ROS 2 capabilities.

## Usage

This repository is organized as a ROS 2 workspace for development, and can be built as such.

**The supported ROS 2 distribution is currently Humble Hawksbill.**

It is cloned and built as part of the build process of every base unit of [`dua-foundation`](https://github.com/IntelligentSystemsLabUTV/dua-foundation). This implies that only the base units of `dua-foundation` automatically offer an environment to build the packages in this repository; everywhere else, dependencies must be manually installed.

---

## Copyright and License

Copyright 2024 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at <http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the License for the specific language governing permissions and limitations under the License.
