# dua-utils

Common utilities for software packages based on the Distributed Unified Architecture.

**This project is now archived since [this PR](https://github.com/dotX-Automation/dua-foundation/pull/2) in dua-foundation removed the need for a GitHub repository to pull in the DUA utils.**

**Thus, this project remains available to maintain backward compatibility with legacy targets.**

This repository is meant as an aggregator for single, independent utilities which should be all together part of a DUA environment, thus built and installed together. The purpose of this repository is then of providing an easy way of installing these packages during the build process of the DUA base units. Submodules are used to keep track of the individual repositories, which can be cloned and used independently if needed, which might be especially true if one wants to avoid using DUA containers, but still use the software that runs within them.

## Contents

This repository contains the following ROS 2 utilities:

- [x] [`dua_app_management`](https://github.com/dotX-Automation/dua_app_management/blob/a45b51d98d9c2a2ca471c013cad2b92d5099f092/README.md): Collection of software modules and source files for the management of applications and processes.
- [x] [`dua_interfaces`](https://github.com/dotX-Automation/dua_interfaces/blob/f8fed0f438a4b6819fe89bfd208ed39f13e9018f/README.md): ROS 2 interfaces for the Distributed Unified Architecture.
- [x] [`dua_node`](https://github.com/dotX-Automation/dua_node/blob/33bdebcfc5bb4f5459d9b00ee91f9991898d9fd2/README.md): C++ library to implement a ROS 2 node leveraging the capabilities of the Distributed Unified Architecture.
- [x] [`dua_qos`](https://github.com/dotX-Automation/dua_qos/blob/955730838c29c0f9f6f220001653862c7ed1c4ae/README.md): Collection of libraries to implement common ROS 2 Quality of Service (QoS) profiles for DUA modules.
- [x] [`dua_structures`](https://github.com/dotX-Automation/dua_structures/blob/31808b4294929936f06af15991aaad01ef7e58c9/README.md): Collection of libraries to implement common data structures.
- [x] [`dynamic_systems`](https://github.com/dotX-Automation/dynamic_systems/blob/f620be8ed4aedf2873c06f3f8145ef881341f212/README.md): C++ library to implement modular dynamic systems.
- [x] [`params_manager`](https://github.com/dotX-Automation/params_manager/blob/9ac6d26838ba5713fd167ed22d8306a404fbab85/README.md): C++ library to easily manage ROS 2 node parameters with the `rclcpp` API.
- [x] [`polynomial_kit`](https://github.com/dotX-Automation/polynomial_kit/blob/85f1a1dba6e35bf243704ba9dba5945bef143fab/README.md): C++ library based on `Eigen` to represent polynomial functions.
- [x] [`pose_kit`](https://github.com/dotX-Automation/pose_kit/blob/9f12b95ddd2f67b544b25e7127d6663da1be2f8e/README.md): Collection of C++ libraries to store and manipulate the state of a rigid body in 2D/3D space.
- [x] [`simple_actionclient`](https://github.com/dotX-Automation/simple_actionclient/blob/c04b2a24a3bb5845f628421e8fa7a33d86f2e015/README.md): Libraries that wrap the ROS action clients providing a simpler interface.
- [x] [`simple_serviceclient`](https://github.com/dotX-Automation/simple_serviceclient/blob/de0a67a5c405a096b627ba828ed242c37af5905b/README.md): Libraries that wrap the ROS service clients providing a simpler interface.
- [x] [`theora_wrappers`](https://github.com/dotX-Automation/theora_wrappers/blob/34abc646e63b696c1eaed33a3c53095839335249/README.md): C++ library to implement auto-resetting `image_transport` publishers and subscribers for Theora-encoded image streams.
- [x] [`transitions_ros`](https://github.com/dotX-Automation/transitions_ros/blob/7f0a2ec8dd57aec8bb0b504d2f724af375493c14/README.md): Python library that extends the `transitions` library to implement finite-state machines with ROS 2 capabilities.

## Usage

This repository is organized as a ROS 2 workspace for development, and can be built as such.

**The supported ROS 2 distribution is currently Humble Hawksbill.**

It is cloned and built as part of the build process of every base unit of [`dua-foundation`](https://github.com/dotX-Automation/dua-foundation). This implies that only the base units of `dua-foundation` automatically offer an environment to build the packages in this repository; everywhere else, dependencies must be manually installed.

---

## Copyright and License

Copyright 2024 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at <http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the License for the specific language governing permissions and limitations under the License.
