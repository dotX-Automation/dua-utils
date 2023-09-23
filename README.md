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
- [x] [`filters`](src/filters/README.md): C++ library to implement dynamic, discrete-time scalar filters as ROS 2 plugins. Requires `pluginlib`.
- [x] [`params_manager`](src/params_manager/README.md): C++ library to easily manage ROS 2 node parameters with the `rclcpp` API.
- [x] [`polynomial_kit`](src/polynomial_kit/README.md): C++ library based on `Eigen` to represent polynomial function.
- [x] [`pose_kit`](src/pose_kit/README.md): Collection of C++ libraries to store and manipulate the state of a rigid body in 2D/3D space.
- [x] [`simple_serviceclient_py`](src/simple_serviceclient_py/README.md): Python library that wraps the `rclpy` service client providing a simpler interface.
- [x] [`theora_wrappers`](src/theora_wrappers/README.md): C++ library to implement auto-resetting `image_transport` publishers and subscribers for Theora-encoded image streams.

## Usage

This repository is organized as a ROS 2 workspace for development, and can be built as such.

**The supported ROS 2 distribution is currently Humble Hawksbill.**

It is cloned and built as part of the build process of every base unit of [`dua-foundation`](https://github.com/IntelligentSystemsLabUTV/dua-foundation).

---

## License

This work is licensed under the GNU General Public License v3.0. See the [`LICENSE`](LICENSE) file for details.

## Copyright

Copyright (c) 2023, Intelligent Systems Lab, University of Rome Tor Vergata
