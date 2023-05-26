# dua-utils

Common utilities for software packages based on the Distributed Unified Architecture.

## Contents

This repository contains the following ROS 2 utilities:

- [x] [`drone_state`](src/drone_state/README.md): C++ library to store and manipulate the state of a drone as a rigid body in 2D/3D space.
- [x] [`dua_app_management`](src/dua_app_management/README.md): Collection of software modules and source files for the management of applications and processes.
- [x] [`dua_interfaces`](src/dua_interfaces/README.md): ROS 2 interfaces for the Distributed Unified Architecture.
- [x] [`dua_node`](src/dua_node/README.md): C++ library to implement a ROS 2 node leveraging the capabilities of the Distributed Unified Architecture.
- [x] [`filters`](src/filters/README.md): C++ library to implement dynamic, discrete-time scalar filters as ROS 2 plugins. Requires `pluginlib`.
- [x] [`params_manager`](src/params_manager/README.md): C++ library to easily manage ROS 2 node parameters with the `rclcpp` API.

## Usage

This repository is organized as a ROS 2 workspace for development, and can be built as such.

**The supported ROS 2 distribution is currently Humble Hawksbill.**

It is cloned and built as part of the build process of every base unit of [`dua-foundation`](https://github.com/IntelligentSystemsLabUTV/dua-foundation).

---

## License

This work is licensed under the GNU General Public License v3.0. See the [`LICENSE`](LICENSE) file for details.

## Copyright

Copyright (c) 2023, Intelligent Systems Lab, University of Rome Tor Vergata
