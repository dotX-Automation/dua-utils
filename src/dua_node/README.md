# dua_node

C++ base class library that extends the base `rclcpp::Node` providing direct and easy access to the [`dua_utils`](../../README.md) library features.

## Contents

This library provides the new `DUANode::NodeBase` base ROS 2 node class that implements the following features:

- [x] Automatic initialization of an embedded `PManager` object to manage node parameters from the [`params_manager`](../params_manager/README.md) library. This way, you only need to define and call `init_parameters` in your node class to automatically declare and set up the parameters.

## Usage

Just include the `dua_node.hpp` header file in your node class, and inherit from the `DUANode::NodeBase` class. The constructor accepts the following arguments:

- `std::string && node_name`: the name of the node;
- `const rclcpp::NodeOptions & opts`: the node options object, defaults to `rclcpp::NodeOptions()`;
- `verbose`: a boolean flag that enables verbose logs in various [`dua_utils`](../../README.md), defaults to `false`.

---

## License

This work is licensed under the GNU General Public License v3.0. See the [`LICENSE`](LICENSE) file for details.

## Copyright

Copyright (c) 2023, Intelligent Systems Lab, University of Rome Tor Vergata
