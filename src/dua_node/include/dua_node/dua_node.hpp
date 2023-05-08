/**
 * DUA ROS 2 node base class.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * May 8, 2023
 */

#ifndef DUA_NODE__DUA_NODE_HPP_
#define DUA_NODE__DUA_NODE_HPP_

#include "visibility_control.h"

#include <rclcpp/rclcpp.hpp>

#include <params_manager/params_manager.hpp>

/**
 * Extends rclcpp::Node adding features from DUA packages.
 */
class DUA_NODE_PUBLIC DUANode : public rclcpp::Node
{
public:
  DUANode(std::string && node_name, rclcpp::NodeOptions && opts = rclcpp::NodeOptions());
  ~DUANode();

protected:
  /* Parameter manager object and declaration routine. */
  ParamsManager::PManager::SharedPtr pmanager_;
};

#endif // DUA_NODE__DUA_NODE_HPP_
