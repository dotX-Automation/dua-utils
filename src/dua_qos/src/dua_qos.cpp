/**
 * Implementation of reference QoS profile getters for DUA modules.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 3, 2023
 */

#include <dua_qos/dua_qos.hpp>

namespace DUAQoS
{

/**
 * @brief Returns the QoS profile for regular data topics.
 *
 * @param depth The depth of the QoS profile.
 * @return The QoS profile.
 */
rclcpp::QoS get_datum_qos(uint depth)
{
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(depth));
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

/**
 * @brief Returns the QoS profile for command topics.
 *
 * @param depth The depth of the QoS profile.
 * @return The QoS profile.
 */
rclcpp::QoS get_command_qos(uint depth)
{
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(depth));
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

/**
 * @brief Returns the QoS profile for scan topics, like pointclouds or laser scans.
 *
 * @param depth The depth of the QoS profile.
 * @return The QoS profile.
 */
rclcpp::QoS get_scan_qos(uint depth)
{
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(depth));
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

/**
 * @brief Returns the QoS profile for image topics.
 *
 * @param depth The depth of the QoS profile.
 * @return The QoS profile.
 */
rclcpp::QoS get_image_qos(uint depth)
{
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(depth));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

namespace Visualization
{

/**
 * @brief Returns the QoS profile for regular data topics intended for visualization.
 *
 * @param depth The depth of the QoS profile.
 * @return The QoS profile.
 */
rclcpp::QoS get_datum_qos(uint depth)
{
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(depth));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

/**
 * @brief Returns the QoS profile for command topics intended for visualization.
 *
 * @param depth The depth of the QoS profile.
 * @return The QoS profile.
 */
rclcpp::QoS get_command_qos(uint depth)
{
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(depth));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

/**
 * @brief Returns the QoS profile for scan topics intended for visualization.
 *
 * @param depth The depth of the QoS profile.
 * @return The QoS profile.
 */
rclcpp::QoS get_scan_qos(uint depth)
{
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(depth));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

/**
 * @brief Returns the QoS profile for image topics intended for visualization.
 *
 * @param depth The depth of the QoS profile.
 * @return The QoS profile.
 */
rclcpp::QoS get_image_qos(uint depth)
{
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(depth));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

/**
 * @brief Returns the QoS profile for marker topics intended for visualization.
 *
 * @param depth The depth of the QoS profile.
 * @return The QoS profile.
 */
rclcpp::QoS get_marker_qos(uint depth)
{
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(depth));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

} // namespace Visualization

} // namespace DUAQoS
