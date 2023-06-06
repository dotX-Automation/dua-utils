/**
 * Reference QoS profile getters for DUA modules.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 3, 2023
 */

#ifndef DUA_QOS__DUAQOS_HPP_
#define DUA_QOS__DUAQOS_HPP_

#include "visibility_control.h"

#include <rcl_action/rcl_action.h>
#include <rclcpp/rclcpp.hpp>

namespace DUAQoS
{

/**
 * @brief Returns the QoS profile for regular data topics.
 *
 * @param depth The depth of the QoS profile.
 * @return The QoS profile.
 */
rclcpp::QoS DUA_QOS_PUBLIC get_datum_qos(uint depth = 10);

/**
 * @brief Returns the QoS profile for command topics.
 *
 * @param depth The depth of the QoS profile.
 * @return The QoS profile.
 */
rclcpp::QoS DUA_QOS_PUBLIC get_command_qos(uint depth = 10);

/**
 * @brief Returns the QoS profile for scan topics, like pointclouds or laser scans.
 *
 * @param depth The depth of the QoS profile.
 * @return The QoS profile.
 */
rclcpp::QoS DUA_QOS_PUBLIC get_scan_qos(uint depth = 5);

/**
 * @brief Returns the QoS profile for image topics.
 *
 * @param depth The depth of the QoS profile.
 * @return The QoS profile.
 */
rclcpp::QoS DUA_QOS_PUBLIC get_image_qos(uint depth = 1);

/**
 * @brief Returns the default settings for action servers.
 *
 * @return Action server options structure.
 */
rcl_action_server_options_t DUA_QOS_PUBLIC get_action_server_options();

/**
 * @brief Returns the default settings for action clients.
 *
 * @return Action server options structure.
 */
rcl_action_client_options_t DUA_QOS_PUBLIC get_action_client_options();

namespace Visualization
{

/**
 * @brief Returns the QoS profile for regular data topics intended for visualization.
 *
 * @param depth The depth of the QoS profile.
 * @return The QoS profile.
 */
rclcpp::QoS DUA_QOS_PUBLIC get_datum_qos(uint depth = 5);

/**
 * @brief Returns the QoS profile for command topics intended for visualization.
 *
 * @param depth The depth of the QoS profile.
 * @return The QoS profile.
 */
rclcpp::QoS DUA_QOS_PUBLIC get_command_qos(uint depth = 5);

/**
 * @brief Returns the QoS profile for scan topics intended for visualization.
 *
 * @param depth The depth of the QoS profile.
 * @return The QoS profile.
 */
rclcpp::QoS DUA_QOS_PUBLIC get_scan_qos(uint depth = 1);

/**
 * @brief Returns the QoS profile for image topics intended for visualization.
 *
 * @param depth The depth of the QoS profile.
 * @return The QoS profile.
 */
rclcpp::QoS DUA_QOS_PUBLIC get_image_qos(uint depth = 1);

/**
 * @brief Returns the QoS profile for marker topics intended for visualization.
 *
 * @param depth The depth of the QoS profile.
 * @return The QoS profile.
 */
rclcpp::QoS DUA_QOS_PUBLIC get_marker_qos(uint depth = 1);

} // namespace Visualization

} // namespace DUAQoS

#endif // DUA_QOS__DUAQOS_HPP_
