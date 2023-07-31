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
  qos.reliable();
  qos.durability_volatile();
  return qos;
}

/**
 * @brief Returns the default settings for action servers.
 *
 * @return Action server options structure.
 */
rcl_action_server_options_t get_action_server_options()
{
  rcl_action_server_options_t options{};
  rmw_qos_profile_t status_qos = rmw_qos_profile_default;
  status_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

  options.allocator = rcl_get_default_allocator();
  options.goal_service_qos = rmw_qos_profile_services_default;
  options.cancel_service_qos = rmw_qos_profile_services_default;
  options.result_service_qos = rmw_qos_profile_services_default;
  options.feedback_topic_qos = rmw_qos_profile_default;
  options.status_topic_qos = status_qos;

  return options;
}

/**
 * @brief Returns the default settings for action clients.
 *
 * @return Action server options structure.
 */
rcl_action_client_options_t get_action_client_options()
{
  rcl_action_client_options_t options{};
  rmw_qos_profile_t status_qos = rmw_qos_profile_default;
  status_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

  options.allocator = rcl_get_default_allocator();
  options.goal_service_qos = rmw_qos_profile_services_default;
  options.cancel_service_qos = rmw_qos_profile_services_default;
  options.result_service_qos = rmw_qos_profile_services_default;
  options.feedback_topic_qos = rmw_qos_profile_default;
  options.status_topic_qos = status_qos;

  return options;
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
