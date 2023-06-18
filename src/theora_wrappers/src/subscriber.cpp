/**
 * Theora subscriber wrapper implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 18, 2023
 */

#define UNUSED(arg) (void)(arg)

#include <theora_wrappers/subscriber.hpp>

namespace TheoraWrappers
{

/**
 * @brief Constructor.
 *
 * @param node The ROS 2 node.
 * @param base_topic The base topic name.
 * @param callback The callback function.
 * @param qos The QoS profile.
 * @param spin Whether to spin the node or not while waiting for the reset service.
 *
 * @throws RuntimeError if the remote publisher cannot be reset.
 */
Subscriber::Subscriber(
  rclcpp::Node * node,
  const std::string & base_topic,
  Callback && callback,
  rmw_qos_profile_t qos,
  bool spin)
: node_(node),
  base_topic_(base_topic),
  qos_(qos)
{
  // Instantiate reset service
  reset_client_ = node_->create_client<std_srvs::srv::Trigger>(base_topic_ + "/stream/reset");

  // Instantiate subscriber
  sub_ = std::make_shared<image_transport::Subscriber>(
    image_transport::create_subscription(
      node_,
      base_topic_ + "/stream",
      callback,
      "theora",
      qos_));

  // Invoke stream reset
  while (!reset_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      throw std::runtime_error("Interrupted while waiting for the service, exiting...");
    }
    RCLCPP_WARN(
      node_->get_logger(),
      "Stream reset service (%s) not available...",
      reset_client_->get_service_name());
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response = reset_client_->async_send_request(request);
  if (spin) {
    if (rclcpp::spin_until_future_complete(
        node_->get_node_base_interface(),
        response) != rclcpp::FutureReturnCode::SUCCESS)
    {
      reset_client_->remove_pending_request(response);
      reset_client_.reset();
      RCLCPP_ERROR(node_->get_logger(), "Failed to contact remote publisher");
      throw std::runtime_error("Failed to contact remote publisher");
    }
  }
  if (!response.get()->success) {
    reset_client_.reset();
    RCLCPP_ERROR(node_->get_logger(), "Failed to reset remote publisher");
    throw std::runtime_error("Failed to reset remote publisher");
  }
}

/**
 * @brief Destructor.
 */
Subscriber::~Subscriber()
{
  shutdown();
}

/**
 * @brief Shuts down the subscriber.
 */
void Subscriber::shutdown()
{
  if (sub_) {
    sub_->shutdown();
    sub_.reset();
  }
  reset_client_.reset();
}

} // namespace TheoraWrappers
