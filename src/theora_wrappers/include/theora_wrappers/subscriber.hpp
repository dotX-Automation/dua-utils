/**
 * Theora subscriber wrapper, based on image_transport.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 18, 2023
 */

#ifndef THEORA_WRAPPERS__SUBSCRIBER_HPP_
#define THEORA_WRAPPERS__SUBSCRIBER_HPP_

#include "visibility_control.h"

#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <std_srvs/srv/trigger.hpp>

namespace TheoraWrappers
{

typedef std::function<void (const sensor_msgs::msg::Image::ConstSharedPtr &)> Callback;

/**
 * Theora subscriber wrapper: invokes publisher reset to trigger header retransmissions.
 */
class THEORA_WRAPPERS_PUBLIC Subscriber
{
public:
  Subscriber(
    rclcpp::Node * node,
    const std::string & base_topic,
    Callback && callback,
    rmw_qos_profile_t qos,
    bool spin = true);
  virtual ~Subscriber();

  void shutdown();

private:
  rclcpp::Node * node_;
  std::string base_topic_;
  rmw_qos_profile_t qos_;
  std::shared_ptr<image_transport::Subscriber> sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
};

} // namespace TheoraWrappers

#endif // THEORA_WRAPPERS__SUBSCRIBER_HPP_
