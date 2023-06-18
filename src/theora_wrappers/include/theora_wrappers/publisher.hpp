/**
 * Theora publisher wrapper, based on image_transport.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 18, 2023
 */

#ifndef THEORA_WRAPPERS__PUBLISHER_HPP_
#define THEORA_WRAPPERS__PUBLISHER_HPP_

#include "visibility_control.h"

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <std_srvs/srv/trigger.hpp>

namespace TheoraWrappers
{

/**
 * Theora publisher wrapper: handlers header retransmissions.
 */
class THEORA_WRAPPERS_PUBLIC Publisher
{
public:
  Publisher(
    rclcpp::Node * node,
    const std::string & base_topic,
    rmw_qos_profile_t qos);
  virtual ~Publisher();

  size_t getNumSubscribers();

  void publish(const sensor_msgs::msg::Image & message);
  void publish(const sensor_msgs::msg::Image::ConstSharedPtr & message);

private:
  rclcpp::Node * node_;
  std::string base_topic_;
  rmw_qos_profile_t qos_;

  std::mutex pub_lock_;
  std::shared_ptr<image_transport::Publisher> pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  void reset_callback_(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    const std_srvs::srv::Trigger::Response::SharedPtr res);
};

} // namespace TheoraWrappers

#endif // THEORA_WRAPPERS__PUBLISHER_HPP_
