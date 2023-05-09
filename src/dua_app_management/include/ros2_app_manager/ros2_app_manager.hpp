/**
 * Wrapper class for the common main thread operations of a ROS 2 application.
 *
 * Roberto Masocco <robmasocco@gmail.com
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 14, 2023
 */

#ifndef DUA_APP_MANAGEMENT__ROS2_APP_MANAGER_HPP_
#define DUA_APP_MANAGEMENT__ROS2_APP_MANAGER_HPP_

#include <memory>
#include <stdexcept>
#include <stdio.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

namespace DUAAppManagement
{

/**
 * Wraps common ROS 2 main thread tasks.
 */
template<class ExecutorT, class NodeT>
class ROS2AppManager final
{
public:
  /**
   * @brief Constructor: initializes the ROS 2 app.
   *
   * @param argc Number of process command line arguments.
   * @param argv Process command line arguments.
   * @param module_name Application name.
   *
   * @throws RuntimeError if I/O buffering configuration fails.
   */
  ROS2AppManager(
    int argc,
    char ** argv,
    std::string && module_name = std::string("ros2_app_manager"))
  : logger_name_(module_name)
  {
    // Disable I/O buffering
    if (setvbuf(stdout, NULL, _IONBF, 0)) {
      RCLCPP_FATAL(
        rclcpp::get_logger(logger_name_),
        "Failed to disable I/O buffering");
      throw std::runtime_error("Failed to disable I/O buffering.");
    }

    // Initialize ROS 2 context
    context_ = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init_options = rclcpp::InitOptions();
    init_options.shutdown_on_signal = true;
    context_->init(argc, argv, init_options);

    // Initialize ROS 2 node
    rclcpp::NodeOptions node_opts = rclcpp::NodeOptions();
    node_opts.context(context_);
    node_ = std::make_shared<NodeT>(node_opts);

    // Initialize ROS 2 executor
    rclcpp::ExecutorOptions executor_opts = rclcpp::ExecutorOptions();
    executor_opts.context = context_;
    executor_ = std::make_shared<ExecutorT>(executor_opts);
    executor_->add_node(node_);

    // Set object state
    is_valid_ = true;

    RCLCPP_WARN(
      rclcpp::get_logger(logger_name_),
      "Process started (%d)",
      getpid());
  }

  /**
   * @brief Destructor: shuts down the ROS 2 context if object is still valid.
   */
  ~ROS2AppManager()
  {
    if (is_valid_) {
      shutdown();
    }
  }

  /**
   * @brief Runs the app, does not return until stopped by a signal.
   *
   * @throws RuntimeError if the app manager is not valid.
   */
  void run()
  {
    if (!is_valid_) {
      throw std::runtime_error("ROS 2 app manager is not valid.");
    }
    RCLCPP_INFO(
      rclcpp::get_logger(logger_name_),
      "Starting executor...");
    executor_->spin();
  }

  /**
   * @brief Shuts down the app, deleting ROS 2 structures.
   */
  void shutdown()
  {
    RCLCPP_WARN(
      rclcpp::get_logger(logger_name_),
      "Shutting down...");
    executor_->remove_node(node_);
    node_.reset();
    executor_.reset();
    context_.reset();
    is_valid_ = false;
  }

  /**
   * @brief Gets the ROS 2 context.
   *
   * @return ROS 2 context.
   */
  std::shared_ptr<rclcpp::Context> get_context() const
  {
    return context_;
  }

  /**
   * @brief Gets the ROS 2 executor.
   */
  std::shared_ptr<ExecutorT> get_executor() const
  {
    return executor_;
  }

  /**
   * @brief Gets the ROS 2 node.
   */
  std::shared_ptr<NodeT> get_node() const
  {
    return node_;
  }

private:
  /* ROS 2 context. */
  std::shared_ptr<rclcpp::Context> context_;

  /* ROS 2 executor. */
  std::shared_ptr<ExecutorT> executor_;

  /* ROS 2 node. */
  std::shared_ptr<NodeT> node_;

  /* Object state. */
  bool is_valid_ = false;

  /* Logger name. */
  std::string logger_name_;
};

} // namespace DUAAppManagement

#endif // DUA_APP_MANAGEMENT__ROS2_APP_MANAGER_HPP_
