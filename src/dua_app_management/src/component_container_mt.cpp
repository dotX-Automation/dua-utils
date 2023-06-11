/**
 * DUA multithreaded component container.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * June 11, 2023
 */

#include <cstdlib>
#include <csignal>

#include <rclcpp/rclcpp.hpp>

#include <ros2_app_manager/ros2_app_manager.hpp>
#include <ros2_signal_handler/ros2_signal_handler.hpp>

#include <rclcpp_components/component_manager.hpp>

#define UNUSED(arg) (void)(arg)

using namespace DUAAppManagement;

int main(int argc, char ** argv)
{
  ROS2AppManager<rclcpp::executors::MultiThreadedExecutor,
    rclcpp_components::ComponentManager> app_manager(
    argc,
    argv,
    "dua_component_container_mt");
  auto executor = app_manager.get_executor();

  SignalHandler & sig_handler = SignalHandler::get_global_signal_handler();
  sig_handler.init(
    app_manager.get_context(),
    "dua_component_container_mt_signal_handler",
    nullptr,
    [executor](int sig, std::string & logger_name) -> void {
      UNUSED(logger_name);
      if (sig == SIGINT || sig == SIGTERM || sig == SIGQUIT) {
        executor->cancel();
      }
    });
  sig_handler.install(SIGINT);
  sig_handler.install(SIGTERM);
  sig_handler.install(SIGQUIT);
  sig_handler.install(SIGPIPE);
  sig_handler.install(SIGUSR1);
  sig_handler.install(SIGUSR2);
  sig_handler.install(SIGCHLD);
  sig_handler.ignore(SIGHUP);

  app_manager.run();

  app_manager.shutdown();
  sig_handler.fini();

  exit(EXIT_SUCCESS);
}
