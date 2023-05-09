/**
 * DUA ROS 2 node base class implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * May 8, 2023
 */

#include <dua_node/dua_node.hpp>

namespace DUANode
{

/**
 * Constructor.
 *
 * @param node_name Name of the node.
 * @param opts Node options.
 */
NodeBase::NodeBase(
  std::string && node_name,
  const rclcpp::NodeOptions & opts,
  bool verbose)
: Node(node_name, opts)
{
  // Create and initialize Parameter Manager object
  pmanager_ = std::make_shared<ParamsManager::PManager>(this, verbose);
}

/**
 * Destructor.
 */
NodeBase::~NodeBase()
{
  // Destroy Parameter Manager object
  pmanager_.reset();
}

} // namespace DUANode
