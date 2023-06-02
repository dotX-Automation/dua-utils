/**
 * Pose library implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 13, 2023
 */

#include <pose/pose.hpp>

namespace PoseKit
{

/**
 * @brief Default constructor.
 */
Pose::Pose()
{}

/**
 * @brief Copy constructor.
 *
 * @param p Pose to copy.
 */
Pose::Pose(const Pose & p)
{
  this->set_position(p.get_position());
  this->set_attitude(p.get_attitude());
  this->set_pose_covariance(p.get_pose_covariance());
  this->set_header(p.get_header());
}

/**
 * @brief Constructor with initial position.
 *
 * @param x Initial X position [m].
 * @param y Initial Y position [m].
 * @param z Initial Z position [m].
 * @param header Pose header.
 */
Pose::Pose(double x, double y, double z, const std_msgs::msg::Header & header)
{
  this->set_position(Eigen::Vector3d(x, y, z));
  this->set_attitude(Eigen::Quaterniond::Identity());
  this->set_header(header);
}

/**
 * @brief Constructor with initial attitude.
 *
 * @param q Initial attitude quaternion.
 * @param header Pose header.
 */
Pose::Pose(const Eigen::Quaterniond & q, const std_msgs::msg::Header & header)
{
  this->set_position(Eigen::Vector3d(0.0, 0.0, 0.0));
  this->set_attitude(q);
  this->set_header(header);
}

/**
 * @brief Constructor with initial euler angles.
 *
 * @param rpy_angles Initial euler angles [rad] in [-PI +PI] (may also be an array).
 * @param header Pose header.
 */
Pose::Pose(const Eigen::EulerAnglesXYZd & rpy_angles, const std_msgs::msg::Header & header)
{
  this->set_position(Eigen::Vector3d(0.0, 0.0, 0.0));
  this->set_rpy(rpy_angles);
  this->set_header(header);
}

/**
 * @brief Constructor with initial position and heading.
 *
 * @param x Initial X position [m].
 * @param y Initial Y position [m].
 * @param z Initial Z position [m].
 * @param heading Initial heading [rad] in [-PI +PI].
 * @param header Pose header.
 * @param cov Initial pose covariance.
 */
Pose::Pose(
  double x, double y, double z, double heading,
  const std_msgs::msg::Header & header,
  const std::array<double, 36> & cov)
{
  this->set_position(Eigen::Vector3d(x, y, z));
  this->set_attitude(
    Eigen::Quaterniond(
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ())));
  this->set_pose_covariance(cov);
  this->set_header(header);
}

/**
 * @brief Constructor with initial position, attitude and euler angles.
 *
 * @param pos Initial position [m].
 * @param q Initial attitude quaternion.
 * @param header Pose header.
 * @param cov Initial pose covariance.
 */
Pose::Pose(
  const Eigen::Vector3d & pos,
  const Eigen::Quaterniond & q,
  const std_msgs::msg::Header & header,
  const std::array<double, 36> & cov)
{
  this->set_position(pos);
  this->set_attitude(q);
  this->set_pose_covariance(cov);
  this->set_header(header);
}

/**
 * @brief Constructor that builds from an EulerPoseStamped ROS message.
 *
 * @param msg ROS message to build from.
 */
Pose::Pose(const EulerPoseStamped & msg)
{
  this->set_position(
    Eigen::Vector3d(
      msg.pose.position.x,
      msg.pose.position.y,
      msg.pose.position.z));
  this->set_attitude(
    Eigen::Quaterniond(
      msg.pose.orientation.w,
      msg.pose.orientation.x,
      msg.pose.orientation.y,
      msg.pose.orientation.z));
  this->set_header(msg.header);
}

/**
 * @brief Copy assignment operator.
 *
 * @param p Pose to copy.
 */
Pose & Pose::operator=(const Pose & p)
{
  this->set_position(p.get_position());
  this->set_attitude(p.get_attitude());
  this->set_pose_covariance(p.get_pose_covariance());
  this->set_header(p.get_header());
  return *this;
}

/**
 * @brief Move assignment operator.
 *
 * @param p Pose to move.
 */
Pose & Pose::operator=(Pose && p)
{
  this->set_position(p.get_position());
  this->set_attitude(p.get_attitude());
  this->set_rpy(p.get_rpy());
  this->set_pose_covariance(p.get_pose_covariance());
  this->set_header(p.get_header());
  return *this;
}

/**
 * @brief Default destructor.
 */
Pose::~Pose()
{}

/**
 * @brief Converts to an EulerPoseStamped ROS message (does not fill other fields).
 */
EulerPoseStamped Pose::to_euler_pose_stamped()
{
  EulerPoseStamped msg{};
  Eigen::Vector3d position = this->get_position();
  Eigen::Quaterniond attitude = this->get_attitude();
  Eigen::EulerAnglesXYZd rpy = this->get_rpy();
  msg.set__header(this->get_header());
  msg.pose.position.x = position(0);
  msg.pose.position.y = position(1);
  msg.pose.position.z = position(2);
  msg.pose.orientation.w = attitude.w();
  msg.pose.orientation.x = attitude.x();
  msg.pose.orientation.y = attitude.y();
  msg.pose.orientation.z = attitude.z();
  msg.roll = rpy.alpha();
  msg.pitch = rpy.beta();
  msg.yaw = rpy.gamma();
  return msg;
}

/**
 * @brief Converts to a PoseStamped ROS message (does not fill other fields).
 */
geometry_msgs::msg::PoseStamped Pose::to_pose_stamped()
{
  geometry_msgs::msg::PoseStamped msg{};
  Eigen::Vector3d position = this->get_position();
  Eigen::Quaterniond attitude = this->get_attitude();
  msg.set__header(this->get_header());
  msg.pose.position.x = position(0);
  msg.pose.position.y = position(1);
  msg.pose.position.z = position(2);
  msg.pose.orientation.w = attitude.w();
  msg.pose.orientation.x = attitude.x();
  msg.pose.orientation.y = attitude.y();
  msg.pose.orientation.z = attitude.z();
  return msg;
}

}  // namespace PoseKit
