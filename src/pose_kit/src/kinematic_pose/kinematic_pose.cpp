/**
 * Kinematic pose library implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 14, 2023
 */

#include <kinematic_pose/kinematic_pose.hpp>

namespace PoseKit
{

/**
 * @brief Copy constructor.
 *
 * @param kp Pose to copy.
 */
KinematicPose::KinematicPose(const KinematicPose & kp)
: Pose(dynamic_cast<const Pose &>(kp))
{
  this->set_velocity(kp.get_velocity());
  this->set_angular_velocity(kp.get_angular_velocity());
  this->set_twist_covariance(kp.get_twist_covariance());
}

/**
 * @brief Constructor with initial position and linear velocity.
 *
 * @param x Initial X position [m].
 * @param y Initial Y position [m].
 * @param z Initial Z position [m].
 * @param vx Initial X linear velocity [m/s].
 * @param vy Initial Y linear velocity [m/s].
 * @param vz Initial Z linear velocity [m/s].
 * @param header ROS header.
 */
KinematicPose::KinematicPose(
  double x, double y, double z,
  double vx, double vy, double vz,
  const std_msgs::msg::Header & header)
: Pose(x, y, z, header)
{
  this->set_velocity(Eigen::Vector3d(vx, vy, vz));
  this->set_angular_velocity(Eigen::Vector3d::Zero());
}

/**
 * @brief Constructor with initial attitude and angular velocity.
 *
 * @param q Initial attitude quaternion.
 * @param angular_vel Initial angular velocity [rad/s].
 * @param header ROS header.
 */
KinematicPose::KinematicPose(
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & angular_vel,
  const std_msgs::msg::Header & header)
: Pose(q, header)
{
  this->set_velocity(Eigen::Vector3d::Zero());
  this->set_angular_velocity(angular_vel);
}

/**
 * @brief Constructor with initial euler angles and angular velocity.
 *
 * @param rpy_angles Initial euler angles [rad].
 * @param angular_vel Initial angular velocity [rad/s].
 * @param header ROS header.
 */
KinematicPose::KinematicPose(
  const Eigen::EulerAnglesXYZd & rpy_angles,
  const Eigen::Vector3d & angular_vel,
  const std_msgs::msg::Header & header)
: Pose(rpy_angles, header)
{
  this->set_velocity(Eigen::Vector3d::Zero());
  this->set_angular_velocity(angular_vel);
}

/**
 * @brief Constructor with initial position, linear velocity, and heading.
 *
 * @param x Initial X position [m].
 * @param y Initial Y position [m].
 * @param z Initial Z position [m].
 * @param vx Initial X linear velocity [m/s].
 * @param vy Initial Y linear velocity [m/s].
 * @param vz Initial Z linear velocity [m/s].
 * @param heading Initial heading [rad].
 * @param header ROS header.
 * @param cov Initial covariance matrix.
 */
KinematicPose::KinematicPose(
  double x, double y, double z,
  double vx, double vy, double vz,
  double heading,
  const std_msgs::msg::Header & header,
  const std::array<double, 36> & cov)
: Pose(x, y, z, heading, header, cov)
{
  this->set_velocity(Eigen::Vector3d(vx, vy, vz));
  this->set_angular_velocity(Eigen::Vector3d::Zero());
}

/**
 * @brief Constructor with initial position, attitude, linear and angular velocity.
 *
 * @param pos Initial position [m].
 * @param q Initial attitude quaternion.
 * @param vel Initial linear velocity [m/s].
 * @param angular_vel Initial angular velocity [rad/s].
 * @param header ROS header.
 * @param cov Initial covariance matrix.
 * @param twist_cov Initial twist covariance matrix.
 */
KinematicPose::KinematicPose(
  const Eigen::Vector3d & pos,
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & vel,
  const Eigen::Vector3d & angular_vel,
  const std_msgs::msg::Header & header,
  const std::array<double, 36> & cov,
  const std::array<double, 36> & twist_cov)
: Pose(pos, q, header, cov)
{
  this->set_velocity(vel);
  this->set_angular_velocity(angular_vel);
  this->set_twist_covariance(twist_cov);
}

/**
 * @brief Copy assignment operator.
 *
 * @param kp KinematicPose to copy.
 */
KinematicPose & KinematicPose::operator=(const KinematicPose & kp)
{
  this->set_position(kp.get_position());
  this->set_attitude(kp.get_attitude());
  this->set_header(kp.get_header());
  this->set_velocity(kp.get_velocity());
  this->set_angular_velocity(kp.get_angular_velocity());
  this->set_twist_covariance(kp.get_twist_covariance());
  return *this;
}

/**
 * @brief Move assignment operator.
 *
 * @param kp KinematicPose to copy.
 */
KinematicPose & KinematicPose::operator=(KinematicPose && kp)
{
  this->set_position(kp.get_position());
  this->set_attitude(kp.get_attitude());
  this->set_header(kp.get_header());
  this->set_velocity(kp.get_velocity());
  this->set_angular_velocity(kp.get_angular_velocity());
  this->set_twist_covariance(kp.get_twist_covariance());
  return *this;
}

} // namespace PoseKit
