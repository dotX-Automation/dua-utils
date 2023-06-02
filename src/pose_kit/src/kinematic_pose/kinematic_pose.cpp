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
  set_velocity(kp.get_velocity());
  set_angular_velocity(kp.get_angular_velocity());
  set_twist_covariance(kp.get_twist_covariance());
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
 */
KinematicPose::KinematicPose(
  double x, double y, double z,
  double vx, double vy, double vz)
: Pose(x, y, z)
{
  set_velocity(Eigen::Vector3d(vx, vy, vz));
  set_angular_velocity(Eigen::Vector3d::Zero());
}

/**
 * @brief Constructor with initial attitude and angular velocity.
 *
 * @param q Initial attitude quaternion.
 * @param angular_vel Initial angular velocity [rad/s].
 */
KinematicPose::KinematicPose(
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & angular_vel)
: Pose(q)
{
  set_velocity(Eigen::Vector3d::Zero());
  set_angular_velocity(angular_vel);
}

/**
 * @brief Constructor with initial euler angles and angular velocity.
 *
 * @param rpy_angles Initial euler angles [rad].
 * @param angular_vel Initial angular velocity [rad/s].
 */
KinematicPose::KinematicPose(
  const Eigen::EulerAnglesXYZd & rpy_angles,
  const Eigen::Vector3d & angular_vel)
: Pose(rpy_angles)
{
  set_velocity(Eigen::Vector3d::Zero());
  set_angular_velocity(angular_vel);
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
 * @param cov Initial covariance matrix.
 */
KinematicPose::KinematicPose(
  double x, double y, double z,
  double vx, double vy, double vz,
  double heading,
  const std::array<double, 36> & cov)
: Pose(x, y, z, heading, cov)
{
  set_velocity(Eigen::Vector3d(vx, vy, vz));
  set_angular_velocity(Eigen::Vector3d::Zero());
}

/**
 * @brief Constructor with initial position, attitude, linear and angular velocity.
 *
 * @param pos Initial position [m].
 * @param q Initial attitude quaternion.
 * @param vel Initial linear velocity [m/s].
 * @param angular_vel Initial angular velocity [rad/s].
 * @param cov Initial covariance matrix.
 * @param twist_cov Initial twist covariance matrix.
 */
KinematicPose::KinematicPose(
  const Eigen::Vector3d & pos,
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & vel,
  const Eigen::Vector3d & angular_vel,
  const std::array<double, 36> & cov,
  const std::array<double, 36> & twist_cov)
: Pose(pos, q, cov)
{
  set_velocity(vel);
  set_angular_velocity(angular_vel);
  set_twist_covariance(twist_cov);
}

/**
 * @brief Copy assignment operator.
 *
 * @param kp KinematicPose to copy.
 */
KinematicPose & KinematicPose::operator=(const KinematicPose & kp)
{
  set_position(kp.get_position());
  set_attitude(kp.get_attitude());
  set_rpy(kp.get_rpy());
  set_velocity(kp.get_velocity());
  set_angular_velocity(kp.get_angular_velocity());
  set_twist_covariance(kp.get_twist_covariance());
  return *this;
}

/**
 * @brief Move assignment operator.
 *
 * @param kp KinematicPose to copy.
 */
KinematicPose & KinematicPose::operator=(KinematicPose && kp)
{
  set_position(kp.get_position());
  set_attitude(kp.get_attitude());
  set_rpy(kp.get_rpy());
  set_velocity(kp.get_velocity());
  set_angular_velocity(kp.get_angular_velocity());
  set_twist_covariance(kp.get_twist_covariance());
  return *this;
}

/**
 * @brief Linear velocity getter.
 *
 * @return Linear velocity [m/s].
 */
Eigen::Vector3d KinematicPose::get_velocity() const
{
  return velocity_;
}

/**
 * @brief Angular velocity getter.
 *
 * @return Angular velocity [rad/s].
 */
Eigen::Vector3d KinematicPose::get_angular_velocity() const
{
  return angular_velocity_;
}

/**
 * @brief Twist covariance getter.
 */
std::array<double, 36> KinematicPose::get_twist_covariance() const
{
  return twist_covariance_;
}

/**
 * @brief Linear velocity setter.
 *
 * @param vel Linear velocity [m/s].
 */
void KinematicPose::set_velocity(const Eigen::Vector3d & vel)
{
  velocity_ = vel;
}

/**
 * @brief Angular velocity setter.
 *
 * @param angular_vel Angular velocity [rad/s].
 */
void KinematicPose::set_angular_velocity(const Eigen::Vector3d & angular_vel)
{
  angular_velocity_ = angular_vel;
}

/**
 * @brief Twist covariance setter.
 *
 * @param twist_cov Twist covariance.
 */
void KinematicPose::set_twist_covariance(const std::array<double, 36> & twist_cov)
{
  twist_covariance_ = twist_cov;
}

/**
 * @brief Right-multiplies by a given pose.
 *
 * @param kp Transform pose.
 * @return Transformed pose.
 *
 * @throws std::runtime_error if the coordinate frame is not coherent.
 */
KinematicPose KinematicPose::operator*(const KinematicPose & kp) const
{
  // TODO Check that the coordinate frame is coherent

  // Build a new pose
  KinematicPose new_pose =
    dynamic_cast<const Pose &>(*this).operator*(dynamic_cast<const Pose &>(kp));
  // TODO Transform velocities
  new_pose.set_twist_covariance(this->get_twist_covariance());
  return new_pose;
}

} // namespace PoseKit
