/**
 * Dynamic pose library implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 14, 2023
 */

#include <dynamic_pose/dynamic_pose.hpp>

namespace DroneState
{

/**
 * @brief Copy constructor.
 *
 * @param dp Pose to copy.
 */
DynamicPose::DynamicPose(const DynamicPose & dp)
: KinematicPose(dynamic_cast<const KinematicPose &>(dp))
{
  set_acceleration(dp.get_acceleration());
  set_angular_acceleration(dp.get_angular_acceleration());
}

/**
 * @brief Constructor with initial position, linear velocity, and linear acceleration.
 *
 * @param x Initial X position [m].
 * @param y Initial Y position [m].
 * @param z Initial Z position [m].
 * @param vx Initial X linear velocity [m/s].
 * @param vy Initial Y linear velocity [m/s].
 * @param vz Initial Z linear velocity [m/s].
 */
DynamicPose::DynamicPose(
  double x, double y, double z,
  double vx, double vy, double vz,
  double ax, double ay, double az)
: KinematicPose(x, y, z, vx, vy, vz)
{
  set_acceleration(Eigen::Vector3d(ax, ay, az));
  set_angular_acceleration(Eigen::Vector3d::Zero());
}

/**
 * @brief Constructor with initial attitude, angular velocity, and angular acceleration.
 *
 * @param q Initial attitude quaternion.
 * @param angular_vel Initial angular velocity [rad/s].
 * @param angular_accel Initial angular acceleration [rad/s^2].
 */
DynamicPose::DynamicPose(
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & angular_vel,
  const Eigen::Vector3d & angular_accel)
: KinematicPose(q, angular_vel)
{
  set_acceleration(Eigen::Vector3d::Zero());
  set_angular_acceleration(angular_accel);
}

/**
 * @brief Constructor with initial euler angles, angular velocity, and angular acceleration.
 *
 * @param rpy_angles Initial euler angles [rad].
 * @param angular_vel Initial angular velocity [rad/s].
 * @param angular_accel Initial angular acceleration [rad/s^2].
 */
DynamicPose::DynamicPose(
  const Eigen::EulerAnglesXYZd & rpy_angles,
  const Eigen::Vector3d & angular_vel,
  const Eigen::Vector3d & angular_accel)
: KinematicPose(rpy_angles, angular_vel)
{
  set_acceleration(Eigen::Vector3d::Zero());
  set_angular_acceleration(angular_accel);
}

/**
 * @brief Constructor with initial position, linear velocity, linear acceleration, and heading.
 *
 * @param x Initial X position [m].
 * @param y Initial Y position [m].
 * @param z Initial Z position [m].
 * @param vx Initial X linear velocity [m/s].
 * @param vy Initial Y linear velocity [m/s].
 * @param vz Initial Z linear velocity [m/s].
 * @param ax Initial X linear acceleration [m/s^2].
 * @param ay Initial Y linear acceleration [m/s^2].
 * @param az Initial Z linear acceleration [m/s^2].
 * @param heading Initial heading [rad].
 */
DynamicPose::DynamicPose(
  double x, double y, double z,
  double vx, double vy, double vz,
  double ax, double ay, double az,
  double heading)
: KinematicPose(x, y, z, vx, vy, vz, heading)
{
  set_acceleration(Eigen::Vector3d(ax, ay, az));
  set_angular_acceleration(Eigen::Vector3d::Zero());
}

/**
 * @brief Constructor with initial position, attitude, linear and angular velocity and acceleration.
 *
 * @param pos Initial position [m].
 * @param q Initial attitude quaternion.
 * @param rpy_angles Initial euler angles [rad].
 * @param vel Initial linear velocity [m/s].
 * @param angular_vel Initial angular velocity [rad/s].
 * @param accel Initial linear acceleration [m/s^2].
 * @param angular_accel Initial angular acceleration [rad/s^2].
 */
DynamicPose::DynamicPose(
  const Eigen::Vector3d & pos,
  const Eigen::Quaterniond & q,
  const Eigen::EulerAnglesXYZd & rpy_angles,
  const Eigen::Vector3d & vel,
  const Eigen::Vector3d & angular_vel,
  const Eigen::Vector3d & accel,
  const Eigen::Vector3d & angular_accel)
: KinematicPose(pos, q, rpy_angles, vel, angular_vel)
{
  set_acceleration(accel);
  set_angular_acceleration(angular_accel);
}

/**
 * @brief Constructor that builds from an EulerPoseStamped ROS message.
 *
 * @param msg ROS message to build from.
 */
DynamicPose::DynamicPose(const EulerPoseStamped & msg)
: KinematicPose(msg)
{
  set_acceleration(Eigen::Vector3d::Zero());
  set_angular_acceleration(Eigen::Vector3d::Zero());
}

/**
 * @brief Copy assignment operator.
 *
 * @param dp DynamicPose to copy.
 */
DynamicPose & DynamicPose::operator=(const DynamicPose & dp)
{
  set_position(dp.get_position());
  set_attitude(dp.get_attitude());
  set_rpy(dp.get_rpy());
  set_velocity(dp.get_velocity());
  set_angular_velocity(dp.get_angular_velocity());
  set_acceleration(dp.get_acceleration());
  set_angular_acceleration(dp.get_angular_acceleration());
  return *this;
}

/**
 * @brief Move assignment operator.
 *
 * @param dp DynamicPose to copy.
 */
DynamicPose & DynamicPose::operator=(DynamicPose && dp)
{
  set_position(dp.get_position());
  set_attitude(dp.get_attitude());
  set_rpy(dp.get_rpy());
  set_velocity(dp.get_velocity());
  set_angular_velocity(dp.get_angular_velocity());
  set_acceleration(dp.get_acceleration());
  set_angular_acceleration(dp.get_angular_acceleration());
  return *this;
}

/**
 * @brief Linear acceleration getter.
 *
 * @return Linear acceleration [m/s^2].
 */
Eigen::Vector3d DynamicPose::get_acceleration() const
{
  return acceleration_;
}

/**
 * @brief Angular acceleration getter.
 *
 * @return Angular acceleration [rad/s^2].
 */
Eigen::Vector3d DynamicPose::get_angular_acceleration() const
{
  return angular_acceleration_;
}

/**
 * @brief Linear acceleration setter.
 *
 * @param accel Linear acceleration [m/s].
 */
void DynamicPose::set_acceleration(const Eigen::Vector3d & accel)
{
  acceleration_ = accel;
}

/**
 * @brief Angular acceleration setter.
 *
 * @param angular_accel Angular acceleration [rad/s^2].
 */
void DynamicPose::set_angular_acceleration(const Eigen::Vector3d & angular_accel)
{
  angular_acceleration_ = angular_accel;
}

/**
 * @brief Roto-translates a pose by a given pose.
 *
 * @param dp Pose to be added.
 * @return Roto-translated pose.
 *
 * @throws std::runtime_error if the coordinate frame is not coherent.
 */
DynamicPose DynamicPose::operator*(const DynamicPose & dp) const
{
  // TODO Check that the coordinate frame is coherent

  // Build a new pose
  DynamicPose new_pose =
    dynamic_cast<const KinematicPose &>(*this).operator*(dynamic_cast<const KinematicPose &>(dp));
  // TODO Transform accelerations
  return new_pose;
}

} // namespace DroneState
