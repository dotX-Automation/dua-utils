/**
 * Dynamic pose library implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 14, 2023
 */

#include <dynamic_pose/dynamic_pose.hpp>

namespace PoseKit
{

/**
 * @brief Copy constructor.
 *
 * @param dp Pose to copy.
 */
DynamicPose::DynamicPose(const DynamicPose & dp)
: KinematicPose(dynamic_cast<const KinematicPose &>(dp))
{
  this->set_acceleration(dp.get_acceleration());
  this->set_angular_acceleration(dp.get_angular_acceleration());
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
 * @param header ROS header.
 */
DynamicPose::DynamicPose(
  double x, double y, double z,
  double vx, double vy, double vz,
  double ax, double ay, double az,
  const std_msgs::msg::Header & header)
: KinematicPose(x, y, z, vx, vy, vz, header)
{
  this->set_acceleration(Eigen::Vector3d(ax, ay, az));
  this->set_angular_acceleration(Eigen::Vector3d::Zero());
}

/**
 * @brief Constructor with initial attitude, angular velocity, and angular acceleration.
 *
 * @param q Initial attitude quaternion.
 * @param angular_vel Initial angular velocity [rad/s].
 * @param angular_accel Initial angular acceleration [rad/s^2].
 * @param header ROS header.
 */
DynamicPose::DynamicPose(
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & angular_vel,
  const Eigen::Vector3d & angular_accel,
  const std_msgs::msg::Header & header)
: KinematicPose(q, angular_vel, header)
{
  this->set_acceleration(Eigen::Vector3d::Zero());
  this->set_angular_acceleration(angular_accel);
}

/**
 * @brief Constructor with initial euler angles, angular velocity, and angular acceleration.
 *
 * @param rpy_angles Initial euler angles [rad].
 * @param angular_vel Initial angular velocity [rad/s].
 * @param angular_accel Initial angular acceleration [rad/s^2].
 * @param header ROS header.
 */
DynamicPose::DynamicPose(
  const Eigen::EulerAnglesXYZd & rpy_angles,
  const Eigen::Vector3d & angular_vel,
  const Eigen::Vector3d & angular_accel,
  const std_msgs::msg::Header & header)
: KinematicPose(rpy_angles, angular_vel, header)
{
  this->set_acceleration(Eigen::Vector3d::Zero());
  this->set_angular_acceleration(angular_accel);
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
 * @param header ROS header.
 * @param cov Initial covariance matrix.
 */
DynamicPose::DynamicPose(
  double x, double y, double z,
  double vx, double vy, double vz,
  double ax, double ay, double az,
  double heading,
  const std_msgs::msg::Header & header,
  const std::array<double, 36> & cov)
: KinematicPose(x, y, z, vx, vy, vz, heading, header, cov)
{
  this->set_acceleration(Eigen::Vector3d(ax, ay, az));
  this->set_angular_acceleration(Eigen::Vector3d::Zero());
}

/**
 * @brief Constructor with initial position, attitude, linear and angular velocity and acceleration.
 *
 * @param pos Initial position [m].
 * @param q Initial attitude quaternion.
 * @param vel Initial linear velocity [m/s].
 * @param angular_vel Initial angular velocity [rad/s].
 * @param accel Initial linear acceleration [m/s^2].
 * @param angular_accel Initial angular acceleration [rad/s^2].
 * @param header ROS header.
 * @param cov Initial covariance matrix.
 * @param twist_cov Initial twist covariance matrix.
 * @param accel_cov Initial acceleration covariance matrix.
 */
DynamicPose::DynamicPose(
  const Eigen::Vector3d & pos,
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & vel,
  const Eigen::Vector3d & angular_vel,
  const Eigen::Vector3d & accel,
  const Eigen::Vector3d & angular_accel,
  const std_msgs::msg::Header & header,
  const std::array<double, 36> & cov,
  const std::array<double, 36> & twist_cov,
  const std::array<double, 36> & accel_cov)
: KinematicPose(pos, q, vel, angular_vel, header, cov, twist_cov)
{
  this->set_acceleration(accel);
  this->set_angular_acceleration(angular_accel);
  this->set_acceleration_covariance(accel_cov);
}

/**
 * @brief Copy assignment operator.
 *
 * @param dp DynamicPose to copy.
 */
DynamicPose & DynamicPose::operator=(const DynamicPose & dp)
{
  this->set_position(dp.get_position());
  this->set_attitude(dp.get_attitude());
  this->set_header(dp.get_header());
  this->set_velocity(dp.get_velocity());
  this->set_angular_velocity(dp.get_angular_velocity());
  this->set_acceleration(dp.get_acceleration());
  this->set_angular_acceleration(dp.get_angular_acceleration());
  this->set_acceleration_covariance(dp.get_acceleration_covariance());
  return *this;
}

/**
 * @brief Move assignment operator.
 *
 * @param dp DynamicPose to copy.
 */
DynamicPose & DynamicPose::operator=(DynamicPose && dp)
{
  this->set_position(dp.get_position());
  this->set_attitude(dp.get_attitude());
  this->set_header(dp.get_header());
  this->set_velocity(dp.get_velocity());
  this->set_angular_velocity(dp.get_angular_velocity());
  this->set_acceleration(dp.get_acceleration());
  this->set_angular_acceleration(dp.get_angular_acceleration());
  this->set_acceleration_covariance(dp.get_acceleration_covariance());
  return *this;
}

} // namespace PoseKit
