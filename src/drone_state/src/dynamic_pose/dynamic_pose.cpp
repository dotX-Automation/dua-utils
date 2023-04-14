/**
 * Dynamic drone pose library implementation.
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
 * @brief Default constructor.
 */
DynamicPose::DynamicPose()
{}

/**
 * @brief Copy constructor.
 *
 * @param dp Pose to copy.
 */
DynamicPose::DynamicPose(const DynamicPose & dp)
: KinematicPose(dp)
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
 * @param frame Coordinate frame.
 */
DynamicPose::DynamicPose(
  double x, double y, double z,
  double vx, double vy, double vz,
  double ax, double ay, double az,
  CoordinateFrame frame)
: KinematicPose(x, y, z, vx, vy, vz, frame)
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
 * @param frame Coordinate frame.
 */
DynamicPose::DynamicPose(
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & angular_vel,
  const Eigen::Vector3d & angular_accel,
  CoordinateFrame frame)
: KinematicPose(q, angular_vel, frame)
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
 * @param frame Coordinate frame.
 */
DynamicPose::DynamicPose(
  const Eigen::EulerAnglesXYZd & rpy_angles,
  const Eigen::Vector3d & angular_vel,
  const Eigen::Vector3d & angular_accel,
  CoordinateFrame frame)
: KinematicPose(rpy_angles, angular_vel, frame)
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
  double heading,
  CoordinateFrame frame)
: KinematicPose(x, y, z, vx, vy, vz, heading, frame)
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
 * @param frame Coordinate frame.
 */
DynamicPose::DynamicPose(
  const Eigen::Vector3d & pos,
  const Eigen::Quaterniond & q,
  const Eigen::EulerAnglesXYZd & rpy_angles,
  const Eigen::Vector3d & vel,
  const Eigen::Vector3d & angular_vel,
  const Eigen::Vector3d & accel,
  const Eigen::Vector3d & angular_accel,
  CoordinateFrame frame)
: KinematicPose(pos, q, rpy_angles, vel, angular_vel, frame)
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
  set_frame(dp.get_frame());
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
  set_frame(dp.get_frame());
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
 * @brief Default destructor.
 */
DynamicPose::~DynamicPose()
{}

/**
 * @brief Convetrs from NWU to NED.
 *
 * @return NED pose.
 */
DynamicPose DynamicPose::nwu_to_ned()
{
  // Check that the coordinate frame is coherent
  if (frame_ == CoordinateFrame::NWU) {
    return DynamicPose(*this);
  }
  if (frame_ != CoordinateFrame::NWU) {
    throw std::runtime_error("DynamicPose::nwu_to_ned: coordinate frame is not NWU");
  }

  // Build a converted pose
  DynamicPose new_pose{};
  new_pose.set_frame(CoordinateFrame::NED);
  new_pose.set_position(
    Eigen::Vector3d(
      position_.x(),
      -position_.y(),
      -position_.z()));
  new_pose.set_attitude(
    Eigen::Quaterniond(
      attitude_.w(),
      attitude_.x(),
      -attitude_.y(),
      -attitude_.z()));
  new_pose.set_rpy(Eigen::EulerAnglesXYZd(attitude_));
  new_pose.set_velocity(
    Eigen::Vector3d(
      velocity_.x(),
      -velocity_.y(),
      -velocity_.z()));
  new_pose.set_angular_velocity(
    Eigen::Vector3d(
      angular_velocity_.x(),
      -angular_velocity_.y(),
      -angular_velocity_.z()));
  new_pose.set_acceleration(
    Eigen::Vector3d(
      acceleration_.x(),
      -acceleration_.y(),
      -acceleration_.z()));
  new_pose.set_angular_acceleration(
    Eigen::Vector3d(
      angular_acceleration_.x(),
      -angular_acceleration_.y(),
      -angular_acceleration_.z()));
  return new_pose;
}

/**
 * @brief Convetrs from NED to NWU.
 *
 * @return NWU pose.
 */
DynamicPose DynamicPose::ned_to_nwu()
{
  // Check that the coordinate frame is coherent
  if (frame_ == CoordinateFrame::NED) {
    return DynamicPose(*this);
  }
  if (frame_ != CoordinateFrame::NED) {
    throw std::runtime_error("DynamicPose::nwu_to_ned: coordinate frame is not NED");
  }

  // Build a converted pose
  DynamicPose new_pose{};
  new_pose.set_frame(CoordinateFrame::NWU);
  new_pose.set_position(
    Eigen::Vector3d(
      position_.x(),
      -position_.y(),
      -position_.z()));
  new_pose.set_attitude(
    Eigen::Quaterniond(
      attitude_.w(),
      attitude_.x(),
      -attitude_.y(),
      -attitude_.z()));
  new_pose.set_rpy(Eigen::EulerAnglesXYZd(attitude_));
  new_pose.set_velocity(
    Eigen::Vector3d(
      velocity_.x(),
      -velocity_.y(),
      -velocity_.z()));
  new_pose.set_angular_velocity(
    Eigen::Vector3d(
      angular_velocity_.x(),
      -angular_velocity_.y(),
      -angular_velocity_.z()));
  new_pose.set_acceleration(
    Eigen::Vector3d(
      acceleration_.x(),
      -acceleration_.y(),
      -acceleration_.z()));
  new_pose.set_angular_acceleration(
    Eigen::Vector3d(
      angular_acceleration_.x(),
      -angular_acceleration_.y(),
      -angular_acceleration_.z()));
  return new_pose;
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

} // namespace DroneState
