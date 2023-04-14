/**
 * Kinematic drone pose library implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 14, 2023
 */

#include <kinematic_pose/kinematic_pose.hpp>

namespace DroneState
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
 * @param frame Coordinate frame.
 */
KinematicPose::KinematicPose(
  double x, double y, double z,
  double vx, double vy, double vz,
  CoordinateFrame frame)
: Pose(x, y, z, frame)
{
  set_velocity(Eigen::Vector3d(vx, vy, vz));
  set_angular_velocity(Eigen::Vector3d::Zero());
}

/**
 * @brief Constructor with initial attitude and angular velocity.
 *
 * @param q Initial attitude quaternion.
 * @param angular_vel Initial angular velocity [rad/s].
 * @param frame Coordinate frame.
 */
KinematicPose::KinematicPose(
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & angular_vel,
  CoordinateFrame frame)
: Pose(q, frame)
{
  set_velocity(Eigen::Vector3d::Zero());
  set_angular_velocity(angular_vel);
}

/**
 * @brief Constructor with initial euler angles and angular velocity.
 *
 * @param rpy_angles Initial euler angles [rad].
 * @param angular_vel Initial angular velocity [rad/s].
 * @param frame Coordinate frame.
 */
KinematicPose::KinematicPose(
  const Eigen::EulerAnglesXYZd & rpy_angles,
  const Eigen::Vector3d & angular_vel,
  CoordinateFrame frame)
: Pose(rpy_angles, frame)
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
 */
KinematicPose::KinematicPose(
  double x, double y, double z,
  double vx, double vy, double vz,
  double heading,
  CoordinateFrame frame)
: Pose(x, y, z, heading, frame)
{
  set_velocity(Eigen::Vector3d(vx, vy, vz));
  set_angular_velocity(Eigen::Vector3d::Zero());
}

/**
 * @brief Constructor with initial position, attitude, linear and angular velocity.
 *
 * @param pos Initial position [m].
 * @param q Initial attitude quaternion.
 * @param rpy_angles Initial euler angles [rad].
 * @param vel Initial linear velocity [m/s].
 * @param angular_vel Initial angular velocity [rad/s].
 * @param frame Coordinate frame.
 */
KinematicPose::KinematicPose(
  const Eigen::Vector3d & pos,
  const Eigen::Quaterniond & q,
  const Eigen::EulerAnglesXYZd & rpy_angles,
  const Eigen::Vector3d & vel,
  const Eigen::Vector3d & angular_vel,
  CoordinateFrame frame)
: Pose(pos, q, rpy_angles, frame)
{
  set_velocity(vel);
  set_angular_velocity(angular_vel);
}

/**
 * @brief Constructor that builds from an EulerPoseStamped ROS message.
 *
 * @param msg ROS message to build from.
 */
KinematicPose::KinematicPose(const EulerPoseStamped & msg)
: Pose(msg)
{
  set_velocity(Eigen::Vector3d::Zero());
  set_angular_velocity(Eigen::Vector3d::Zero());
}

/**
 * @brief Copy assignment operator.
 *
 * @param kp KinematicPose to copy.
 */
KinematicPose & KinematicPose::operator=(const KinematicPose & kp)
{
  set_frame(kp.get_frame());
  set_position(kp.get_position());
  set_attitude(kp.get_attitude());
  set_rpy(kp.get_rpy());
  set_velocity(kp.get_velocity());
  set_angular_velocity(kp.get_angular_velocity());
  return *this;
}

/**
 * @brief Move assignment operator.
 *
 * @param kp KinematicPose to copy.
 */
KinematicPose & KinematicPose::operator=(KinematicPose && kp)
{
  set_frame(kp.get_frame());
  set_position(kp.get_position());
  set_attitude(kp.get_attitude());
  set_rpy(kp.get_rpy());
  set_velocity(kp.get_velocity());
  set_angular_velocity(kp.get_angular_velocity());
  return *this;
}

/**
 * @brief Convetrs from NWU to NED.
 *
 * @return NED pose.
 */
KinematicPose KinematicPose::nwu_to_ned()
{
  // Check that the coordinate frame is coherent
  if (frame_ == CoordinateFrame::NWU) {
    return KinematicPose(*this);
  }
  if (frame_ != CoordinateFrame::NWU) {
    throw std::runtime_error("KinematicPose::nwu_to_ned: coordinate frame is not NWU");
  }

  // Build a converted pose
  KinematicPose new_pose{};
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
  return new_pose;
}

/**
 * @brief Convetrs from NED to NWU.
 *
 * @return NWU pose.
 */
KinematicPose KinematicPose::ned_to_nwu()
{
  // Check that the coordinate frame is coherent
  if (frame_ == CoordinateFrame::NED) {
    return KinematicPose(*this);
  }
  if (frame_ != CoordinateFrame::NED) {
    throw std::runtime_error("KinematicPose::nwu_to_ned: coordinate frame is not NED");
  }

  // Build a converted pose
  KinematicPose new_pose{};
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
  return new_pose;
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
 * @brief Roto-translates a pose by a given pose.
 *
 * @param kp Pose to be added.
 * @return Roto-translated pose.
 */
KinematicPose KinematicPose::operator*(const KinematicPose & kp) const
{
  // Check that the coordinate frame is coherent
  if (frame_ != kp.frame_) {
    throw std::runtime_error("KinematicPose::operator*: coordinate frames are not coherent");
  }

  // Build a new pose
  Eigen::AngleAxisd r = kp.get_rotation();
  KinematicPose new_pose =
    dynamic_cast<const Pose &>(*this).operator*(dynamic_cast<const Pose &>(kp));
  new_pose.set_velocity(r * velocity_);
  new_pose.set_angular_velocity(r * angular_velocity_);
  return new_pose;
}

} // namespace DroneState
