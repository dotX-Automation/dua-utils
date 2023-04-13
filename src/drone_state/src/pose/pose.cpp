/**
 * Drone pose library implementation.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 13, 2023
 */

#include <pose/pose.hpp>

namespace DroneState
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
  set_frame(p.get_frame());
  set_position(p.get_position());
  set_attitude(p.get_attitude());
  set_rpy(p.get_rpy());
}

/**
 * @brief Constructor with initial position.
 *
 * @param x Initial X position [m].
 * @param y Initial Y position [m].
 * @param z Initial Z position [m].
 * @param frame Coordinate frame.
 */
Pose::Pose(double x, double y, double z, CoordinateFrame frame)
{
  set_frame(frame);
  set_position(Eigen::Vector3d(x, y, z));
  set_attitude(Eigen::Quaterniond::Identity());
  set_rpy({0.0, 0.0, 0.0});
}

/**
 * @brief Constructor with initial attitude.
 *
 * @param q Initial attitude quaternion.
 * @param frame Coordinate frame.
 */
Pose::Pose(const Eigen::Quaterniond & q, CoordinateFrame frame)
{
  set_frame(frame);
  set_position(Eigen::Vector3d(0.0, 0.0, 0.0));
  set_attitude(q);
  set_rpy({0.0, 0.0, 0.0});
}

/**
 * @brief Constructor with initial euler angles.
 *
 * @param rpy_angles Initial euler angles [rad] in [-PI +PI] (may also be an array).
 * @param frame Coordinate frame.
 */
Pose::Pose(const Eigen::EulerAnglesXYZd & rpy_angles, CoordinateFrame frame)
{
  set_frame(frame);
  set_position(Eigen::Vector3d(0.0, 0.0, 0.0));
  set_attitude(Eigen::Quaterniond(rpy_angles));
  set_rpy(rpy_angles);
}

/**
 * @brief Constructor with initial position and heading.
 *
 * @param x Initial X position [m].
 * @param y Initial Y position [m].
 * @param z Initial Z position [m].
 * @param heading Initial heading [rad] in [-PI +PI].
 * @param frame Coordinate frame.
 */
Pose::Pose(double x, double y, double z, double heading, CoordinateFrame frame)
{
  set_frame(frame);
  set_position(Eigen::Vector3d(x, y, z));
  set_attitude(
    Eigen::Quaterniond(
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ())));
  set_rpy({0.0, 0.0, heading});
}

/**
 * @brief Constructor with initial position, attitude and euler angles.
 *
 * @param pos Initial position [m].
 * @param q Initial attitude quaternion.
 * @param rpy_angles Initial euler angles [rad] in [-PI +PI].
 * @param frame Coordinate frame.
 */
Pose::Pose(
  const Eigen::Vector3d & pos,
  const Eigen::Quaterniond & q,
  const Eigen::EulerAnglesXYZd & rpy_angles,
  CoordinateFrame frame)
{
  set_frame(frame);
  set_position(pos);
  set_attitude(q);
  set_rpy(rpy_angles);
}

/**
 * @brief Constructor that builds from an EulerPoseStamped ROS message.
 *
 * @param msg ROS message to build from.
 */
Pose::Pose(const EulerPoseStamped & msg)
{
  set_frame(static_cast<CoordinateFrame>(msg.coordinate_system.coordinate_system));
  set_position(
    Eigen::Vector3d(
      msg.pose.position.x,
      msg.pose.position.y,
      msg.pose.position.z));
  set_attitude(
    Eigen::Quaterniond(
      msg.pose.orientation.w,
      msg.pose.orientation.x,
      msg.pose.orientation.y,
      msg.pose.orientation.z));
  set_rpy({msg.roll, msg.pitch, msg.yaw});
}

/**
 * @brief Copy assignment operator.
 *
 * @param p Pose to copy.
 */
Pose & Pose::operator=(const Pose & p)
{
  set_frame(p.get_frame());
  set_position(p.get_position());
  set_attitude(p.get_attitude());
  set_rpy(p.get_rpy());
  return *this;
}

/**
 * @brief Move assignment operator.
 *
 * @param p Pose to move.
 */
Pose & Pose::operator=(Pose && p)
{
  set_frame(p.get_frame());
  set_position(p.get_position());
  set_attitude(p.get_attitude());
  set_rpy(p.get_rpy());
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
  msg.coordinate_system.coordinate_system = frame_;
  msg.pose.position.x = position_(0);
  msg.pose.position.y = position_(1);
  msg.pose.position.z = position_(2);
  msg.pose.orientation.w = attitude_.w();
  msg.pose.orientation.x = attitude_.x();
  msg.pose.orientation.y = attitude_.y();
  msg.pose.orientation.z = attitude_.z();
  msg.roll = rpy_.alpha();
  msg.pitch = rpy_.beta();
  msg.yaw = rpy_.gamma();
  return msg;
}

/**
 * @brief Converts to a PoseStamped ROS message (does not fill other fields).
 */
geometry_msgs::msg::PoseStamped Pose::to_pose_stamped()
{
  geometry_msgs::msg::PoseStamped msg{};
  msg.pose.position.x = position_(0);
  msg.pose.position.y = position_(1);
  msg.pose.position.z = position_(2);
  msg.pose.orientation.w = attitude_.w();
  msg.pose.orientation.x = attitude_.x();
  msg.pose.orientation.y = attitude_.y();
  msg.pose.orientation.z = attitude_.z();
  return msg;
}

/**
 * @brief Converts from NWU to NED.
 *
 * @return NED pose.
 */
Pose Pose::nwu_to_ned()
{
  // Check that the coordinate frame is coherent
  if (frame_ == CoordinateFrame::NWU) {
    return Pose(*this);
  }
  if (frame_ != CoordinateFrame::NWU) {
    throw std::runtime_error("Pose::nwu_to_ned: coordinate frame is not NWU");
  }

  // Build a converted pose
  Pose new_pose{};
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
  return new_pose;
}

/**
 * @brief Converts from NED to NWU.
 *
 * @return NWU pose.
 */
Pose Pose::ned_to_nwu()
{
  // Check that the coordinate frame is coherent
  if (frame_ == CoordinateFrame::NED) {
    return Pose(*this);
  }
  if (frame_ != CoordinateFrame::NED) {
    throw std::runtime_error("Pose::ned_to_nwu: coordinate frame is not NED");
  }

  // Build a converted pose
  Pose new_pose{};
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
  return new_pose;
}

/**
 * @brief Coordinate frame getter.
 *
 * @return Coordinate frame.
 */
CoordinateFrame Pose::get_frame() const
{
  return frame_;
}

/**
 * @brief Position getter.
 *
 * @return Position [m].
 */
Eigen::Vector3d Pose::get_position() const
{
  return position_;
}

/**
 * @brief Attitude getter.
 *
 * @return Attitude quaternion.
 */
Eigen::Quaterniond Pose::get_attitude() const
{
  return attitude_;
}

/**
 * @brief Euler angles getter.
 *
 * @return Euler angles [rad] in [-PI +PI].
 */
Eigen::EulerAnglesXYZd Pose::get_rpy() const
{
  return rpy_;
}

/**
 * @brief Coordinate frame setter.
 *
 * @param frame Coordinate frame.
 */
void Pose::set_frame(CoordinateFrame frame)
{
  // Check that the coordinate frame is valid
  if ((frame != CoordinateFrame::NWU) &&
    (frame != CoordinateFrame::NED))
  {
    throw std::invalid_argument("Invalid coordinate frame");
  }

  frame_ = frame;
}

/**
 * @brief Position setter.
 *
 * @param pos Position [m].
 */
void Pose::set_position(const Eigen::Vector3d & pos)
{
  position_ = pos;
}

/**
 * @brief Attitude setter.
 *
 * @param q Attitude quaternion.
 */
void Pose::set_attitude(const Eigen::Quaterniond & q)
{
  attitude_ = q;
}

/**
 * @brief Euler angles setter.
 *
 * @param rpy_angles Euler angles [rad] in [-PI +PI].
 */
void Pose::set_rpy(const Eigen::EulerAnglesXYZd & rpy_angles)
{
  rpy_ = rpy_angles;
}

/**
 * @brief Roto-translates a pose by a given pose.
 *
 * @param p Pose to be added.
 * @return Roto-translated pose.
 */
Pose Pose::operator*(const Pose & p) const
{
  // Check that the coordinate frame is coherent
  if (frame_ != p.frame_) {
    throw std::runtime_error("Pose::operator*: coordinate frames are not coherent");
  }

  // Compute the roto-translation using Eigen transformations
  Eigen::AngleAxisd r(p.attitude_);
  Eigen::Translation3d t(p.position_);
  Eigen::Transform<double, 3, Eigen::Affine> rt = t * r;
  Eigen::Vector3d new_position = rt * position_;
  Eigen::Quaterniond new_attitude = r * attitude_;

  // Build a new pose
  Pose new_pose{};
  new_pose.set_frame(frame_);
  new_pose.set_position(new_position);
  new_pose.set_attitude(new_attitude);
  new_pose.set_rpy(Eigen::EulerAnglesXYZd(new_attitude));
  return new_pose;
}

}  // namespace DroneState
