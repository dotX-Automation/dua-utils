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
 * @brief Constructor that builds from a PoseStamped and a TwistStamped ROS messages.
 *
 * @param pose_stamped PoseStamped ROS message.
 * @param twist_stamped TwistStamped ROS message.
 * @param header ROS header to use (to conciliate the two headers of the messages).
 */
KinematicPose::KinematicPose(
  const geometry_msgs::msg::PoseStamped & pose_stamped,
  const geometry_msgs::msg::TwistStamped & twist_stamped,
  const std_msgs::msg::Header & header)
: Pose(pose_stamped)
{
  this->set_velocity(
    Eigen::Vector3d(
      twist_stamped.twist.linear.x,
      twist_stamped.twist.linear.y,
      twist_stamped.twist.linear.z));
  this->set_angular_velocity(
    Eigen::Vector3d(
      twist_stamped.twist.angular.x,
      twist_stamped.twist.angular.y,
      twist_stamped.twist.angular.z));
  this->set_header(header);
}

/**
 * @brief Constructor that builds from a EulerPose and a TwistStamped ROS messages.
 *
 * @param euler_pose_stamped EulerPoseStamped ROS message.
 * @param twist_stamped TwistStamped ROS message.
 * @param header ROS header to use (to conciliate the two headers of the messages).
 */
KinematicPose::KinematicPose(
  const dua_interfaces::msg::EulerPoseStamped & euler_pose_stamped,
  const geometry_msgs::msg::TwistStamped & twist_stamped,
  const std_msgs::msg::Header & header)
: Pose(euler_pose_stamped)
{
  this->set_velocity(
    Eigen::Vector3d(
      twist_stamped.twist.linear.x,
      twist_stamped.twist.linear.y,
      twist_stamped.twist.linear.z));
  this->set_angular_velocity(
    Eigen::Vector3d(
      twist_stamped.twist.angular.x,
      twist_stamped.twist.angular.y,
      twist_stamped.twist.angular.z));
  this->set_header(header);
}

/**
 * @brief Constructor that builds from a PoseWithCovarianceStamped and a TwistWithCovarianceStamped ROS messages.
 *
 * @param pose_with_cov_stamped PoseWithCovarianceStamped ROS message.
 * @param twist_with_cov_stamped TwistWithCovarianceStamped ROS message.
 * @param header ROS header to use (to conciliate the two headers of the messages).
 */
KinematicPose::KinematicPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_with_cov_stamped,
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov_stamped,
  const std_msgs::msg::Header & header)
: Pose(pose_with_cov_stamped)
{
  this->set_velocity(
    Eigen::Vector3d(
      twist_with_cov_stamped.twist.twist.linear.x,
      twist_with_cov_stamped.twist.twist.linear.y,
      twist_with_cov_stamped.twist.twist.linear.z));
  this->set_angular_velocity(
    Eigen::Vector3d(
      twist_with_cov_stamped.twist.twist.angular.x,
      twist_with_cov_stamped.twist.twist.angular.y,
      twist_with_cov_stamped.twist.twist.angular.z));
  this->set_twist_covariance(twist_with_cov_stamped.twist.covariance);
  this->set_header(header);
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

/**
 * @brief Fills and returns a TwistStamped ROS message.
 *
 * @return TwistStamped TwistStamped ROS message.
 */
geometry_msgs::msg::TwistStamped KinematicPose::to_twist_stamped() const
{
  geometry_msgs::msg::TwistStamped twist_stamped{};
  Eigen::Vector3d linear_vel = this->get_velocity();
  Eigen::Vector3d angular_vel = this->get_angular_velocity();
  twist_stamped.set__header(this->get_header());
  twist_stamped.twist.linear.set__x(linear_vel.x());
  twist_stamped.twist.linear.set__y(linear_vel.y());
  twist_stamped.twist.linear.set__z(linear_vel.z());
  twist_stamped.twist.angular.set__x(angular_vel.x());
  twist_stamped.twist.angular.set__y(angular_vel.y());
  twist_stamped.twist.angular.set__z(angular_vel.z());
  return twist_stamped;
}

/**
 * @brief Fills and returns a TwistWithCovarianceStamped ROS message.
 *
 * @return TwistWithCovarianceStamped TwistWithCovarianceStamped ROS message.
 */
geometry_msgs::msg::TwistWithCovarianceStamped KinematicPose::to_twist_with_covariance_stamped()
const
{
  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov_stamped{};
  Eigen::Vector3d linear_vel = this->get_velocity();
  Eigen::Vector3d angular_vel = this->get_angular_velocity();
  twist_with_cov_stamped.set__header(this->get_header());
  twist_with_cov_stamped.twist.twist.linear.set__x(linear_vel.x());
  twist_with_cov_stamped.twist.twist.linear.set__y(linear_vel.y());
  twist_with_cov_stamped.twist.twist.linear.set__z(linear_vel.z());
  twist_with_cov_stamped.twist.twist.angular.set__x(angular_vel.x());
  twist_with_cov_stamped.twist.twist.angular.set__y(angular_vel.y());
  twist_with_cov_stamped.twist.twist.angular.set__z(angular_vel.z());
  twist_with_cov_stamped.twist.set__covariance(this->get_twist_covariance());
  return twist_with_cov_stamped;
}

} // namespace PoseKit
