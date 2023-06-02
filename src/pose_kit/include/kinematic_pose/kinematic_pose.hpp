/**
 * Kinematic pose library, based on Eigen geometry types.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 14, 2023
 */

#ifndef POSE_KIT__KINEMATIC_POSE_HPP_
#define POSE_KIT__KINEMATIC_POSE_HPP_

#include "visibility_control.h"

#include <stdexcept>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

#include <dua_interfaces/msg/euler_pose_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_msgs/msg/header.hpp>

#include <pose/pose.hpp>

namespace PoseKit
{

/**
 * Represents position, orientation, and velocity of an autonomous agent.
 */
class KINEMATIC_POSE_PUBLIC KinematicPose : public Pose
{
public:
  /* Constructors. */
  KinematicPose() {}
  KinematicPose(const KinematicPose & kp);
  KinematicPose(
    double x, double y, double z,
    double vx, double vy, double vz,
    const std_msgs::msg::Header & header);
  KinematicPose(
    const Eigen::Quaterniond & q,
    const Eigen::Vector3d & angular_vel,
    const std_msgs::msg::Header & header);
  KinematicPose(
    const Eigen::EulerAnglesXYZd & rpy_angles,
    const Eigen::Vector3d & angular_vel,
    const std_msgs::msg::Header & header);
  KinematicPose(
    double x, double y, double z,
    double vx, double vy, double vz,
    double heading,
    const std_msgs::msg::Header & header,
    const std::array<double, 36> & cov = std::array<double, 36>{});
  KinematicPose(
    const Eigen::Vector3d & pos,
    const Eigen::Quaterniond & q,
    const Eigen::Vector3d & vel,
    const Eigen::Vector3d & angular_vel,
    const std_msgs::msg::Header & header,
    const std::array<double, 36> & cov = std::array<double, 36>{},
    const std::array<double, 36> & twist_cov = std::array<double, 36>{});
  KinematicPose(Pose p)
  : Pose(std::move(p)) {}

  /* Constructors from ROS messages. */
  KinematicPose(
    const geometry_msgs::msg::PoseStamped & pose_stamped,
    const geometry_msgs::msg::TwistStamped & twist_stamped,
    const std_msgs::msg::Header & header);
  KinematicPose(
    const dua_interfaces::msg::EulerPoseStamped & euler_pose_stamped,
    const geometry_msgs::msg::TwistStamped & twist_stamped,
    const std_msgs::msg::Header & header);
  KinematicPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose_with_cov_stamped,
    const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov_stamped,
    const std_msgs::msg::Header & header);

  /* Destructor. */
  virtual ~KinematicPose() {}

  /* ROS interfaces conversion methods. */
  geometry_msgs::msg::TwistStamped to_twist_stamped() const;
  geometry_msgs::msg::TwistWithCovarianceStamped to_twist_with_covariance_stamped() const;

  /* Getters. */
  inline Eigen::Vector3d get_velocity() const
  {
    return velocity_;
  }
  inline Eigen::Vector3d get_angular_velocity() const
  {
    return angular_velocity_;
  }
  inline std::array<double, 36> get_twist_covariance() const
  {
    return twist_covariance_;
  }

  /* Setters. */
  inline void set_velocity(const Eigen::Vector3d & vel)
  {
    velocity_ = vel;
  }
  inline void set_angular_velocity(const Eigen::Vector3d & angular_vel)
  {
    angular_velocity_ = angular_vel;
  }
  inline void set_twist_covariance(const std::array<double, 36> & twist_cov)
  {
    twist_covariance_ = twist_cov;
  }

  /* Geometric operations. */
  // TODO

  /* Assignment operators. */
  KinematicPose & operator=(const KinematicPose & kp);
  KinematicPose & operator=(KinematicPose && kp);

protected:
  /* Internal data. */
  Eigen::Vector3d velocity_ = {0.0, 0.0, 0.0}; // [m/s]
  Eigen::Vector3d angular_velocity_ = {0.0, 0.0, 0.0}; // [rad/s]
  std::array<double, 36> twist_covariance_{};
};

} // namespace PoseKit

#endif // POSE_KIT__KINEMATIC_POSE_HPP_
