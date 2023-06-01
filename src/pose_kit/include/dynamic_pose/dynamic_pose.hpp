/**
 * Dynamic pose library, based on Eigen geometry types.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 14, 2023
 */

#ifndef POSE_KIT__DYNAMIC_POSE_HPP_
#define POSE_KIT__DYNAMIC_POSE_HPP_

#include "visibility_control.h"

#include <stdexcept>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

#include <dua_interfaces/msg/euler_pose_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <kinematic_pose/kinematic_pose.hpp>

namespace PoseKit
{

/**
 * Represents position, orientation, velocity, and acceleration of an autonomous agent.
 */
class DYNAMIC_POSE_PUBLIC DynamicPose : public KinematicPose
{
public:
  /* Constructors. */
  DynamicPose() {}
  DynamicPose(const DynamicPose & dp);
  DynamicPose(
    double x, double y, double z,
    double vx, double vy, double vz,
    double ax, double ay, double az);
  DynamicPose(
    const Eigen::Quaterniond & q,
    const Eigen::Vector3d & angular_vel,
    const Eigen::Vector3d & angular_accel);
  DynamicPose(
    const Eigen::EulerAnglesXYZd & rpy_angles,
    const Eigen::Vector3d & angular_vel,
    const Eigen::Vector3d & angular_accel);
  DynamicPose(
    double x, double y, double z,
    double vx, double vy, double vz,
    double ax, double ay, double az,
    double heading,
    const std::array<double, 36> & cov);
  DynamicPose(
    const Eigen::Vector3d & pos,
    const Eigen::Quaterniond & q,
    const Eigen::EulerAnglesXYZd & rpy_angles,
    const Eigen::Vector3d & vel,
    const Eigen::Vector3d & angular_vel,
    const Eigen::Vector3d & accel,
    const Eigen::Vector3d & angular_accel,
    const std::array<double, 36> & cov = std::array<double, 36>{},
    const std::array<double, 36> & twist_cov = std::array<double, 36>{},
    const std::array<double, 36> & accel_cov = std::array<double, 36>{});
  DynamicPose(const EulerPoseStamped & msg);
  DynamicPose(Pose p)
  : KinematicPose(std::move(p)) {}
  DynamicPose(KinematicPose kp)
  : KinematicPose(std::move(kp)) {}

  /* Destructor. */
  virtual ~DynamicPose() {}

  /* Getters. */
  Eigen::Vector3d get_acceleration() const;
  Eigen::Vector3d get_angular_acceleration() const;
  std::array<double, 36> get_acceleration_covariance() const;

  /* Setters. */
  void set_acceleration(const Eigen::Vector3d & accel);
  void set_angular_acceleration(const Eigen::Vector3d & angular_accel);
  void set_acceleration_covariance(const std::array<double, 36> & accel_cov);

  /* Assignment operators. */
  DynamicPose & operator=(const DynamicPose & dp);
  DynamicPose & operator=(DynamicPose && dp);

  /* Geometric operations. */
  DynamicPose operator*(const DynamicPose & dp) const;

protected:
  /* Internal data. */
  Eigen::Vector3d acceleration_ = {0.0, 0.0, 0.0}; // [m/s^2]
  Eigen::Vector3d angular_acceleration_ = {0.0, 0.0, 0.0}; // [rad/s^2]
  std::array<double, 36> acceleration_cov_{};
};

}  // namespace PoseKit

#endif  // POSE_KIT__DYNAMIC_POSE_HPP_
