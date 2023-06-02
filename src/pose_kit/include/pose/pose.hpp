/**
 * Pose library, based on Eigen geometry types.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 13, 2023
 */

#ifndef POSE_KIT__POSE_HPP_
#define POSE_KIT__POSE_HPP_

#include "visibility_control.h"

#include <stdexcept>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

#include <dua_interfaces/msg/euler_pose_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace dua_interfaces::msg;

namespace PoseKit
{

/**
 * Represents position and orientation of an autonomous agent.
 */
class POSE_PUBLIC Pose
{
public:
  /* Constructors. */
  Pose();
  Pose(const Pose & p);
  Pose(double x, double y, double z);
  Pose(const Eigen::Quaterniond & q);
  Pose(const Eigen::EulerAnglesXYZd & rpy_angles);
  Pose(double x, double y, double z, double heading,
    const std::array<double, 36> & cov = std::array<double, 36>{});
  Pose(
    const Eigen::Vector3d & pos,
    const Eigen::Quaterniond & q,
    const std::array<double, 36> & cov = std::array<double, 36>{});

  /* Constructors from ROS messages. */
  Pose(const EulerPoseStamped & msg);
  // TODO

  /* Destructor. */
  virtual ~Pose();

  /* ROS interfaces conversion methods. */
  EulerPoseStamped to_euler_pose_stamped();
  geometry_msgs::msg::PoseStamped to_pose_stamped();
  // TODO

  /* Getters. */
  Eigen::Vector3d get_position() const;
  Eigen::Quaterniond get_attitude() const;
  Eigen::EulerAnglesXYZd get_rpy() const;
  Eigen::AngleAxisd get_rotation() const;
  Eigen::Translation3d get_translation() const;
  Eigen::Transform<double, 3, Eigen::Affine> get_roto_translation() const;
  std::array<double, 36> get_pose_covariance() const;

  /* Setters. */
  void set_position(const Eigen::Vector3d & pos);
  void set_attitude(const Eigen::Quaterniond & q);
  void set_rpy(const Eigen::EulerAnglesXYZd & rpy_angles);
  void set_pose_covariance(const std::array<double, 36> & cov);

  /* Geometric operations. */
  Pose operator*(const Pose & p) const;

  /* Assignment operators. */
  Pose & operator=(const Pose & p);
  Pose & operator=(Pose && p);

protected:
  /* Internal data. */
  Eigen::Vector3d position_ = {0.0, 0.0, 0.0}; // [m]
  Eigen::Quaterniond attitude_ = Eigen::Quaterniond::Identity();
  Eigen::EulerAnglesXYZd rpy_ = {0.0, 0.0, 0.0}; // [rad] in [-PI +PI]
  std::array<double, 36> pose_covariance_{};
};

}  // namespace PoseKit

#endif  // POSE_KIT__POSE_HPP_
