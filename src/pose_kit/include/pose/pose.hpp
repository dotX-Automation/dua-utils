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
#include <std_msgs/msg/header.hpp>

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
  Pose(
    double x, double y, double z,
    const std_msgs::msg::Header & header);
  Pose(
    const Eigen::Quaterniond & q,
    const std_msgs::msg::Header & header);
  Pose(
    const Eigen::EulerAnglesXYZd & rpy_angles,
    const std_msgs::msg::Header & header);
  Pose(
    double x, double y, double z, double heading,
    const std_msgs::msg::Header & header,
    const std::array<double, 36> & cov = std::array<double, 36>{});
  Pose(
    const Eigen::Vector3d & pos,
    const Eigen::Quaterniond & q,
    const std_msgs::msg::Header & header,
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
  inline Eigen::Vector3d get_position() const
  {
    return position_;
  }
  inline Eigen::Quaterniond get_attitude() const
  {
    return attitude_;
  }
  inline Eigen::EulerAnglesXYZd get_rpy() const
  {
    return rpy_;
  }
  inline Eigen::AngleAxisd get_rotation() const
  {
    return Eigen::AngleAxisd(attitude_);
  }
  inline Eigen::Translation3d get_translation() const
  {
    return Eigen::Translation3d(position_);
  }
  inline Eigen::Isometry3d get_isometry() const
  {
    Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
    isometry.rotate(this->get_rotation());
    isometry.pretranslate(this->get_translation().vector());
    return isometry;
  }
  inline std::array<double, 36> get_pose_covariance() const
  {
    return pose_covariance_;
  }
  inline std_msgs::msg::Header get_header() const
  {
    return header_;
  }

  /* Setters. */
  inline void set_position(const Eigen::Vector3d & pos)
  {
    position_ = pos;
  }
  inline void set_attitude(const Eigen::Quaterniond & q)
  {
    attitude_ = q;
    rpy_ = Eigen::EulerAnglesXYZd(q);
  }
  inline void set_rpy(const Eigen::EulerAnglesXYZd & rpy_angles)
  {
    rpy_ = rpy_angles;
    attitude_ = Eigen::Quaterniond(rpy_angles);
  }
  inline void set_pose_covariance(const std::array<double, 36> & cov)
  {
    pose_covariance_ = cov;
  }
  inline void set_header(const std_msgs::msg::Header & header)
  {
    header_ = header;
  }

  /* Geometric operations. */
  // TODO

  /* Assignment operators. */
  Pose & operator=(const Pose & p);
  Pose & operator=(Pose && p);

protected:
  /* Internal data. */
  Eigen::Vector3d position_ = {0.0, 0.0, 0.0}; // [m]
  Eigen::Quaterniond attitude_ = Eigen::Quaterniond::Identity();
  Eigen::EulerAnglesXYZd rpy_ = {0.0, 0.0, 0.0}; // [rad] in [-PI +PI]
  std::array<double, 36> pose_covariance_{};
  std_msgs::msg::Header header_{};
};

}  // namespace PoseKit

#endif  // POSE_KIT__POSE_HPP_
