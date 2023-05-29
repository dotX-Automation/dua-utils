/**
 * Drone pose library, based on Eigen geometry types.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 13, 2023
 */

#ifndef DRONE_STATE__POSE_HPP_
#define DRONE_STATE__POSE_HPP_

#include "visibility_control.h"

#include <stdexcept>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

#include <dua_interfaces/msg/coordinate_system.hpp>
#include <dua_interfaces/msg/euler_pose_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace dua_interfaces::msg;

namespace DroneState
{

/* Keeps this library coherent with the CoordinateSystem ROS message */
enum class POSE_PUBLIC CoordinateFrame : uint8_t
{
  NWU = CoordinateSystem::COORDINATE_SYSTEM_NWU,
  NED = CoordinateSystem::COORDINATE_SYSTEM_NED
};

/**
 * Represents position and orientation of an autonomous agent.
 */
class POSE_PUBLIC Pose
{
public:
  /* Constructors. */
  Pose();
  Pose(const Pose & p);
  Pose(double x, double y, double z, CoordinateFrame frame);
  Pose(const Eigen::Quaterniond & q, CoordinateFrame frame);
  Pose(const Eigen::EulerAnglesXYZd & rpy_angles, CoordinateFrame frame);
  Pose(double x, double y, double z, double heading, CoordinateFrame frame);
  Pose(
    const Eigen::Vector3d & pos,
    const Eigen::Quaterniond & q,
    const Eigen::EulerAnglesXYZd & rpy_angles,
    CoordinateFrame frame);
  Pose(const EulerPoseStamped & msg);
  // TODO Constructor from PoseStamped
  // TODO Constructor from PoseWithCovarianceStamped

  /* Destructor. */
  virtual ~Pose();

  /* ROS interfaces conversion methods. */
  EulerPoseStamped to_euler_pose_stamped();
  geometry_msgs::msg::PoseStamped to_pose_stamped();
  // TODO to_pose_with_covariance_stamped

  /* Coordinate frame conversions. */
  Pose nwu_to_ned();
  Pose ned_to_nwu();

  /* Getters. */
  CoordinateFrame get_frame() const;
  Eigen::Vector3d get_position() const;
  Eigen::Quaterniond get_attitude() const;
  Eigen::EulerAnglesXYZd get_rpy() const;
  Eigen::AngleAxisd get_rotation() const;
  Eigen::Translation3d get_translation() const;
  Eigen::Transform<double, 3, Eigen::Affine> get_roto_translation() const;

  /* Setters. */
  void set_frame(CoordinateFrame frame);
  void set_position(const Eigen::Vector3d & pos);
  void set_attitude(const Eigen::Quaterniond & q);
  void set_rpy(const Eigen::EulerAnglesXYZd & rpy_angles);

  /* Geometric operations. */
  Pose operator*(const Pose & p) const;

  /* Assignment operators. */
  Pose & operator=(const Pose & p);
  Pose & operator=(Pose && p);

protected:
  /* Internal data. */
  CoordinateFrame frame_ = CoordinateFrame::NWU;
  Eigen::Vector3d position_ = {0.0, 0.0, 0.0}; // [m]
  Eigen::Quaterniond attitude_ = Eigen::Quaterniond::Identity();
  Eigen::EulerAnglesXYZd rpy_ = {0.0, 0.0, 0.0}; // [rad] in [-PI +PI]
};

}  // namespace DroneState

#endif  // DRONE_STATE__POSE_HPP_
