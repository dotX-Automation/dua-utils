/**
 * Dynamic drone pose library, based on Eigen geometry types.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 14, 2023
 */

#ifndef DRONE_STATE__DYNAMIC_POSE_HPP_
#define DRONE_STATE__DYNAMIC_POSE_HPP_

#include "visibility_control.h"

#include <stdexcept>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

#include <dua_interfaces/msg/coordinate_system.hpp>
#include <dua_interfaces/msg/euler_pose_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <kinematic_pose/kinematic_pose.hpp>

namespace DroneState
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
    double ax, double ay, double az,
    CoordinateFrame frame);
  DynamicPose(
    const Eigen::Quaterniond & q,
    const Eigen::Vector3d & angular_vel,
    const Eigen::Vector3d & angular_accel,
    CoordinateFrame frame);
  DynamicPose(
    const Eigen::EulerAnglesXYZd & rpy_angles,
    const Eigen::Vector3d & angular_vel,
    const Eigen::Vector3d & angular_accel,
    CoordinateFrame frame);
  DynamicPose(
    double x, double y, double z,
    double vx, double vy, double vz,
    double ax, double ay, double az,
    double heading,
    CoordinateFrame frame);
  DynamicPose(
    const Eigen::Vector3d & pos,
    const Eigen::Quaterniond & q,
    const Eigen::EulerAnglesXYZd & rpy_angles,
    const Eigen::Vector3d & vel,
    const Eigen::Vector3d & angular_vel,
    const Eigen::Vector3d & accel,
    const Eigen::Vector3d & angular_accel,
    CoordinateFrame frame);
  DynamicPose(const EulerPoseStamped & msg);
  DynamicPose(Pose p)
  : KinematicPose(std::move(p)) {}
  DynamicPose(KinematicPose kp)
  : KinematicPose(std::move(kp)) {}

  /* Destructor. */
  virtual ~DynamicPose() {}

  /* Coordinate frame conversions. */
  DynamicPose nwu_to_ned();
  DynamicPose ned_to_nwu();

  /* Getters. */
  Eigen::Vector3d get_acceleration() const;
  Eigen::Vector3d get_angular_acceleration() const;

  /* Setters. */
  void set_acceleration(const Eigen::Vector3d & accel);
  void set_angular_acceleration(const Eigen::Vector3d & angular_accel);

  /* Assignment operators. */
  DynamicPose & operator=(const DynamicPose & dp);
  DynamicPose & operator=(DynamicPose && dp);

  /* Geometric operations. */
  DynamicPose operator*(const DynamicPose & dp) const;

protected:
  /* Internal data. */
  Eigen::Vector3d acceleration_ = {0.0, 0.0, 0.0}; // [m/s^2]
  Eigen::Vector3d angular_acceleration_ = {0.0, 0.0, 0.0}; // [rad/s^2]
};

}  // namespace DroneState

#endif  // DRONE_STATE__DYNAMIC_POSE_HPP_
