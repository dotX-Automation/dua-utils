/**
 * Kinematic drone pose library, based on Eigen geometry types.
 *
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * April 14, 2023
 */

#ifndef DRONE_STATE__KINEMATIC_POSE_HPP_
#define DRONE_STATE__KINEMATIC_POSE_HPP_

#include "visibility_control.h"

#include <stdexcept>

#include <Eigen/Geometry>
#include <unsupported/Eigen/EulerAngles>

#include <dua_interfaces/msg/coordinate_system.hpp>
#include <dua_interfaces/msg/euler_pose_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pose/pose.hpp>

namespace DroneState
{

/**
 * Represents position, orientation, and velocity of an autonomous agent.
 */
class KINEMATIC_POSE_EXPORT KinematicPose : public Pose
{
public:
  /* Constructors */
  KinematicPose();
  KinematicPose(const KinematicPose & kp);
  KinematicPose(
    double x, double y, double z,
    double vx, double vy, double vz,
    CoordinateFrame frame);
  KinematicPose(
    const Eigen::Quaterniond & q,
    const Eigen::Vector3d & angular_vel,
    CoordinateFrame frame);
  KinematicPose(
    const Eigen::EulerAnglesXYZd & rpy_angles,
    const Eigen::Vector3d & angular_vel,
    CoordinateFrame frame);
  KinematicPose(
    double x, double y, double z,
    double vx, double vy, double vz,
    double heading,
    CoordinateFrame frame);
  KinematicPose(
    const Eigen::Vector3d & pos,
    const Eigen::Quaterniond & q,
    const Eigen::EulerAnglesXYZd & rpy_angles,
    const Eigen::Vector3d & vel,
    const Eigen::Vector3d & angular_vel,
    CoordinateFrame frame);
  KinematicPose(const EulerPoseStamped & msg);

  /* Destructor */
  virtual ~KinematicPose();

  /* Coordinate frame conversions */
  KinematicPose nwu_to_ned();
  KinematicPose ned_to_nwu();

  /* Getters */
  Eigen::Vector3d get_velocity() const;
  Eigen::Vector3d get_angular_velocity() const;

  /* Setters */
  void set_velocity(const Eigen::Vector3d & vel);
  void set_angular_velocity(const Eigen::Vector3d & angular_vel);

  /* Assignment operators */
  KinematicPose & operator=(const KinematicPose & kp);
  KinematicPose & operator=(KinematicPose && kp);

protected:
  /* Internal data */
  Eigen::Vector3d velocity_ = {0.0, 0.0, 0.0}; // [m/s]
  Eigen::Vector3d angular_velocity_ = {0.0, 0.0, 0.0}; // [rad/s]
};

}  // namespace DroneState

#endif  // DRONE_STATE__KINEMATIC_POSE_HPP_
