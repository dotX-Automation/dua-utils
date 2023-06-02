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

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>

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
    double ax, double ay, double az,
    const std_msgs::msg::Header & header);
  DynamicPose(
    const Eigen::Quaterniond & q,
    const Eigen::Vector3d & angular_vel,
    const Eigen::Vector3d & angular_accel,
    const std_msgs::msg::Header & header);
  DynamicPose(
    const Eigen::EulerAnglesXYZd & rpy_angles,
    const Eigen::Vector3d & angular_vel,
    const Eigen::Vector3d & angular_accel,
    const std_msgs::msg::Header & header);
  DynamicPose(
    double x, double y, double z,
    double vx, double vy, double vz,
    double ax, double ay, double az,
    double heading,
    const std_msgs::msg::Header & header,
    const std::array<double, 36> & cov);
  DynamicPose(
    const Eigen::Vector3d & pos,
    const Eigen::Quaterniond & q,
    const Eigen::Vector3d & vel,
    const Eigen::Vector3d & angular_vel,
    const Eigen::Vector3d & accel,
    const Eigen::Vector3d & angular_accel,
    const std_msgs::msg::Header & header,
    const std::array<double, 36> & cov = std::array<double, 36>{},
    const std::array<double, 36> & twist_cov = std::array<double, 36>{},
    const std::array<double, 36> & accel_cov = std::array<double, 36>{});
  DynamicPose(Pose p)
  : KinematicPose(std::move(p)) {}
  DynamicPose(KinematicPose kp)
  : KinematicPose(std::move(kp)) {}

  /* Destructor. */
  virtual ~DynamicPose() {}

  /* ROS interfaces conversion methods. */
  // TODO

  /* Getters. */
  inline Eigen::Vector3d get_acceleration() const
  {
    return acceleration_;
  }
  inline Eigen::Vector3d get_angular_acceleration() const
  {
    return angular_acceleration_;
  }
  inline std::array<double, 36> get_acceleration_covariance() const
  {
    return acceleration_cov_;
  }
  inline uint64_t get_timestamp_ns() const
  {
    return header_.stamp.sec * 1e9 + header_.stamp.nanosec;
  }
  inline uint64_t get_timestamp_us() const
  {
    return header_.stamp.sec * 1e6 + header_.stamp.nanosec / 1e3;
  }
  inline uint64_t get_timestamp_ms() const
  {
    return header_.stamp.sec * 1e3 + header_.stamp.nanosec / 1e6;
  }
  inline uint64_t get_timestamp_s() const
  {
    return header_.stamp.sec;
  }
  inline std::string get_frame_id() const
  {
    return header_.frame_id;
  }

  /* Setters. */
  inline void set_acceleration(const Eigen::Vector3d & accel)
  {
    acceleration_ = accel;
  }
  inline void set_angular_acceleration(const Eigen::Vector3d & angular_accel)
  {
    angular_acceleration_ = angular_accel;
  }
  inline void set_acceleration_covariance(const std::array<double, 36> & accel_cov)
  {
    acceleration_cov_ = accel_cov;
  }
  inline void set_timestamp(const rclcpp::Time & time_point)
  {
    header_.set__stamp(time_point);
  }
  inline void set_timestamp_ns(uint64_t timestamp_ns)
  {
    header_.stamp.set__sec(timestamp_ns / 1e9);
    header_.stamp.set__nanosec(timestamp_ns % static_cast<uint64_t>(1e9));
  }
  inline void set_timestamp_us(uint64_t timestamp_us)
  {
    header_.stamp.set__sec(timestamp_us / 1e6);
    header_.stamp.set__nanosec((timestamp_us % static_cast<uint64_t>(1e6)) * 1e3);
  }
  inline void set_timestamp_ms(uint64_t timestamp_ms)
  {
    header_.stamp.set__sec(timestamp_ms / 1e3);
    header_.stamp.set__nanosec((timestamp_ms % static_cast<uint64_t>(1e3)) * 1e6);
  }
  inline void set_timestamp_s(uint64_t timestamp_s)
  {
    header_.stamp.set__sec(timestamp_s);
    header_.stamp.set__nanosec(0);
  }
  inline void set_frame_id(const std::string & frame_id)
  {
    header_.set__frame_id(frame_id);
  }

  /* Assignment operators. */
  DynamicPose & operator=(const DynamicPose & dp);
  DynamicPose & operator=(DynamicPose && dp);

  /* Geometric operations. */
  // TODO

protected:
  /* Internal data. */
  Eigen::Vector3d acceleration_ = {0.0, 0.0, 0.0}; // [m/s^2]
  Eigen::Vector3d angular_acceleration_ = {0.0, 0.0, 0.0}; // [rad/s^2]
  std::array<double, 36> acceleration_cov_{};
};

}  // namespace PoseKit

#endif  // POSE_KIT__DYNAMIC_POSE_HPP_
