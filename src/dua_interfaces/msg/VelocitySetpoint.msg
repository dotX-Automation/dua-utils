# 3D velocity setpoint.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# January 31, 2023

# ROS 2 message header (with timestamp from ROS 2 clock)
std_msgs/Header header

# Velocity setpoint (meters/sec)
geometry_msgs/Vector3 v_sp

# Yaw velocity setpoint [-PI +PI] (radians/sec)
float64 vyaw_sp
