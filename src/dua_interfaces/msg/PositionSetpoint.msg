# 3D position setpoint.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# January 31, 2023

# ROS 2 message header (with timestamp from ROS 2 clock)
std_msgs/Header header

# Indicates the coordinate system used.
dua_interfaces/CoordinateSystem coordinate_system

# Position setpoint (meters)
geometry_msgs/Point position_sp

# Yaw angle setpoint [-PI +PI] (radians)
float64 yaw_sp
