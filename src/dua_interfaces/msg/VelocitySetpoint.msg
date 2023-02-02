# 3D velocity setpoint.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# January 31, 2023

# ROS 2 message header (with timestamp from ROS 2 clock)
std_msgs/Header header

# Indicates the coordinate system used.
dua_interfaces/CoordinateSystem coordinate_system

# Velocity setpoint (meters/sec)
geometry_msgs/Vector3 v_sp

# Yaw velocity setpoint [-PI +PI] (radians/sec)
float64 vyaw_sp
