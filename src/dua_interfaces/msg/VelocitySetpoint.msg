# 3D velocity setpoint.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# January 31, 2023

# ROS 2 message header (with timestamp from ROS 2 clock)
std_msgs/Header header

# Indicates whether setpoints are in NED or ENU
bool ned

# X velocity setpoint (meters/sec)
float32 vx_sp

# Y velocity setpoint (meters/sec)
float32 vy_sp

# Z velocity setpoint (meters/sec)
float32 vz_sp

# Yaw velocity setpoint [-PI +PI] (radians/sec)
float32 vyaw_sp
