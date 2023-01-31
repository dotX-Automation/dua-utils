# 3D position setpoint.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# January 31, 2023

# ROS 2 message header (with timestamp from ROS 2 clock)
std_msgs/Header header

# Indicates whether setpoints are in NED or ENU
bool ned

# X position setpoint (meters)
float32 x_sp

# Y position setpoint (meters)
float32 y_sp

# Z position setpoint (meters)
float32 z_sp

# Yaw angle setpoint [-PI +PI] (radians)
float32 yaw_sp
