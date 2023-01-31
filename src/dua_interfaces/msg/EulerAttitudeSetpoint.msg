# 3D attitude setpoint.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# January 31, 2023

# ROS 2 message header (with timestamp from ROS 2 clock)
std_msgs/Header header

# Indicates whether setpoints are in NED or ENU
bool ned

# Roll angle setpoint [-PI +PI] (radians)
float32 roll_sp

# Pitch angle setpoint [-PI +PI] (radians)
float32 pitch_sp

# Yaw angle setpoint [-PI +PI] (radians)
float32 yaw_sp

# Thrust setpoint [0 1]
float32 thrust_sp
