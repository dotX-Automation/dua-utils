# 3D attitude setpoint.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# January 31, 2023

# ROS 2 message header (with timestamp from ROS 2 clock)
std_msgs/Header header

# Roll angle setpoint [-PI +PI] (radians)
float64 roll_sp

# Pitch angle setpoint [-PI +PI] (radians)
float64 pitch_sp

# Yaw angle setpoint [-PI +PI] (radians)
float64 yaw_sp

# Thrust setpoint [-1 +1]
float64 thrust_sp
