# Pose sample with RPY angles.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# January 31, 2023

# ROS 2 message header (with timestamp from ROS 2 clock)
std_msgs/Header header

# Indicates whether coordinates are in NED or ENU
bool ned

# X coordinate (meters)
float32 x

# Y coordinate (meters)
float32 y

# Z coordinate (meters)
float32 z

# Linear velocity along X (meters/sec)
float32 vx

# Linear velocity along Y (meters/sec)
float32 vy

# Linear velocity along Z (meters/sec)
float32 vz

# Attitude quaternion (w, x, y, z)
float32[4] q

# Roll angle [-PI +PI] (radians)
float32 roll

# Pitch angle [-PI +PI] (radians)
float32 pitch

# Yaw angle [-PI +PI] (radians)
float32 yaw
