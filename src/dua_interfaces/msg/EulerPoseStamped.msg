# Pose sample with RPY angles.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# January 31, 2023

# ROS 2 message header (with timestamp from ROS 2 clock)
std_msgs/Header header

# Indicates the coordinate system used.
dua_interfaces/CoordinateSystem coordinate_system

# Pose, intended as a 3D point in space plus an orientation quaternion
geometry_msgs/Pose pose

# Roll angle [-PI +PI] (radians)
float64 roll

# Pitch angle [-PI +PI] (radians)
float64 pitch

# Yaw angle [-PI +PI] (radians)
float64 yaw
