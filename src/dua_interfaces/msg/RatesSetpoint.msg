# Attitude rate setpoint.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# June 11, 2023

# Header
std_msgs/Header header

# Roll rate setpoint [-PI PI] (radians)
float64 roll_rate

# Pitch rate setpoint [-PI PI] (radians)
float64 pitch_rate

# Yaw rate setpoint [-PI PI] (radians)
float64 yaw_rate

# Thrust setpoint [0 1] (normalized)
float64 thrust
