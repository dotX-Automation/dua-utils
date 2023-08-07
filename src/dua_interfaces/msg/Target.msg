# Target detected by a sensor or subsystem.
#
# Roberto Masocco <robmasocco@gmail.com>
# Lorenzo Bianchi <lnz.bnc@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# August 7, 2023

# Contains the detection timestamp and frame ID of the sensor/subsystem that detected the target
std_msgs/Header header

# Identifier of the target
dua_interfaces/TargetID target_id

# Pose of the target in the sensor/subsystem frame
geometry_msgs/Pose pose
