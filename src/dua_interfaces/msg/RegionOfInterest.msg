# Region of interest representation.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# June 3, 2023

# Type of the ROI
uint8 BOX=0
uint8 type

# Box ROI data: four corners
geometry_msgs/PointStamped[4] box_corners
