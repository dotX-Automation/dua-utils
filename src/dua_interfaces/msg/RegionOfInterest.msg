# Region of interest representation.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# June 3, 2023

# Type of the ROI
uint8 BOX=0
uint8 CIRCLE=1
uint8 CYLINDER=2
uint8 type

# Box ROI data: four corners
geometry_msgs/PointStamped[4] box_corners

# Circle ROI data: center and radius
# The Z axis of the pose is the normal to the circle plane
geometry_msgs/PoseStamped circle_center
float64 circle_radius

# Cylinder ROI data: four reference points:
# - center
# - on x axis, along the circle
# - on y axis, along the circle
# - on z axis, on the top of the cylinder
geometry_msgs/PointStamped[4] cylinder_points
