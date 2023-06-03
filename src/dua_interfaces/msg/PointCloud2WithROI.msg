# Point cloud with a region of interest.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# June 3, 2023

# Indicates if the cloud has already been culled to the ROI
bool culled

# The region of interest
dua_interfaces/RegionOfInterest roi

# The point cloud
sensor_msgs/PointCloud2 cloud
