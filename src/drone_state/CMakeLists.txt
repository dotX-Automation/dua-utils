cmake_minimum_required(VERSION 3.8)
project(drone_state)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fvisibility=hidden)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(dua_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# Pose library configuration
add_library(pose SHARED
  src/pose/pose.cpp)
target_compile_features(pose PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(pose PRIVATE "POSE_BUILDING_LIBRARY")
target_include_directories(pose PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pose
  Eigen3::Eigen)
ament_target_dependencies(
  pose
  dua_interfaces
  geometry_msgs)

# Kinematic Pose library configuration
add_library(kinematicpose SHARED
  src/kinematic_pose/kinematic_pose.cpp)
target_compile_features(kinematicpose PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(kinematicpose PRIVATE "KINEMATIC_POSE_BUILDING_LIBRARY")
target_include_directories(kinematicpose PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(kinematicpose
  pose
  Eigen3::Eigen)
ament_target_dependencies(
  kinematicpose
  dua_interfaces
  geometry_msgs)

# Libraries installation
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS pose kinematicpose
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_libraries(
  pose
  kinematicpose)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export headers and targets for downstream ament packages
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
