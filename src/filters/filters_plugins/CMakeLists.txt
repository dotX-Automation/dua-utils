cmake_minimum_required(VERSION 3.8)
project(filters_plugins)

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
find_package(filters_base REQUIRED)
find_package(pluginlib REQUIRED)

# Median Filter plugin
add_library(medianfilter SHARED
  src/median_filter/median_filter.cpp)
target_compile_definitions(medianfilter PRIVATE "FILTERS_BUILDING_LIBRARY")
target_include_directories(medianfilter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(medianfilter
  filters_base
  pluginlib)

# Export Plugin Declaration file
pluginlib_export_plugin_description_file(filters_base plugins.xml)

# Libraries installation
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS medianfilter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_libraries(
  medianfilter)

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
