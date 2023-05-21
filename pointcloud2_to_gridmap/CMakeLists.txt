cmake_minimum_required(VERSION 3.8)
project(pointcloud2_to_gridmap)
cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_pcl REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(filters REQUIRED)


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


add_executable(pointcloud2_to_gridmap_demo 
  src/pointcloud2_to_gridmap_demo_node.cpp
  src/PointCloud2ToGridmapDemo.cpp)

set(dependencies
  grid_map_msgs
  grid_map_ros
  grid_map_pcl
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rclcpp
  tf2_eigen
  PCL
  pcl_ros
)

ament_target_dependencies(pointcloud2_to_gridmap_demo ${dependencies})

include_directories(
  include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
pointcloud2_to_gridmap_demo
  DESTINATION lib/${PROJECT_NAME}

)


ament_package()
