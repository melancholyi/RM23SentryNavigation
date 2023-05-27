cmake_minimum_required(VERSION 3.8)
project(octomap_generate)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Default Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# -------- Find thirdparty library ------- #
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_pcl REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rmoss_util REQUIRED)
find_package(rmoss_interfaces REQUIRED)
find_package(PCL REQUIRED)

set(thirdparty_libs
  ${PCL_LIBRARIES}
)

set(dependencies
  rclcpp
  rclcpp_components
  message_filters
  tf2
  tf2_ros
  grid_map_core
  grid_map_msgs
  grid_map_pcl
  grid_map_ros
  pcl_conversions
  pcl_ros
  sensor_msgs
  rmoss_util
  rmoss_interfaces
)

# -------- Build library ------- #
add_library(${PROJECT_NAME} SHARED
  src/pointcloud_gridmap_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  ${thirdparty_libs}
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "pointcloud_gridmap::PointcloudGridmapNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# ---------------- Install --------------- #
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install  directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ----------------- Test ----------------- #
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
