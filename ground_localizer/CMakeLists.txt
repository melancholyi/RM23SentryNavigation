cmake_minimum_required(VERSION 3.3)
project(pcl_cloud)

add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Release)
add_definitions(${PCL_DEFINITIONS})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(Ceres_DIR
"/opt/third_party/ceres/lib/cmake/Ceres")
set(CERES_INCLUDE_DIRS
"/opt/third_party/ceres/include/")

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)


set(PCL_DEFINITIONS "-DFLANN_STATIC-DDISABLE_ENSENSO-DDISABLE_DAVIDSDK-DDISABLE_DSSDK-DDISABLE_PCAP-DDISABLE_PNG-DDISABLE_LIBUSB_1_0-Dqh_QHpointer-D-ffloat-store")

include_directories(
include 
${PCL_INCLUDE_DIRS}
${CERES_INCLUDE_DIRS}
${geometry_msgs_INCLUDE_DIRS}
${OCTOMAP_INCLUDE_DIRS})


set(DEPS
  PCL
  Ceres
  geometry_msgs
  message_filters
  pcl_conversions
  octomap
  octomap_msgs
  rclcpp
  tf2
  tf2_ros
  sensor_msgs
  std_msgs
  nav_msgs
  tf2_geometry_msgs
  Boost
)

link_directories(${OCTOMAP_LIBRARY_DIRS})

ament_export_include_directories(include)


add_executable(${PROJECT_NAME}_node
  src/conversions.cpp
  src/transforms.cpp  
  src/octomap_projection.cpp
  src/main.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node
  ${DEPS}
)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
