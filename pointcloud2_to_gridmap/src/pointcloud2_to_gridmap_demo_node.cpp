/*
 * image_to_gridmap_demo_node.cpp
 *
 *  Created on: May 04, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, ANYbotics
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "pointcloud2_to_gridmap/PointCloud2ToGridmapDemo.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pointcloud2_to_grid_map::PointCloud2ToGridmapDemo>());
  rclcpp::shutdown();
  return 0;
}
