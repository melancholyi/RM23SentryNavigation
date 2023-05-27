// Copyright 2023 Yunlong Feng
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef POINTCLOUD_GRIDMAP__POINTCLOUD_GRIDMAP_NODE_HPP_
#define POINTCLOUD_GRIDMAP__POINTCLOUD_GRIDMAP_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_pcl/grid_map_pcl.hpp>

namespace pointcloud_gridmap
{

class PointcloudGridmapNode : public rclcpp::Node
{
public:
  explicit PointcloudGridmapNode(const rclcpp::NodeOptions & options);

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  std::string map_frame_id_;
  std::string map_layer_name_;
  grid_map::GridMap map_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> tf_filter_;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> pointcloud_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_publisher_;

  std::unique_ptr<grid_map::GridMapPclLoader> grid_map_pcl_loader_;
};

}  // namespace pointcloud_gridmap

#endif  // POINTCLOUD_GRIDMAP__POINTCLOUD_GRIDMAP_NODE_HPP_
