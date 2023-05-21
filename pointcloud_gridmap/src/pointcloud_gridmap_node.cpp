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

#include <pointcloud_gridmap/pointcloud_gridmap_node.hpp>

#include <chrono>

#include <grid_map_ros/GridMapRosConverter.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>

#include <rmoss_util/url_resolver.hpp>

namespace pointcloud_gridmap
{

PointcloudGridmapNode::PointcloudGridmapNode(const rclcpp::NodeOptions & options)
: Node("pointcloud_gridmap_node", options)
{
  // Log level
  bool is_verbosity_debug = this->declare_parameter("set_verbosity_to_debug", false);
  if (is_verbosity_debug) {
    RCLCPP_INFO(this->get_logger(), "Setting verbosity to debug");
    auto ret = rcutils_logging_set_logger_level(
      this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to set verbosity to debug: %s",
        rcutils_get_error_string().str);
      rcutils_reset_error();
    }
  }

  map_frame_id_ = this->declare_parameter("map_frame_id", "map");
  map_layer_name_ = this->declare_parameter("map_layer_name", std::string("elevation"));

  auto grid_map_topic_name = this->declare_parameter("grid_map_topic_name", "grid_map");
  auto point_cloud_topic_name = this->declare_parameter("point_cloud_topic_name", "point_cloud");
  auto parameter_path = this->declare_parameter("parameter_path", "");
  parameter_path = rmoss_util::URLResolver::get_resolved_path(parameter_path);
  if (parameter_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to resolve parameter path");
  }
  RCLCPP_INFO(this->get_logger(), "parameter_path: %s", parameter_path.c_str());
  RCLCPP_INFO(this->get_logger(), "map_frame_id: %s", map_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "map_layer_name: %s", map_layer_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "grid_map_topic_name: %s", grid_map_topic_name.c_str());
  RCLCPP_INFO(this->get_logger(), "point_cloud_topic_name: %s", point_cloud_topic_name.c_str());

  grid_map_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(grid_map_topic_name, 1);

  grid_map_pcl_loader_ = std::make_unique<grid_map::GridMapPclLoader>(this->get_logger());
  grid_map_pcl_loader_->loadParameters(parameter_path);

  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(10.0));
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  pointcloud_sub_ =
    std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
    this,
    point_cloud_topic_name);
  tf_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
    *pointcloud_sub_, *tf_buffer_, map_frame_id_, 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  tf_filter_->registerCallback(&PointcloudGridmapNode::pointcloud_callback, this);

  map_ = grid_map::GridMap();
  map_.setFrameId(map_frame_id_);
  map_.add(map_layer_name_, 0.0);

  RCLCPP_INFO(this->get_logger(), "pointcloud_gridmap_node initialized");
}

void PointcloudGridmapNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  tf2::Transform transform;
  if (msg->header.frame_id == map_frame_id_) {
    transform.setIdentity();
  } else {
    try {
      auto transform_stamped = tf_buffer_->lookupTransform(
        map_frame_id_, msg->header.frame_id,
        msg->header.stamp);
      tf2::fromMsg(transform_stamped.transform, transform);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Error while transforming: %s", ex.what());
      return;
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  pcl_ros::transformPointCloud(*cloud, *cloud, transform);

  grid_map_pcl_loader_->setInputCloud(cloud);
  grid_map_pcl_loader_->preProcessInputCloud();
  grid_map_pcl_loader_->initializeGridMapGeometryFromInputCloud();
  grid_map_pcl_loader_->addLayerFromInputCloud(map_layer_name_);
  auto gridmap = grid_map_pcl_loader_->getGridMap();
  gridmap.setFrameId(map_frame_id_);
  map_[map_layer_name_] = map_[map_layer_name_] + gridmap[map_layer_name_];

  auto gridmap_msg = grid_map::GridMapRosConverter::toMessage(map_);
  gridmap_msg->header.stamp = msg->header.stamp;
  grid_map_publisher_->publish(std::move(gridmap_msg));
}

}  // namespace pointcloud_gridmap

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_gridmap::PointcloudGridmapNode)
