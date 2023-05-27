#include <iostream>                 //标准输入输出头文件
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"   
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <pcl/io/pcd_io.h>         //I/O操作头文件
#include <pcl/point_types.h>        //点类型定义头文件
#include <pcl/registration/icp.h>   //ICP配准类相关头文件
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件
#include <pcl/filters/voxel_grid.h>  //体素滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>    //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件
#include <pcl/visualization/cloud_viewer.h>

#include<octomap_msgs/msg/octomap.hpp>
#include<octomap_msgs/srv/bounding_box_query.hpp>
#include<octomap_msgs/srv/get_octomap.hpp>
#include<octomap_msgs/conversions.h>
#include<octomap/octomap.h>
#include<octomap/OcTreeKey.h>
#include<octomap_server2/transforms.hpp>
#include<octomap_server2/conversions.h>

namespace octomap_project
{
  class OctomapProject:public rclcpp::Node
  {
  public:
    explicit OctomapProject();
    ~OctomapProject();

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gridmap_pub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr fullMapPub_;

    bool InitParams();
    void OctomapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void PassThroughFilter(const bool& flag_in);
    void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid& msg);
    void SetOctoMapTopicMsg(geometry_msgs::msg::Vector3 & sensor_tf, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, octomap_msgs::msg::Octomap& msg);
    void EachGridmap();
    void EachOctomap();
    std::string world_frame_id_;
    std::string robot_frame_id_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud_;
    std::shared_ptr<tf2_ros::Buffer> buffers_;//share_ptr 使用 reset赋值,单纯的ptr使用=new赋值
    // static std::function<void(std::shared_future<geometry_msgs::msg::TransformStamped>)> callback_(const std::shared_future<geometry_msgs::msg::TransformStamped> & ttf);
    // std::function<void(const tf2_ros::TransformStampedFuture)> callback_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    float thre_x_min_,thre_y_min_,thre_z_min_;
    float thre_x_max_,thre_y_max_,thre_z_max_;
    nav_msgs::msg::OccupancyGrid map_topic_msg_;
    float map_resolution_;
    geometry_msgs::msg::TransformStamped echo_transform;
    octomap::OcTree * m_octree_;
    octomap_msgs::msg::Octomap octomap_;
    octomap::OcTreeKey m_updateBBXMin,m_updateBBXMax;

    

    inline static void updateMinKey(const octomap::OcTreeKey& in,
                                  octomap::OcTreeKey& min) {
      for (unsigned i = 0; i < 3; ++i) {
          min[i] = std::min(in[i], min[i]);
      }
    };

    inline static void updateMaxKey(const octomap::OcTreeKey& in,
                                  octomap::OcTreeKey& max) {
      for (unsigned i = 0; i < 3; ++i) {
          max[i] = std::max(in[i], max[i]);
      }
    };
  };
  
}