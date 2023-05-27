# include "octomap_projection.h"


namespace octomap_project
{
  OctomapProject::OctomapProject(): Node("octomap_projection")
  {
    InitParams();
  }
  
  OctomapProject::~OctomapProject()
  {

  }


  void OctomapProject::OctomapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
  }

  void OctomapProject::EachGridmap()
  {
   PassThroughFilter(false);
   SetMapTopicMsg(cloud_after_PassThrough_, map_topic_msg_);
  }

  void OctomapProject::EachOctomap()
  {
    // PassThroughFilter(false);
    SetOctoMapTopicMsg(echo_transform.transform.translation,cloud_after_PassThrough_,octomap_);
  }
  

// std::function<void(std::shared_future<geometry_msgs::msg::TransformStamped>)> OctomapProject::callback_(const std::shared_future<geometry_msgs::msg::TransformStamped> & ttf)
  // {
  //   try {
  //     RCLCPP_INFO(this->get_logger(), "Returned from get() with transform");
  //   } catch (tf2::TimeoutException & e) {
  //     RCLCPP_INFO(this->get_logger(), "Transform timed out");
  //   }
  // }


  void OctomapProject::PassThroughFilter(const bool& flag_in)
  {
    // 初始化,并通过tf2_ros::TransformListener完成对tf2_ros::Buffer类的初始化和构造，并订阅相应tf消息
    buffers_.reset(new tf2_ros::Buffer(this->get_clock()));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffers_.get());
    
    
    /*方法一：直通滤波器对点云进行处理。*/
    cloud_after_PassThrough_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(pcd_cloud_);//输入点云
    passthrough.setFilterFieldName("x");//对x轴进行操作
    passthrough.setFilterLimits(thre_x_min_, thre_x_max_);//设置直通滤波器操作范围
    passthrough.setFilterLimitsNegative(flag_in);//true表示保留范围外，false表示保留范围内
    passthrough.filter(*cloud_after_PassThrough_);//执行滤波，过滤结果保存在 cloud_after_PassThrough_

    passthrough.setFilterFieldName("y");//对y轴进行操作
    passthrough.setFilterLimits(thre_y_min_, thre_y_max_);//设置直通滤波器操作范围
    passthrough.filter(*cloud_after_PassThrough_);

    passthrough.setFilterFieldName("z");//对z轴进行操作
    passthrough.setFilterLimits(thre_z_min_, thre_z_max_);//设置直通滤波器操作范围
    passthrough.filter(*cloud_after_PassThrough_);

    std::cout << "直通滤波后点云数据点数：" << cloud_after_PassThrough_->points.size() << std::endl;
    try
    {
      // buffers_->waitForTransform(world_frame_id_,robot_frame_id_,tf2::TimePoint(),rclcpp::Duration(0.2),callback_);
      echo_transform = buffers_->lookupTransform(world_frame_id_,robot_frame_id_,tf2::TimePoint());
      Eigen::Matrix4f matrix_transform = pcl_ros::transformAsMatrix(echo_transform);
      // Eigen::Matrix4f matrix_transform = Eigen::Matrix4f::Zero();
      // Eigen::Vector3f vec_transform;
      // //pcl_ros::transformAsMatrix(sensorToBaseTf, sensorToBase);
      // vec_transform << echo_transform.transform.translation.x,echo_transform.transform.translation.y,echo_transform.transform.translation.z;
      // tf2Scalar roll, pitch, yaw;
      // tf2::Quaternion qua_transform;
      // geometry_msgs::msg::Quaternion quat_msg = echo_transform.transform.rotation;
      // tf2::fromMsg(quat_msg, qua_transform);
      // tf2::Matrix3x3(qua_transform).getRPY(roll, pitch, yaw); // rpy得是double

      // Eigen::Affine3f affine = pcl::getTransformation(vec_transform[0], vec_transform[1], vec_transform[2], roll, pitch, yaw);
      // // transform pointcloud from sensor frame to fixed robot frame
      // matrix_transform = affine.matrix();
      pcl::transformPointCloud(*cloud_after_PassThrough_.get(),*cloud_after_PassThrough_.get(),matrix_transform);
    }
    catch(const tf2::TransformException& ex)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform error of sensor data: " << ex.what() << ", quitting callback");
    }

  }

  void OctomapProject::SetOctoMapTopicMsg(geometry_msgs::msg::Vector3 & sensor_tf, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, octomap_msgs::msg::Octomap& msg)
  {
    m_octree_ = new octomap::OcTree(0.05f);
    // m_octree_ = std::make_shared<octomap::OcTree>(0.05f);
    m_octree_->setProbHit(0.7);
    m_octree_->setProbMiss(0.4);
    m_octree_->setClampingThresMin(0.12);
    m_octree_->setClampingThresMax(0.97);
    sensor_tf.x =1.0;
    sensor_tf.y =1.0;
    sensor_tf.z =1.0;

    // m_updateBBXMin[0] =m_octree_->coordToKey(1.0);
    // m_updateBBXMin[1] =m_octree_->coordToKey(1.0);
    // m_updateBBXMin[2] =m_octree_->coordToKey(1.0);

    // m_updateBBXMax[0] =m_octree_->coordToKey(1.0);
    // m_updateBBXMax[1] =m_octree_->coordToKey(1.0);
    // m_updateBBXMax[2] =m_octree_->coordToKey(1.0);
    octomap::point3d octomap_point = octomap::pointTfToOctomap(sensor_tf);


    if(!m_octree_->coordToKeyChecked(octomap_point,m_updateBBXMin) || !m_octree_->coordToKeyChecked(octomap_point,m_updateBBXMax))//将三维坐标转为3D octreekey,并进行边界检查
    {
      RCLCPP_WARN(this->get_logger(),
                        "Could not generate Key for origin");
    }
    octomap::KeySet free_set, occupied_cells;
    octomap::KeyRay octkey_ray;
    unsigned char* colors = new unsigned char[3];

    for(auto it = cloud->begin(); it!=cloud->end(); it++)
    {
      octomap::point3d point(it->x,it->y,it->z);
      if(m_octree_->computeRayKeys(octomap_point,point,octkey_ray))//computeRayKeys函数的参数origin（光束起点）和参数end（传感器末端击中点）
      {
        free_set.insert(octkey_ray.begin(),octkey_ray.end());
      }
      //更新最小和最大的值
      // octomap::OcTreeKey endKey;
      // if (m_octree_->coordToKeyChecked(point, endKey)) {
      //     updateMinKey(endKey, m_updateBBXMin);
      //     updateMaxKey(endKey, m_updateBBXMax);
      // } else {
      //     RCLCPP_ERROR(this->get_logger(),
      //                   "Could not generate Key for endpoint");
      // }

      // std::cout<<"11"<<std::endl;
      octomap::OcTreeKey key;
      if (m_octree_->coordToKeyChecked(point, key)) {
          occupied_cells.insert(key);

          updateMinKey(key, m_updateBBXMin);
          updateMaxKey(key, m_updateBBXMax);
      }
    }
    // m_octree_->averageNodeColor(it->x, it->y, it->z,
    //                             it->r, it->g, it->b);
    // std::cout<<"111"<<std::endl;
    // mark free cells only if not seen occupied in this cloud
    for(auto it = free_set.begin(), end=free_set.end();
        it!= end; ++it){
        if (occupied_cells.find(*it) == occupied_cells.end()){
            m_octree_->updateNode(*it, false);
        }
    }
// std::cout<<"222"<<std::endl;
    // now mark all occupied cells:
    for (auto it = occupied_cells.begin(),
              end=occupied_cells.end(); it!= end; it++) {
        // 将点云里的点插入到octomap中
        m_octree_->updateNode(*it, true);
    }
    // std::cout<<"444"<<std::endl;
    // for(auto it = m_octree_->begin(), end = m_octree_->end(); it != end; ++it)
    // {
    //   if(m_octree_->isNodeOccupied(*it))
    //   {
    //     double x = it.getX();
    //     double y = it.getY();
    //     double z = it.getZ();
    //   }

    // }
    // std::cout<<"33333333"<<std::endl;
    m_octree_->updateInnerOccupancy();
    m_octree_->writeBinary("sample.bt");
    std::cout<<"done"<<std::endl;

  }


  void OctomapProject::SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid& msg)
  {
    msg.header.stamp = builtin_interfaces::msg::Time(this->now());;
    msg.header.frame_id = "map";
    msg.info.map_load_time = builtin_interfaces::msg::Time(this->now());;
    msg.info.resolution = map_resolution_;
  
    double x_min, x_max, y_min, y_max;
    double z_max_grey_rate = 0.05;
    double z_min_grey_rate = 0.95;
    double k_line = (z_max_grey_rate - z_min_grey_rate) / (thre_z_max_ - thre_z_min_);
    double b_line = (thre_z_max_ * z_min_grey_rate - thre_z_min_ * z_max_grey_rate) / (thre_z_max_ - thre_z_min_);
  
    if(cloud->points.empty())
    {
      RCLCPP_WARN(this->get_logger(),"pcd is empty!\n");
      return;
    }
  
    for(int i = 0; i < cloud->points.size() - 1; i++)
    {
      if(i == 0)
      {
        x_min = x_max = cloud->points[i].x;
        y_min = y_max = cloud->points[i].y;
      }
  
      double x = cloud->points[i].x;
      double y = cloud->points[i].y;
  
      if(x < x_min) x_min = x;
      if(x > x_max) x_max = x;
  
      if(y < y_min) y_min = y;
      if(y > y_max) y_max = y;
    }
  
    msg.info.origin.position.x = x_min;
    msg.info.origin.position.y = y_min;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;
  
    msg.info.width = int((x_max - x_min) / map_resolution_);//可以根据x_max和x_min来设置地图动态大小
    msg.info.height = int((y_max - y_min) / map_resolution_);
  
    msg.data.resize(msg.info.width * msg.info.height);
    msg.data.assign(msg.info.width * msg.info.height, 0);
  
    RCLCPP_INFO(this->get_logger(), "data size = %d\n", msg.data.size());
  
    for(int iter = 0; iter < cloud->points.size(); iter++)
    {
      int i = int((cloud->points[iter].x - x_min) / map_resolution_);
      if(i < 0 || i >= msg.info.width) continue;
  
      int j = int((cloud->points[iter].y - y_min) / map_resolution_);
      if(j < 0 || j >= msg.info.height - 1) continue;
      msg.data[i + j * msg.info.width] = 100;//int(255 * (cloud->points[iter].z * k_line + b_line)) % 255;
    }
  }

  bool OctomapProject::InitParams()
  {
    world_frame_id_ = "map";
    robot_frame_id_ = "robot";
    this->declare_parameter<float>("thre_x_min", -0.0);
    this->get_parameter_or<float>("thre_x_min", thre_x_min_, 0.0);
    this->declare_parameter<float>("thre_x_max", 2.0);
    this->get_parameter_or<float>("thre_x_max", thre_x_max_, 2.0);

    this->declare_parameter<float>("thre_y_min", 0.0);
    this->get_parameter_or<float>("thre_y_min", thre_y_min_, 0.0);
    this->declare_parameter<float>("thre_y_max", 2.0);
    this->get_parameter_or<float>("thre_y_max", thre_y_max_, 2.0);

    this->declare_parameter<float>("thre_z_min", 0.0);
    this->get_parameter_or<float>("thre_z_min", thre_z_min_, 0.0);
    this->declare_parameter<float>("thre_z_max", 2.0);
    this->get_parameter_or<float>("thre_z_max", thre_z_max_, 2.0);

    this->declare_parameter<float>("map_resolution", 0.05);
    this->get_parameter_or<float>("map_resolution", map_resolution_, 0.05);

    gridmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

    fullMapPub_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap_full", 10);

    // pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //   "point", 1, std::bind(&OctomapProject::OctomapCallback, this, std::placeholders::_1));
    std::string pcd_file = "/home/scurm/ros_ws/src/ground_localizer/dat/test.pcd";
    pcd_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file,*pcd_cloud_) == -1)
    {
     PCL_ERROR ("Couldn't read file: %s \n", pcd_file.c_str());
     return -1;
    }

    //lambda https://blog.csdn.net/weixin_44363885/article/details/93076574
    // callback_ = [this, logger = this->get_logger()](
    //             const std::shared_future<geometry_msgs::msg::TransformStamped> & tf) {
    //   try {
    //     RCLCPP_INFO(logger, "Returned from get() with transform");
    //   } catch (tf2::TimeoutException & e) {
    //     RCLCPP_INFO(logger, "Transform timed out");
    //   }
    // };

    std::cout << "初始点云数据点数：" << pcd_cloud_->points.size() << std::endl;
    EachGridmap();
    EachOctomap();
    
    rclcpp::WallRate  loop_rate(5000);
    while (rclcpp::ok())
    {
    gridmap_pub_->publish(map_topic_msg_);
      if (octomap_msgs::fullMapToMsg(*m_octree_, octomap_)) {
          fullMapPub_->publish(octomap_);
          RCLCPP_INFO(this->get_logger(),
                        "Start OctoMap");
      } else {            
          RCLCPP_ERROR(this->get_logger(),
                        "Error serializing OctoMap");
      }
    loop_rate.sleep();
    }


    return 0;
    // pcl::visualization::CloudViewer viewer("cloud viewer");
    // viewer.showCloud(pcd_cloud_);
    // while (!viewer.wasStopped())
    // {
    // }




  }
}


