#include "octomap_projection.h"
#ifndef PCL_NO_PRECOMPILE
int main(int argc, char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<octomap_project::OctomapProject>());
  rclcpp::shutdown();
  return 0;
}

#endif // PCL_NO_PRECOMPILE