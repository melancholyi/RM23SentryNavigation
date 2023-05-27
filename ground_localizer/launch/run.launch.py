from launch import LaunchDescription
from launch_ros.actions import Node
import os
def generate_launch_description():
    ns = os.environ.get('QOMOLO_ROBOT_ID')
    return LaunchDescription([
        Node(
            package='pcl_cloud',
            executable='pcl_cloud_node',
            namespace= ns,
            parameters=[{"fence_x": 10000.0,
                         "fence_x2": 332.5,
                         "thre_z_min" : 0.0,
                         "thre_z_max" : 0.3}]
            # name='landmark_localizer',
        )
    ])