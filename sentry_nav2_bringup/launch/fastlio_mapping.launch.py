from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('sentry_nav2_bringup')
    # sentry description
    sentry_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sentry_description'), 'launch', 'sentry_urdf.launch.py')])
    )
    
    # mid360
    mid360_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360_launch.py')])
    )


    fast_lio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('fast_lio'),'launch','mapping.launch.py')])
    )


    return LaunchDescription([
        sentry_description,
        # mid360_node,
        fast_lio_node,
    ])
