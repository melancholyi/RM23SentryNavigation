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

    # wit_imu
    wit_imu_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('wit_imu_driver'), 'launch', 'wit901.launch.py')])
    )

    # lio-sam localization
    lio_sam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('lio_sam'), 'launch', 'run.launch.py')])
    )

    fast_lio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('fast_lio'),'launch','mapping.launch.py')])
    )

    # gridmap
    pc2_gridmap_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pointcloud2_to_gridmap'), 'launch', 'pointcloud2_to_gridmap_demo_launch.py')]),
    )

    # navigation2
    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'navigation2.launch.py')]),
    )

    # cmd2chassis
    cmd2chassis_node = Node(
        package="cmd_chassis",
        executable="cmdvel_chassis",
        name='cmd_chassis'
    )

    # communication
    serialcom_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rm_base'), 'launch', 'rm_base.launch.py')])
    )

    return LaunchDescription([
        sentry_description,
        mid360_node,
        fast_lio_node,
        pc2_gridmap_node,
        cmd2chassis_node,
        serialcom_node,
        # wit_imu_node,
        # lio_sam_node,
        
        # TimerAction(
        #     period=5.,
        #     actions=[nav2_node]
        # ),
        # nav2_node,
    ])
