# Copyright 2023 Yunlong Feng
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os.path

from ament_index_python.packages import get_package_share_directory

from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition

from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    package_path = get_package_share_directory('pointcloud_gridmap')
    default_config_file_path = os.path.join(
        package_path, 'config', 'ros_params.yaml')

    # Declare the launch config
    namespace = LaunchConfiguration('namespace')
    config_file = LaunchConfiguration('config_file')
    container_name = LaunchConfiguration('container_name')
    use_eternal_container = LaunchConfiguration('use_eternal_container')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Namespace')
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='octomap_server_container', description='Container name')
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value=default_config_file_path, description='Config file path')
    declare_use_eternal_container_cmd = DeclareLaunchArgument(
        'use_eternal_container', default_value='false', description='Use eternal container')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use sim time')

    # Create container node
    container_node = Node(
        name=container_name,
        package='rclcpp_components',
        executable='component_container_isolated',
        output='log',
        condition=UnlessCondition(use_eternal_container),
    )

    # Create robot base node
    load_gridmap = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            # ComposableNode(
            #     package='pointcloud_gridmap',
            #     plugin='pointcloud_gridmap::PointcloudGridmapNode',
            #     name='pointcloud_gridmap_node',
            #     parameters=[config_file,
            #                 {'use_sim_time': use_sim_time}],
            #     namespace=namespace
            # )
            ComposableNode(
                package='octomap_server',
                plugin='octomap_server::OctomapServer',
                name='octomap_server',
                parameters=[{
                    'resolution': 0.05,
                    'frame_id': 'odom',
                    'sensor_model.max_range': 100.,
                    'sensor_model.min_range': 0.35,
                    'occupancy_min_z': -0.3,
                    'occupancy_max_z': 0.5
                }],
                remappings=[('cloud_in', 'livox/lidar')]
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_use_eternal_container_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add nodes
    ld.add_action(container_node)
    ld.add_action(load_gridmap)

    return ld
