import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Find the grid_map_demos package share directory
    grid_map_demos_dir = get_package_share_directory('pointcloud2_to_gridmap')

    # Declare launch configuration variables that can access the launch arguments values
    param_file = LaunchConfiguration('param_file')
    rviz_config_file = LaunchConfiguration('rviz_config')

    # Declare launch arguments
    declare_param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=os.path.join(
            grid_map_demos_dir, 'config', 'pointcloud2_to_gridmap_demo.yaml'),
        description='Full path to the config file to use')

    pointcloud2_to_gridmap_demo_node = Node(
        package='pointcloud2_to_gridmap',
        executable='pointcloud2_to_gridmap_demo',
        name='pointcloud2_to_gridmap',
        parameters=[param_file,
                    {
                    "config_file_path" : os.path.join(grid_map_demos_dir, 'config', 'pcl_grid_config.yaml')
                    }
                ],
        arguments=['--ros-args', '--log-level', 'ERROR'],
        output='log',
        respawn=True
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch arguments to the launch description
    ld.add_action(declare_param_file_cmd)

    # Add node actions to the launch description
    ld.add_action(pointcloud2_to_gridmap_demo_node)

    return ld
