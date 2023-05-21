from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    respawn_delay = 1
    cmdvel2chassis_node = Node(
        package="cmd_chassis",
        executable="cmdvel_chassis",
        name='cmd_chassis',
        respawn=True, respawn_delay=respawn_delay,
    )

    launch_description = LaunchDescription([cmdvel2chassis_node])

    return launch_description
