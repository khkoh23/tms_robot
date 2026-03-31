from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    move_tcp_z_server_node = Node(
        package='tms_robot_control',
        executable='move_tcp_z_server',
        output='screen',
    )
   
    return LaunchDescription([
        move_tcp_z_server_node,
    ])