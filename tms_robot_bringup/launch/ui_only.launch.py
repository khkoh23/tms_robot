from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tms_robot_ui',
            executable='tms_robot_ui',
            name='tms_robot_ui',
            output='screen'
        )
    ])
