from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tms_robot_control',
            executable='robot_task_executor',
            name='robot_task_executor',
            output='screen'
        )
    ])
