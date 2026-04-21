from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_pkg = 'tms_robot_moveit_config'
    moveit_launch = os.path.join(get_package_share_directory(moveit_pkg), 'launch', 'move_group.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch)
        )
    ])