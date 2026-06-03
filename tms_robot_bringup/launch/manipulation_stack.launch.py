from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    moveit_launch = os.path.join(
        get_package_share_directory("tms_robot_moveit_config"),
        "launch",
        "move_group.launch.py",
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_launch)
            )
        ]
    )