from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("ur_type", default_value="ur10e")
    )
    declared_arguments.append(
        DeclareLaunchArgument("robot_ip", default_value="192.168.56.101", description="IP address by which the robot can be reached.")
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="false")
    )
    declared_arguments.append(
        DeclareLaunchArgument("use_mock_hardware", default_value="true")
    )

    return LaunchDescription(
        declared_arguments
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("tms_robot_hardware"),
                                "launch",
                                "ur_control.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "ur_type": ur_type,
                    "robot_ip": robot_ip,
                    "tf_prefix": [LaunchConfiguration("ur_type"), "_"],
                    "use_mock_hardware": use_mock_hardware,
                    "rviz_config_file": PathJoinSubstitution(
                        [
                            FindPackageShare("tms_robot_hardware"),
                            "rviz",
                            "urdf.rviz",
                        ]
                    ),
                    "description_launchfile": PathJoinSubstitution(
                        [
                            FindPackageShare("tms_robot_hardware"),
                            "launch",
                            "rsp.launch.py",
                        ]
                    ),
                }.items(),
            ),
        ]
    )
