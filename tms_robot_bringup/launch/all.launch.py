from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    bringup_share = get_package_share_directory("tms_robot_bringup")
    moveit_config = (
        MoveItConfigsBuilder(
            "tms_robot_cell",
            package_name="tms_robot_moveit_config",
        ).to_moveit_configs()
    )
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    kinematics_parameters_file = LaunchConfiguration("kinematics_parameters_file")
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10e",
            description="Type/series of used UR robot.",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.0.2",
            description="IP address by which the robot can be reached.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Full experiment launch defaults to real hardware.",
        ),
        DeclareLaunchArgument(
            "kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tms_robot_hardware"),
                    "config",
                    "real_robot_calibration.yaml",
                ]
            ),
            description="UR calibration YAML for the real robot.",
        ),
    ]
    return LaunchDescription(
        declared_arguments
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        bringup_share,
                        "launch",
                        "robot_runtime.launch.py",
                    )
                ),
                launch_arguments={
                    "ur_type": ur_type,
                    "robot_ip": robot_ip,
                    "use_mock_hardware": use_mock_hardware,
                    "kinematics_parameters_file": kinematics_parameters_file,
                    "launch_rviz": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        bringup_share,
                        "launch",
                        "manipulation_stack.launch.py",
                    )
                )
            ),
            Node(
                package="tms_robot_control",
                executable="robot_task_executor",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                ],
            ),
            Node(
                package="tms_robot_ui",
                executable="tms_robot_ui",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        bringup_share,
                        "launch",
                        "uc4_only.launch.py",
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        bringup_share,
                        "launch",
                        "fts_only.launch.py",
                    )
                ),
                launch_arguments={
                    "max_retries": "10",
                    "read_rate": "10",
                    "ftdi_id": "fts",
                    "frame_id": "robotiq_ft_frame_id",
                }.items(),
            ),
            Node(
                package="tms_robot_bringup",
                executable="tcp_pose_publisher",
                output="screen",
            ),
        ]
    )