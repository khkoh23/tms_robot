from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    bringup_share = get_package_share_directory('tms_robot_bringup')
    moveit_config = (MoveItConfigsBuilder("tms_robot_cell", package_name="tms_robot_moveit_config").to_moveit_configs())
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("use_mock_hardware", default_value="true"))

    return LaunchDescription(declared_arguments + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'robot_runtime.launch.py')
            ),
            launch_arguments={"use_mock_hardware": use_mock_hardware}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'manipulation_stack.launch.py')
            )
        ),

        Node(
            package='tms_robot_control',
            executable='robot_task_executor',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
            ]
        ),

        Node(
            package='tms_robot_ui',
            executable='tms_robot_ui',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
            ]
        ),

    ])
