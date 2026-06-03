from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "tms_robot_cell",
            package_name="tms_robot_moveit_config",
        ).to_moveit_configs()
    )
    return LaunchDescription(
        [
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
            )
        ]
    )