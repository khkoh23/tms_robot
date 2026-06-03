from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("max_retries", default_value="100"),
        DeclareLaunchArgument("read_rate", default_value="10"),
        DeclareLaunchArgument("ftdi_id", default_value="fts"),
        DeclareLaunchArgument("frame_id", default_value="robotiq_ft_frame_id"),
    ]
    return LaunchDescription(
        declared_arguments
        + [
            Node(
                package="robotiq_ft_sensor_hardware",
                executable="robotiq_ft_sensor_standalone_node",
                namespace=LaunchConfiguration("namespace"),
                parameters=[
                    {
                        "max_retries": LaunchConfiguration("max_retries"),
                        "read_rate": LaunchConfiguration("read_rate"),
                        "ftdi_id": LaunchConfiguration("ftdi_id"),
                        "frame_id": LaunchConfiguration("frame_id"),
                    }
                ],
                output="screen",
            )
        ]
    )