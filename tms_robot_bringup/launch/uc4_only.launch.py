from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("port", default_value="/dev/uc4"),
        DeclareLaunchArgument("baudrate", default_value="460800"),
        DeclareLaunchArgument("window_size", default_value="10"),
        DeclareLaunchArgument("median_size", default_value="3"),
        DeclareLaunchArgument("adc_min", default_value="0.1"),
        DeclareLaunchArgument("adc_max", default_value="3.2"),
        DeclareLaunchArgument("dist_min", default_value="0.025"),
        DeclareLaunchArgument("dist_max", default_value="0.150"),
    ]
    return LaunchDescription(
        declared_arguments
        + [
            Node(
                package="uc4_adc_driver",
                executable="uc4_adc_driver",
                parameters=[
                    {
                        "port": LaunchConfiguration("port"),
                        "baudrate": LaunchConfiguration("baudrate"),
                        "window_size": LaunchConfiguration("window_size"),
                        "median_size": LaunchConfiguration("median_size"),
                        "adc_min": LaunchConfiguration("adc_min"),
                        "adc_max": LaunchConfiguration("adc_max"),
                        "dist_min": LaunchConfiguration("dist_min"),
                        "dist_max": LaunchConfiguration("dist_max"),
                    }
                ],
                output="screen",
            )
        ]
    )