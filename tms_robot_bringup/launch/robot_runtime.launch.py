from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
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
            "launch_rviz",
            default_value="false",
            description="Launch standalone RViz from UR driver launch.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to states.",
        ),
        DeclareLaunchArgument(
            "kinematics_parameters_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("tms_robot_hardware"),
                    "config",
                    "default_ur10e_calibration.yaml",
                ]
            ),
            description="UR calibration YAML used by the robot description.",
        ),
    ]
    return LaunchDescription(
        declared_arguments
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("tms_robot_hardware"),
                            "launch",
                            "ur_control.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "ur_type": ur_type,
                    "robot_ip": robot_ip,
                    "tf_prefix": [ur_type, "_"],
                    "use_mock_hardware": use_mock_hardware,
                    "launch_rviz": launch_rviz,
                    "kinematics_parameters_file": kinematics_parameters_file,
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