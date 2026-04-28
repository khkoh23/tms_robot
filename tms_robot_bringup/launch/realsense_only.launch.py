from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='realsense2_camera',
            namespace='camera',
            executable='realsense2_camera_node',
            name='camera',
            parameters=[{
                'rgb_camera.color_profile': '640x480x30',
                # 'depth_module.depth_profile': '640x480x15',
                'enable_metadata': 'false',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                # ... other parameters
            }],
            remappings=[
                ('/camera/camera/color/image_raw', '/camera/ai_results'),
            ]
        )
    ])
