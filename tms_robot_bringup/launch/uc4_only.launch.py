from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uc4_adc_driver',
            executable='uc4_adc_driver',
            name='uc4_adc_driver',
            parameters=[{
                'port': '/dev/uc4',
                'baudrate': 460800,
                'window_size': 10,  # buffer for moving average smoothing
                'median_size': 3,   # buffer for spike median filter
                'adc_min': 0.1,     # Raw ADC value at closest distance
                'adc_max': 3.2,     # Raw ADC value at furthest distance
                'dist_min': 0.025,  # Minimum distance in meters
                'dist_max': 0.150   # Maximum distance in meters
            }],
            output='screen'
        )
    ])