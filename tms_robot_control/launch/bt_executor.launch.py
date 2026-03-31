from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    tree_path = PathJoinSubstitution ( [get_package_share_directory('tms_robot_control'), 'tree', 'tms_bt.xml'])
    bt_node = Node(
        package='tms_robot_control',
        executable='tms_robot_executor',
        output='screen',
        parameters=[{'tree_xml_file': tree_path}],
    )
    return LaunchDescription([bt_node])