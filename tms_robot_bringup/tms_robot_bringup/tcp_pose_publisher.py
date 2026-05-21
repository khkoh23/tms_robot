import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
import tf2_ros

class TcpPosePublisher(Node):
    def __init__(self):
        super().__init__('tcp_pose_publisher')
        custom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        # self.publisher_ = self.create_publisher(PoseStamped, '/ur10e/tcp_pose', custom_qos)
        self.publisher_ = self.create_publisher(PoseStamped, '/ur10e/tcp_pose', qos_profile_sensor_data)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # 50 Hz timer (1 / 50 = 0.02 seconds)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'ur10e_base_link', 
                'ur10e_tool0', 
                now
            )
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'ur10e_base_link'            
            msg.pose.position.x = trans.transform.translation.x
            msg.pose.position.y = trans.transform.translation.y
            msg.pose.position.z = trans.transform.translation.z
            msg.pose.orientation = trans.transform.rotation
            self.publisher_.publish(msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Fail silently during startup while waiting for TF tree to link up
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TcpPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()