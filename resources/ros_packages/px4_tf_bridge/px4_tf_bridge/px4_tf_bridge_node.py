import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class PX4TFBridge(Node):

    def __init__(self):
        super().__init__('px4_tf_bridge')

        # PX4-compatible QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriber
        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile
        )

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info('PX4 TF Bridge started (QoS + float-safe)')

    def odom_callback(self, msg):
        t = TransformStamped()

        # --------------------
        # Header
        # --------------------
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        # --------------------
        # NED → ENU POSITION
        # PX4: x=N, y=E, z=D
        # ROS: x=E, y=N, z=U
        # --------------------
        t.transform.translation.x = float(msg.position[1])
        t.transform.translation.y = float(msg.position[0])
        t.transform.translation.z = float(-msg.position[2])

        # --------------------
        # NED → ENU ORIENTATION
        # PX4 quaternion: [w, x, y, z]
        # --------------------
        t.transform.rotation.x = float(msg.q[2])
        t.transform.rotation.y = float(msg.q[1])
        t.transform.rotation.z = float(-msg.q[3])
        t.transform.rotation.w = float(msg.q[0])

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PX4TFBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
