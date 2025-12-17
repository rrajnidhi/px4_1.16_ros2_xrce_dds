#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        #QoS for high-frequency image topics from Gazebo
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Best effort for live streaming
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub = self.create_subscription(
            Image,
            '/camera',      # Topic name
            self.callback,
            camera_qos
        )

        self.get_logger().info('Camera subscriber started, waiting for images...')

    def callback(self, msg: Image):
        # Log basic info about the received image
        stamp = msg.header.stamp
        self.get_logger().info(f'Received image seq: {msg.header.stamp.sec}.{msg.header.stamp.nanosec} '
                               f'height: {msg.height}, width: {msg.width}, encoding: {msg.encoding}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
