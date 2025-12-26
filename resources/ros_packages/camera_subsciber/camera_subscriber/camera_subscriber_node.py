#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


class YoloCameraSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_camera_subscriber')

        # QoS for Gazebo camera
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscriber
        self.sub = self.create_subscription(
            Image,
            '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image',
            self.callback,
            camera_qos
        )

        # Publisher: detected video output
        self.detected_pub = self.create_publisher(
            Image,
            'detected_video_out',
            10
        )

        # CV bridge
        self.bridge = CvBridge()

        # Load YOLOv8 model
        self.model = YOLO('/home/ubuntu/.cache/ultralytics/yolov8m.pt')

        self.get_logger().info('YOLOv8 camera subscriber started')

    def callback(self, msg: Image):
        try:
            # Convert ROS Image → OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run YOLOv8 inference
            results = self.model(frame, verbose=False)

            # Draw detections
            annotated_frame = results[0].plot()

            # Log detections
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.model.names[cls_id]
                self.get_logger().info(
                    f'Detected {class_name} with confidence {conf:.2f}'
                )

            # Convert annotated frame → ROS Image
            detected_msg = self.bridge.cv2_to_imgmsg(
                annotated_frame,
                encoding='bgr8'
            )
            detected_msg.header = msg.header

            # Publish detected output
            self.detected_pub.publish(detected_msg)

        except Exception as e:
            self.get_logger().error(f'YOLO callback error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloCameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
