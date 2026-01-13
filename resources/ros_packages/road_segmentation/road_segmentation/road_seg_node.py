import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import torch
import os

from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
import cv2


class RoadSegmentationNode(Node):

    def __init__(self):
        super().__init__('road_segmentation_node')

        self.bridge = CvBridge()

        # Subscriber to camera
        self.subscription = self.create_subscription(Image,'/world/sonoma_raceway/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image',self.image_callback,10)

        # Publisher for segmented output (frame + semi-transparent road overlay)
        self.segmented_publisher = self.create_publisher(Image,'/road_segmentation/segmented_output',10)

        self.get_logger().info('Road segmentation node started')

        # Load YOLOv8 segmentation model
        pkg_path = get_package_share_directory('road_segmentation')
        model_path = os.path.join(pkg_path, 'models', 'best.pt')
        self.get_logger().info(f'Loading YOLOv8 model from: {model_path}')

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = YOLO(model_path)
        self.model.to(self.device)

        # Set road class ID (adjust for your model)
        self.ROAD_CLASS_ID = 0

        # Minimum mask area to consider a valid road segment
        self.MIN_ROAD_AREA = 5000  # pixels, tune based on image resolution

        # Overlay transparency factor (0.0=transparent, 1.0=solid)
        self.ALPHA = 0.5

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape

        # YOLO inference
        results = self.model.predict(
            source=frame,
            imgsz=640,
            conf=0.25,
            device=self.device,
            verbose=False,
            save=False,
            save_txt=False,
            save_conf=False,
            project="/tmp",      # writable directory
            name="road_seg",     # avoids default "predict"
            exist_ok=True
            )


        # Initialize mask
        road_mask = np.zeros((h, w), dtype=np.uint8)

        if results and results[0].masks is not None:
            masks = results[0].masks.data.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy().astype(int)

            for mask, cls in zip(masks, classes):
                if cls == self.ROAD_CLASS_ID:
                    mask_resized = cv2.resize(mask, (w, h))

                    # Skip small road segments
                    mask_area = np.sum(mask_resized > 0.5)
                    if mask_area < self.MIN_ROAD_AREA:
                        continue

                    # Keep only the largest connected component
                    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
                        (mask_resized > 0.5).astype(np.uint8)
                    )
                    if num_labels <= 1:
                        continue  # nothing detected

                    largest_idx = np.argmax(stats[1:, cv2.CC_STAT_AREA]) + 1  # skip background
                    main_road_mask = (labels == largest_idx).astype(np.uint8) * 255

                    # Add to final road mask
                    road_mask = cv2.bitwise_or(road_mask, main_road_mask)

        # Create overlay: green road mask on original frame with transparency
        overlay_color = np.array([0, 255, 0], dtype=np.uint8)  # green

        # Convert road_mask to 3 channels
        colored_overlay = np.zeros_like(frame)
        colored_overlay[road_mask > 0] = overlay_color


        # Blend original frame and colored overlay
        segmented_frame = cv2.addWeighted(colored_overlay, self.ALPHA, frame, 1 - self.ALPHA, 0)

        # Publish the segmented output
        segmented_msg = self.bridge.cv2_to_imgmsg(segmented_frame, encoding='bgr8')
        segmented_msg.header = msg.header
        self.segmented_publisher.publish(segmented_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RoadSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
