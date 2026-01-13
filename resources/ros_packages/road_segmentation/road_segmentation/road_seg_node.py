import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import torch
import os
import cv2

from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO


class RoadSegmentationNode(Node):

    def __init__(self):
        super().__init__('road_segmentation_node')

        self.bridge = CvBridge()

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/world/sonoma_raceway/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image',
            self.image_callback,
            10
        )

        # Publisher
        self.segmented_publisher = self.create_publisher(
            Image,
            '/road_segmentation/segmented_output',
            10
        )

        self.get_logger().info('Road segmentation node started')

        # Load YOLOv8 segmentation model
        pkg_path = get_package_share_directory('road_segmentation')
        model_path = os.path.join(pkg_path, 'models', 'best.pt')

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = YOLO(model_path)
        self.model.to(self.device)

        # Parameters
        self.ROAD_CLASS_ID = 0
        self.MIN_ROAD_AREA = 5000
        self.ALPHA = 0.5

    # ------------------------------------------------
    # STEP 1: ROAD SCORING FUNCTION (GLOBAL SELECTION)
    # ------------------------------------------------
    def score_road_component(self, stats, centroid, image_shape):
        h, w = image_shape

        area = stats[cv2.CC_STAT_AREA]

        cx, cy = centroid
        dist_to_center = np.sqrt((cx - w / 2) ** 2 + (cy - h / 2) ** 2)

        # Bigger + more central road wins
        score = area * 1.0 - dist_to_center * 2.0
        return score

    # ------------------------------------------------
    # IMAGE CALLBACK
    # ------------------------------------------------
    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape

        # YOLO inference (NO FILE WRITES)
        results = self.model.predict(
            source=frame,
            imgsz=640,
            conf=0.25,
            device=self.device,
            verbose=False,
            save=False,
            save_txt=False,
            save_conf=False,
            project="/tmp",
            name="road_seg",
            exist_ok=True
        )

        best_score = -np.inf
        best_road_mask = None

        if results and results[0].masks is not None:
            masks = results[0].masks.data.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy().astype(int)

            for mask, cls in zip(masks, classes):
                if cls != self.ROAD_CLASS_ID:
                    continue

                mask_resized = cv2.resize(mask, (w, h))
                binary_mask = (mask_resized > 0.5).astype(np.uint8)

                if np.sum(binary_mask) < self.MIN_ROAD_AREA:
                    continue

                num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
                    binary_mask
                )

                for i in range(1, num_labels):  # skip background
                    area = stats[i, cv2.CC_STAT_AREA]
                    if area < self.MIN_ROAD_AREA:
                        continue

                    score = self.score_road_component(
                        stats[i],
                        centroids[i],
                        (h, w)
                    )

                    if score > best_score:
                        best_score = score
                        best_road_mask = (labels == i).astype(np.uint8) * 255

        # Use ONLY the best road
        if best_road_mask is None:
            road_mask = np.zeros((h, w), dtype=np.uint8)
        else:
            road_mask = best_road_mask

        # ------------------------------------------------
        # OVERLAY
        # ------------------------------------------------
        overlay_color = np.array([0, 255, 0], dtype=np.uint8)

        colored_overlay = np.zeros_like(frame)
        colored_overlay[road_mask > 0] = overlay_color

        segmented_frame = cv2.addWeighted(
            colored_overlay,
            self.ALPHA,
            frame,
            1 - self.ALPHA,
            0
        )

        segmented_msg = self.bridge.cv2_to_imgmsg(
            segmented_frame,
            encoding='bgr8'
        )
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
