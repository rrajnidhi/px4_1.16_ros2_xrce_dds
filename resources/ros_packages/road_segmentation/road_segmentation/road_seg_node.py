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

        self.subscription = self.create_subscription(
            Image,
            '/world/sonoma_raceway/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image',
            self.image_callback,
            10
        )

        self.segmented_publisher = self.create_publisher(
            Image,
            '/road_segmentation/segmented_output',
            10
        )

        # ── Centerline & visualization settings ──
        self.row_step            = 12              #how many pixels apart the code checks horizontal rows in the image to find the center of the road.
        self.min_road_width      = 40
        self.look_ahead_fraction = 0.4              # 0.0 to ~0.4 = look more ahead and 0.6 to 1.0 = look more behind or where it came from
        self.centerline_color    = (50, 180, 255)   # BGR
        self.target_point_color  = (0, 0, 255)

        self.get_logger().info('Road segmentation node started')
        self.get_logger().info(
            f"Centerline: step={self.row_step}px | min_width={self.min_road_width}px | "
            f"look_ahead={self.look_ahead_fraction:.2f}"
        )

        # YOLO setup
        pkg_path = get_package_share_directory('road_segmentation')
        model_path = os.path.join(pkg_path, 'models', 'best.pt')
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = YOLO(model_path)
        self.model.to(self.device)

        self.ROAD_CLASS_ID = 0
        self.MIN_ROAD_AREA = 5000
        self.ALPHA = 0.5

    def score_road_component(self, stats, centroid, image_shape):
        h, w = image_shape
        area = stats[cv2.CC_STAT_AREA]
        cx, cy = centroid
        dist_to_center = np.sqrt((cx - w / 2) ** 2 + (cy - h / 2) ** 2)
        return area * 1.0 - dist_to_center * 2.0

    #     SIMPLE, INDEPENDENT CENTERLINE FUNCTIONS

    def extract_centerline_points(self, road_mask: np.ndarray, h: int, w: int) -> list[tuple[int, int]]:
        """
        This is the function you can replace later with any other algorithm.
        Currently: horizontal slicing + mean x per row.
        """
        points = []

        for y in range(0, h, self.row_step):
            row = road_mask[y, :]
            road_x = np.where(row > 127)[0]
            if len(road_x) >= self.min_road_width:
                x_center = int(np.mean(road_x))
                points.append((x_center, y))

        return points

    def select_target_point(self, centerline_points: list[tuple[int, int]], image_width: int) -> tuple[int, int] | None:
        """
        Pick one point to aim for , a poistion setpoint.
        """
        if not centerline_points:
            return None

        idx = int(len(centerline_points) * self.look_ahead_fraction)
        idx = min(max(idx, 0), len(centerline_points) - 1)
        return centerline_points[idx]

    def draw_centerline_and_target(self,frame: np.ndarray,centerline_points: list[tuple[int, int]],target_point: tuple[int, int] | None) -> None:
        """
        Just draws — easy to modify style, thickness, colors, etc.
        """
        h, w, _ = frame.shape

        if len(centerline_points) >= 2:
            pts = np.array(centerline_points, dtype=np.int32)
            cv2.polylines(frame,[pts],isClosed=False,color=self.centerline_color,thickness=3,lineType=cv2.LINE_AA)

        if target_point is not None:
            cv2.circle(frame, target_point, 12, self.target_point_color, -1)
            cv2.circle(frame, target_point, 4, (255, 255, 255), -1)
        
        #center of frame
        cv2.circle(frame, (w // 2, h // 2), 6, (180, 180, 60), 2)

    def process_centerline(self, segmented_frame: np.ndarray, road_mask: np.ndarray):
        """
        High-level function that calls the three steps above.
        Returns target and points so you can use them for control later.
        """
        h, w, _ = segmented_frame.shape

        centerline_points = self.extract_centerline_points(road_mask, h, w)
        if not centerline_points:
            return None, []

        target_point = self.select_target_point(centerline_points, w)
        self.draw_centerline_and_target(segmented_frame, centerline_points, target_point)

        return target_point, centerline_points

    #IMAGE CALLBACK
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape

        # YOLO + best mask selection (unchanged)
        results = self.model.predict(
            source=frame,
            imgsz=640,
            conf=0.5,
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

                num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_mask)

                for i in range(1, num_labels):
                    area = stats[i, cv2.CC_STAT_AREA]
                    if area < self.MIN_ROAD_AREA:
                        continue

                    score = self.score_road_component(stats[i], centroids[i], (h, w))

                    if score > best_score:
                        best_score = score
                        best_road_mask = (labels == i).astype(np.uint8) * 255

        road_mask = best_road_mask if best_road_mask is not None else np.zeros((h, w), dtype=np.uint8)

        # Green overlay
        overlay_color = np.array([0, 255, 0], dtype=np.uint8)
        colored_overlay = np.zeros_like(frame)
        colored_overlay[road_mask > 0] = overlay_color

        segmented_frame = cv2.addWeighted(
            colored_overlay, self.ALPHA,
            frame, 1 - self.ALPHA, 0
        )

        # ── Centerline processing (simple function call) ──
        target_point, centerline_points = self.process_centerline(segmented_frame, road_mask)

        # You can use target_point or centerline_points here later (e.g. for control)

        # Publish result
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