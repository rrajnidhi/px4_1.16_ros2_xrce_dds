import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import torch
import os
import cv2
import math

from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

# PX4 messages
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint


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

        # ── PX4 Offboard publishers (v1.16 compatible) ──
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10
        )

        # Control tuning (start very small!)
        self.k_yaw_rate       = 0.10       # now used as k_lateral_vel → m/s per pixel error
        self.k_forward_vel    = 0.006       # m/s per pixel (forward bias)
        self.max_yaw_rate     = 2.5         # now used as max_lateral_vel → m/s cap
        self.max_forward_vel  = 1.2         # m/s cap
        self.deadzone_px      = 12.0        # ignore tiny errors

        # Default cruise speed when road is centered
        self.base_forward_vel = 2.5         # m/s

        # Timer: send setpoints at ~30 Hz (required!)
        self.setpoint_timer = self.create_timer(0.033, self.publish_offboard_setpoint)

        # State from latest frame
        self.latest_x_error     = 0.0
        self.latest_y_error     = 0.0
        self.have_valid_target  = False

        # ── Centerline & visualization settings ──
        self.row_step            = 12
        self.min_road_width      = 40
        self.look_ahead_fraction = 0.4
        self.centerline_color    = (50, 180, 255)
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

    def publish_offboard_setpoint(self):
        """Send continuous offboard heartbeat + velocity command (PX4 v1.16 / Humble)"""

        now_ns = self.get_clock().now().nanoseconds
        timestamp_us = int(now_ns / 1000)

        # 1. Heartbeat: OffboardControlMode
        mode_msg = OffboardControlMode()
        mode_msg.timestamp = timestamp_us
        mode_msg.position = False
        mode_msg.velocity = True          # body-frame velocity
        mode_msg.acceleration = False
        mode_msg.attitude = False
        mode_msg.body_rate = False
        mode_msg.thrust_and_torque = False
        self.offboard_control_mode_pub.publish(mode_msg)

        # 2. Setpoint: body-frame velocity + yawspeed
        sp = TrajectorySetpoint()
        sp.timestamp = timestamp_us

        # Body-frame velocity (FLU: x=forward, y=right/left, z=up/down)
        forward_vel = self.base_forward_vel
        lateral_vel = 0.0   # body y-velocity for lateral correction

        if self.have_valid_target:
            # Lateral correction: strafe/sideslip to center the road
        
            lateral_vel = -self.latest_x_error * self.k_yaw_rate
            lateral_vel = np.clip(lateral_vel, -self.max_yaw_rate, self.max_yaw_rate)         

            # Forward speed bias
            forward_vel += self.latest_y_error * self.k_forward_vel
            forward_vel = np.clip(forward_vel, 0.2, self.max_forward_vel)

        # Apply deadzone on lateral velocity
        if abs(self.latest_x_error) < self.deadzone_px:
            lateral_vel = 0.0

        #frame convertion

        sp.velocity = [lateral_vel, -forward_vel,  0.0]   

        # Yaw control: set yawspeed to 0 (no yaw rate)
        sp.yawspeed = 0.0                               # ← correct field name

        # Leave other fields as NaN
        sp.position = [math.nan, math.nan, math.nan]
        sp.acceleration = [math.nan, math.nan, math.nan]
        sp.jerk = [math.nan, math.nan, math.nan]
        sp.yaw = math.nan

        self.trajectory_setpoint_pub.publish(sp)

        # Debug print every few seconds
        if hasattr(self, '_last_log') and (now_ns - self._last_log) < 3e9:
            return
        self._last_log = now_ns
        self.get_logger().info(
            f"Offboard cmd → lateral_vel: {lateral_vel:.3f} m/s | fwd_vel: {forward_vel:.2f} m/s"
        )

        
    def score_road_component(self, stats, centroid, image_shape):
        h, w = image_shape
        area = stats[cv2.CC_STAT_AREA]
        cx, cy = centroid
        dist_to_center = np.sqrt((cx - w / 2) ** 2 + (cy - h / 2) ** 2)
        return area * 1.0 - dist_to_center * 2.0

    def extract_centerline_points(self, road_mask: np.ndarray, h: int, w: int) -> list[tuple[int, int]]:
        points = []
        for y in range(0, h, self.row_step):
            row = road_mask[y, :]
            road_x = np.where(row > 127)[0]
            if len(road_x) >= self.min_road_width:
                x_center = int(np.mean(road_x))
                points.append((x_center, y))
        return points

    def select_target_point(self, centerline_points: list[tuple[int, int]], image_width: int) -> tuple[int, int] | None:
        if not centerline_points:
            return None
        idx = int(len(centerline_points) * self.look_ahead_fraction)
        idx = min(max(idx, 0), len(centerline_points) - 1)
        return centerline_points[idx]

    def draw_centerline_and_target(self, frame: np.ndarray, centerline_points: list[tuple[int, int]], target_point: tuple[int, int] | None) -> None:
        h, w, _ = frame.shape
        if len(centerline_points) >= 2:
            pts = np.array(centerline_points, dtype=np.int32)
            cv2.polylines(frame, [pts], isClosed=False, color=self.centerline_color, thickness=3, lineType=cv2.LINE_AA)
        if target_point is not None:
            cv2.circle(frame, target_point, 12, self.target_point_color, -1)
            cv2.circle(frame, target_point, 4, (255, 255, 255), -1)
        cv2.circle(frame, (w // 2, h // 2), 6, (180, 180, 60), 2)

    def calculate_pixel_errors(self, target_point: tuple[int, int] | None, frame_height: int, frame_width: int) -> tuple[float | None, float | None]:
        if target_point is None:
            return None, None
        center_x = frame_width // 2
        center_y = frame_height // 2
        target_x, target_y = target_point
        x_error = target_x - center_x
        y_error = target_y - center_y
        return float(x_error), float(y_error)

    def process_centerline(self, segmented_frame: np.ndarray, road_mask: np.ndarray):
        h, w, _ = segmented_frame.shape
        centerline_points = self.extract_centerline_points(road_mask, h, w)
        if not centerline_points:
            return None, []
        target_point = self.select_target_point(centerline_points, w)
        self.draw_centerline_and_target(segmented_frame, centerline_points, target_point)
        return target_point, centerline_points

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

        # centerline processing 
        target_point, centerline_points = self.process_centerline(segmented_frame, road_mask)

        # pixel error is calculated for control
        x_error, y_error = self.calculate_pixel_errors(target_point, h, w)

        if x_error is not None:
            self.latest_x_error = x_error
            self.latest_y_error = y_error
            self.have_valid_target = True
            #self.get_logger().info(f"Pixel errors → x: {x_error:+.1f} px (right positive), "f"y: {y_error:+.1f} px (forward/lower positive)")
        else:
            self.have_valid_target = False
            self.latest_x_error = 0.0
            self.latest_y_error = 0.0

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