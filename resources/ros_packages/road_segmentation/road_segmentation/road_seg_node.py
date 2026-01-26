import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

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
from px4_msgs.msg import OffboardControlMode, VehicleAttitude, VehicleLocalPosition, VehicleStatus, TrajectorySetpoint


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

        # PX4 publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10
        )

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile=qos
        )
        self.get_logger().info("Subscribed to /fmu/out/vehicle_attitude with BEST_EFFORT QoS")

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.status_callback,
            qos_profile=qos
        )

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_pos_callback,
            qos_profile=qos
        )


        self.current_z = 0.0
        self.desired_z = 0.0           # Set when first entering OFFBOARD
        self.kp_z = 1.2                # P gain for altitude hold (tune 0.8–2.0)
        self.hover_thrust = -1.0      # Base hover thrust (negative Z) — tune this

        # Control tuning
        self.k_yaw = 2.0                    # how aggressively to set desired yaw
        self.yaw_threshold = 0.12           # rad (~7°) — consider aligned
        self.thrust_forward = 0.75          # normalized forward thrust when aligned
        self.thrust_align = 0.73            # thrust while rotating

        self.setpoint_timer = self.create_timer(0.033, self.publish_trajectory_setpoint)

        # State
        self.current_yaw = 0.0
        self.latest_x_error = 0.0
        self.latest_y_error = 0.0
        self.have_valid_target = False
        self.offboard_active = False       # NEW: track when we enter OFFBOARD

        # Centerline settings
        self.row_step = 12
        self.min_road_width = 40
        self.look_ahead_fraction = 0.4
        self.centerline_color = (50, 180, 255)
        self.target_point_color = (0, 0, 255)

        self.get_logger().info('Road segmentation node started (q_d yaw + thrust + altitude hold)')
        self.get_logger().info(f"Look-ahead: {self.look_ahead_fraction:.2f} | Yaw gain: {self.k_yaw:.2f}")

        # YOLO
        pkg_path = get_package_share_directory('road_segmentation')
        model_path = os.path.join(pkg_path, 'models', 'best.pt')
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = YOLO(model_path)
        self.model.to(self.device)

        self.ROAD_CLASS_ID = 0
        self.MIN_ROAD_AREA = 5000
        self.ALPHA = 0.5

        self.current_state = ""

        self.latest_look_angle = 0.0

    def status_callback(self, msg):
        if msg.nav_state == 14:
            self.current_state = "OFFBOARD" 
        else:
            self.current_state = "not_OFFBOARD" 

    def attitude_callback(self, msg):
        q = msg.q
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2]**2 + q[3]**2)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def local_pos_callback(self, msg):
        self.current_z = msg.z  # NED, negative up

    def yaw_to_quaternion(self, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return [cy, 0.0, 0.0, sy]

    def publish_trajectory_setpoint(self):
        timestamp_us = int(self.get_clock().now().nanoseconds / 1000)

        mode = OffboardControlMode()
        mode.timestamp = timestamp_us
        mode.position = False
        mode.velocity = True
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.thrust_and_torque = False
        self.offboard_control_mode_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = timestamp_us

        if self.current_state != "OFFBOARD":
            sp.position   = [float('nan')] * 3
            sp.velocity   = [0.0, 0.0, 0.0]
            sp.yaw        = 0.0
            sp.yawspeed   = float('nan')
        else:
            sp.position = [float('nan')] * 3

            if self.have_valid_target:
                # ── FORWARD SPEED ── always try to move forward
                forward_speed = 1.0          # m/s – start with 0.8–1.5, tune
                sp.velocity = [forward_speed, 0.0, 0.0]

                # ── YAW CORRECTION ──
                error = self.latest_look_angle

                # Option A: absolute yaw (smoother, recommended first)
                desired_yaw = self.current_yaw + (self.k_yaw * error)

                # Rate limit big changes (prevents violent turns)
                max_delta_yaw = 0.30         # rad per cycle (~17° per 33 ms)
                delta = np.clip(desired_yaw - self.current_yaw, -max_delta_yaw, max_delta_yaw)
                sp.yaw = self.current_yaw + delta
                sp.yawspeed = float('nan')

                # Option B: yaw rate mode (more direct, often better with vision)
                # sp.yaw = float('nan')
                # sp.yawspeed = np.clip(3.0 * error, -1.8, 1.8)   # max ±~100 deg/s

            else:
                # No target → slow down and hold heading
                sp.velocity = [0.3, 0.0, 0.0]   # crawl forward or [0,0,0]
                sp.yaw = self.current_yaw
                sp.yawspeed = float('nan')

        self.trajectory_setpoint_pub.publish(sp)

    def calculate_look_angle(self, target_point, frame_width, frame_height):
        if target_point is None:
            return 0.0, False

        center_x = frame_width / 2.0
        center_y = frame_height / 2.0
        tx, ty   = target_point

        dx = tx - center_x
        dy = ty - center_y

        print(f"Target: {target_point}, Center: ({center_x:.1f}, {center_y:.1f}), dx:{dx:6.1f}, dy:{dy:6.1f} → ", end="")

        # ───────────────────────────────────────────────────────────────
        # NO rejection based on dy anymore
        # We always compute the angle — even if target is directly below or very close
        # ───────────────────────────────────────────────────────────────

        raw_angle = math.atan2(dx, dy)

        # Important: choose the correct sign for your setup
        angle_rad = raw_angle          # try this first
        # angle_rad = -raw_angle       # flip if turning direction is wrong

        # Small deadband to prevent jitter when almost perfectly aligned
        if abs(angle_rad) < 0.04:      # ≈ 2.3°
            angle_rad = 0.0

        print(f"angle: {angle_rad:+.3f} rad")
        return angle_rad, True


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
        #center_point of frame
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

        overlay_color = np.array([0, 255, 0], dtype=np.uint8)
        colored_overlay = np.zeros_like(frame)
        colored_overlay[road_mask > 0] = overlay_color

        segmented_frame = cv2.addWeighted(
            colored_overlay, self.ALPHA,
            frame, 1 - self.ALPHA, 0
        )

        target_point, centerline_points = self.process_centerline(segmented_frame, road_mask)

        x_error, y_error = self.calculate_pixel_errors(target_point, h, w)

        # calculate for yaw control
        look_angle, valid = self.calculate_look_angle(target_point, w, h)
        #print(target_point, w, h, look_angle, valid)

        if valid:
            self.latest_look_angle = look_angle
            self.have_valid_target = True
        else:
            self.latest_look_angle = 0.0
            self.have_valid_target = False

        if x_error is not None:
            self.latest_x_error = x_error
            self.latest_y_error = y_error
            self.have_valid_target = True
        else:
            self.have_valid_target = False
            self.latest_x_error = 0.0
            self.latest_y_error = 0.0

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