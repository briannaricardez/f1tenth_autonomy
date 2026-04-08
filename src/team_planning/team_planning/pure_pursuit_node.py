#!/usr/bin/env python3

import csv
import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
<<<<<<< HEAD

=======
from nav_msgs.msg import Odometry, Path
>>>>>>> f374ce0 (Add MPP node, PP local-path tracking, and SLAM-only test launches)
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener, TransformException


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # Core params
        self.declare_parameter('drive_topic', '/drive_pp')
        self.declare_parameter('waypoints_csv', '')
<<<<<<< HEAD
        self.declare_parameter('use_slam_pose', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'ego_racecar/base_link')

        # Pure pursuit tuning
        self.declare_parameter('lookahead_distance', 1.2)
        self.declare_parameter('speed', 1.5)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('max_steering_angle', 0.4189)  # about 24 deg
        self.declare_parameter('loop_path', True)

        # New: local path support from MPP
        self.declare_parameter('use_local_path_topic', False)
        self.declare_parameter('local_path_topic', '/local_path')

        self.drive_topic = self.get_parameter('drive_topic').value
        self.waypoints_csv = self.get_parameter('waypoints_csv').value
        self.use_slam_pose = bool(self.get_parameter('use_slam_pose').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.speed = float(self.get_parameter('speed').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.control_rate = float(self.get_parameter('control_rate').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.loop_path = bool(self.get_parameter('loop_path').value)

        self.use_local_path_topic = bool(self.get_parameter('use_local_path_topic').value)
        self.local_path_topic = self.get_parameter('local_path_topic').value

        self.pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.global_waypoints: List[Tuple[float, float]] = []
        self.local_waypoints: List[Tuple[float, float]] = []

        if self.waypoints_csv:
            self.global_waypoints = self.load_waypoints(self.waypoints_csv)
            self.get_logger().info(
                f'Loaded {len(self.global_waypoints)} waypoints from {self.waypoints_csv}'
            )
        else:
            self.get_logger().warn('No waypoints_csv provided. Waiting for local path only.')

        if self.use_local_path_topic:
            self.local_path_sub = self.create_subscription(
                Path,
                self.local_path_topic,
                self.local_path_callback,
                10
            )
            self.get_logger().info(
                f'Pure Pursuit using local path topic: {self.local_path_topic}'
            )

        if self.use_slam_pose:
            self.get_logger().info(
                f'Using SLAM pose: TF lookup {self.map_frame} -> {self.base_frame}'
            )

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_callback)

    def load_waypoints(self, csv_path: str) -> List[Tuple[float, float]]:
        pts = []
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
=======
        self.declare_parameter('lookahead', 1.2)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('steer_limit', 0.4189)
        self.declare_parameter('speed', 1.5)

        self.declare_parameter('use_slam_pose', False)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'ego_racecar/base_link')

        # NEW: local path support
        self.declare_parameter('use_local_path', True)
        self.declare_parameter('path_topic', '/local_path')
        self.declare_parameter('local_path_timeout', 0.5)

        self.drive_topic = self.get_parameter('drive_topic').value
        self.lookahead = float(self.get_parameter('lookahead').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.steer_limit = float(self.get_parameter('steer_limit').value)
        self.speed = float(self.get_parameter('speed').value)

        self.use_slam_pose = self.get_parameter('use_slam_pose').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.use_local_path = self.get_parameter('use_local_path').value
        self.path_topic = self.get_parameter('path_topic').value
        self.local_path_timeout = float(
            self.get_parameter('local_path_timeout').value
        )

        wp_path = self.get_parameter('waypoints_csv').value
        if not wp_path:
            raise RuntimeError("Missing waypoints_csv")

        self.global_waypoints: List[Tuple[float, float]] = self.load_waypoints(wp_path)
        self.local_waypoints: List[Tuple[float, float]] = []
        self.local_path_time = None

        self.pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)

        if self.use_local_path:
            self.path_sub = self.create_subscription(
                Path,
                self.path_topic,
                self.path_cb,
                10
            )

        self.last_nearest_i = 0

        if self.use_slam_pose:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.timer = self.create_timer(1.0 / 30.0, self.control_loop)
        else:
            odom_topic = self.get_parameter('odom_topic').value
            self.sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)

    def load_waypoints(self, path: str) -> List[Tuple[float, float]]:
        if not os.path.isfile(path):
            raise FileNotFoundError(path)

        pts = []
        with open(path, 'r') as f:
            reader = csv.reader(
                line for line in f
                if line.strip() and not line.strip().startswith('#')
            )
>>>>>>> f374ce0 (Add MPP node, PP local-path tracking, and SLAM-only test launches)
            for row in reader:
                if not row:
                    continue
                if row[0].startswith('#'):
                    continue
                if len(row) < 2:
                    continue
<<<<<<< HEAD
                x = float(row[0].strip())
                y = float(row[1].strip())
                pts.append((x, y))
        return pts

    def local_path_callback(self, msg: Path):
        pts = []
        for ps in msg.poses:
            pts.append((ps.pose.position.x, ps.pose.position.y))
        self.local_waypoints = pts

    def get_current_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
=======
                pts.append((float(row[0]), float(row[1])))

        return pts

    def path_cb(self, msg: Path):
        pts = []
        for ps in msg.poses:
            pts.append((ps.pose.position.x, ps.pose.position.y))

        if len(pts) >= 2:
            self.local_waypoints = pts
            self.local_path_time = self.get_clock().now()

    def local_path_is_fresh(self):
        if not self.use_local_path:
            return False
        if len(self.local_waypoints) < 2:
            return False
        if self.local_path_time is None:
            return False

        age = (self.get_clock().now() - self.local_path_time).nanoseconds * 1e-9
        return age <= self.local_path_timeout

    def get_active_path(self):
        if self.local_path_is_fresh():
            return self.local_waypoints, True
        return self.global_waypoints, False

    def control_loop(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
>>>>>>> f374ce0 (Add MPP node, PP local-path tracking, and SLAM-only test launches)

            qx = tf.transform.rotation.x
            qy = tf.transform.rotation.y
            qz = tf.transform.rotation.z
            qw = tf.transform.rotation.w

<<<<<<< HEAD
            yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz)
            )
            return x, y, yaw

        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def get_active_path(self) -> List[Tuple[float, float]]:
        if self.use_local_path_topic and len(self.local_waypoints) > 1:
            return self.local_waypoints
        return self.global_waypoints

    def nearest_index(self, x: float, y: float, path_points: List[Tuple[float, float]]) -> int:
        best_i = 0
        best_d = float('inf')
        for i, (px, py) in enumerate(path_points):
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def get_lookahead_point(
        self,
        x: float,
        y: float,
        nearest_i: int,
        path_points: List[Tuple[float, float]]
    ) -> Optional[Tuple[float, float]]:
        n = len(path_points)
        if n == 0:
            return None

        for step in range(n if self.loop_path else max(0, n - nearest_i)):
            idx = (nearest_i + step) % n if self.loop_path else nearest_i + step
            if idx >= n:
                break

            px, py = path_points[idx]
            d = math.hypot(px - x, py - y)
            if d >= self.lookahead_distance:
                return px, py

        return path_points[-1]

    def publish_drive(self, speed: float, steering: float):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self.pub.publish(msg)

    def control_callback(self):
        pose = self.get_current_pose()
        if pose is None:
            return

        path_points = self.get_active_path()
        if len(path_points) < 2:
            self.get_logger().warn('No active path available for Pure Pursuit')
            return

        x, y, yaw = pose
        nearest_i = self.nearest_index(x, y, path_points)
        target = self.get_lookahead_point(x, y, nearest_i, path_points)
        if target is None:
            return

        tx, ty = target
=======
    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.pursue(x, y, yaw)

    def nearest_index(self, pts, x, y):
        best_i = 0
        best_d2 = float('inf')
        for i, (wx, wy) in enumerate(pts):
            d2 = (wx - x) ** 2 + (wy - y) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i

    def pursue(self, x: float, y: float, yaw: float):
        pts, using_local = self.get_active_path()
        n = len(pts)
        if n < 2:
            return

        best_i = self.nearest_index(pts, x, y)

        target_i = best_i
        accum = 0.0
        while accum < self.lookahead:
            nxt = min(target_i + 1, n - 1) if using_local else (target_i + 1) % n
            x1, y1 = pts[target_i]
            x2, y2 = pts[nxt]
            accum += math.hypot(x2 - x1, y2 - y1)
            target_i = nxt
            if using_local and target_i >= n - 1:
                break

        tx, ty = pts[target_i]
>>>>>>> f374ce0 (Add MPP node, PP local-path tracking, and SLAM-only test launches)

        dx = tx - x
        dy = ty - y

<<<<<<< HEAD
        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        ld = math.hypot(local_x, local_y)
        if ld < 1e-6:
            return

        curvature = 2.0 * local_y / (ld * ld)
        steering = math.atan(self.wheelbase * curvature)

        steering = max(-self.max_steering_angle, min(self.max_steering_angle, steering))
=======
        if x_v <= 0.05:
            self.publish_drive(0.0, 0.0)
            return

        Ld2 = x_v * x_v + y_v * y_v
        curvature = (2.0 * y_v) / Ld2
        steering = math.atan(curvature * self.wheelbase)
        steering = clamp(steering, -self.steer_limit, self.steer_limit)
>>>>>>> f374ce0 (Add MPP node, PP local-path tracking, and SLAM-only test launches)

        self.publish_drive(self.speed, steering)


def main():
    rclpy.init()
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
