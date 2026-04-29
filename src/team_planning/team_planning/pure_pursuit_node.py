#!/usr/bin/env python3

import csv
import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener, TransformException


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # Core params
        self.declare_parameter('drive_topic', '/drive_pp')
        self.declare_parameter('waypoints_csv', '')
        self.declare_parameter('use_slam_pose', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'ego_racecar/base_link')

        # Pure pursuit tuning
        self.declare_parameter('lookahead_distance', 1.2)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('max_steering_angle', 0.4189)
        self.declare_parameter('loop_path', True)

        # Speed params
        self.declare_parameter('max_speed', 2.5)
        self.declare_parameter('min_speed', 0.8)
        self.declare_parameter('curvature_lookahead_points', 8)

        # Local path support from MPP
        self.declare_parameter('use_local_path_topic', False)
        self.declare_parameter('local_path_topic', '/local_path')

        self.drive_topic         = self.get_parameter('drive_topic').value
        self.waypoints_csv       = self.get_parameter('waypoints_csv').value
        self.use_slam_pose       = bool(self.get_parameter('use_slam_pose').value)
        self.map_frame           = self.get_parameter('map_frame').value
        self.base_frame          = self.get_parameter('base_frame').value

        self.lookahead_distance  = float(self.get_parameter('lookahead_distance').value)
        self.wheelbase           = float(self.get_parameter('wheelbase').value)
        self.control_rate        = float(self.get_parameter('control_rate').value)
        self.max_steering_angle  = float(self.get_parameter('max_steering_angle').value)
        self.loop_path           = bool(self.get_parameter('loop_path').value)

        self.max_speed           = float(self.get_parameter('max_speed').value)
        self.min_speed           = float(self.get_parameter('min_speed').value)
        self.curv_lookahead_pts  = int(self.get_parameter('curvature_lookahead_points').value)

        self.use_local_path_topic = bool(self.get_parameter('use_local_path_topic').value)
        self.local_path_topic     = self.get_parameter('local_path_topic').value

        self.pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.global_waypoints: List[Tuple[float, float]] = []
        self.local_waypoints:  List[Tuple[float, float]] = []

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

        self.get_logger().info(
            f'Speed scaling active | min={self.min_speed} m/s | max={self.max_speed} m/s | '
            f'curvature_lookahead_points={self.curv_lookahead_pts}'
        )

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_callback)

    def load_waypoints(self, csv_path: str) -> List[Tuple[float, float]]:
        pts = []
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row:
                    continue
                if row[0].startswith('#'):
                    continue
                if len(row) < 2:
                    continue
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

            qx = tf.transform.rotation.x
            qy = tf.transform.rotation.y
            qz = tf.transform.rotation.z
            qw = tf.transform.rotation.w

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

    def compute_curvature_ahead(
        self,
        nearest_i: int,
        path_points: List[Tuple[float, float]]
    ) -> float:
        n = len(path_points)
        if n < 3:
            return 0.0

        max_curvature = 0.0

        for step in range(self.curv_lookahead_pts):
            if self.loop_path:
                i0 = (nearest_i + step)     % n
                i1 = (nearest_i + step + 1) % n
                i2 = (nearest_i + step + 2) % n
            else:
                i0 = nearest_i + step
                i1 = nearest_i + step + 1
                i2 = nearest_i + step + 2
                if i2 >= n:
                    break

            ax, ay = path_points[i0]
            bx, by = path_points[i1]
            cx, cy = path_points[i2]

            ab = math.hypot(bx - ax, by - ay)
            bc = math.hypot(cx - bx, cy - by)
            ca = math.hypot(ax - cx, ay - cy)

            area = abs((bx - ax) * (cy - ay) - (cx - ax) * (by - ay)) / 2.0

            denom = ab * bc * ca
            if denom < 1e-6:
                continue

            curvature = (4.0 * area) / denom
            if curvature > max_curvature:
                max_curvature = curvature

        return max_curvature

    def curvature_to_speed(self, curvature: float) -> float:
        curvature_scale = 0.8
        t = min(max(curvature - 0.05, 0.0) / curvature_scale, 1.0)
        t_smooth = t * t
        speed = self.max_speed - t_smooth * (self.max_speed - self.min_speed)
        return max(self.min_speed, min(self.max_speed, speed))

    def publish_drive(self, speed: float, steering: float):
        msg = AckermannDriveStamped()
        msg.drive.speed          = float(speed)
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
        nearest_i  = self.nearest_index(x, y, path_points)
        target     = self.get_lookahead_point(x, y, nearest_i, path_points)
        if target is None:
            return

        tx, ty = target

        dx = tx - x
        dy = ty - y

        local_x =  math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y =  math.sin(-yaw) * dx + math.cos(-yaw) * dy

        ld = math.hypot(local_x, local_y)
        if ld < 1e-6:
            return

        curvature = 2.0 * local_y / (ld * ld)
        steering  = math.atan(self.wheelbase * curvature)
        steering  = max(-self.max_steering_angle, min(self.max_steering_angle, steering))

        ahead_curvature = self.compute_curvature_ahead(nearest_i, path_points)
        speed           = self.curvature_to_speed(ahead_curvature)

        self.publish_drive(speed, steering)


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
