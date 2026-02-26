#!/usr/bin/env python3
import math
import csv
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def yaw_from_quat(q) -> float:
    # geometry_msgs/Quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('drive_topic', '/drive_pp')
        self.declare_parameter('waypoints_csv', '')
        self.declare_parameter('lookahead', 1.2)          # meters
        self.declare_parameter('wheelbase', 0.33)         # meters (approx F1TENTH)
        self.declare_parameter('steer_limit', 0.4189)     # rad
        self.declare_parameter('speed', 1.5)              # m/s

        self.odom_topic = self.get_parameter('odom_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.lookahead = float(self.get_parameter('lookahead').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.steer_limit = float(self.get_parameter('steer_limit').value)
        self.speed = float(self.get_parameter('speed').value)

        wp_path = self.get_parameter('waypoints_csv').value
        if not wp_path:
            self.get_logger().error("waypoints_csv param is empty. Provide a CSV file path.")
            raise RuntimeError("Missing waypoints_csv")

        self.waypoints: List[Tuple[float, float]] = self.load_waypoints(wp_path)
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {wp_path}")

        self.pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)

        self.last_nearest_i = 0

    def load_waypoints(self, path: str) -> List[Tuple[float, float]]:
        pts = []
        with open(path, 'r') as f:
            reader = csv.reader(line for line in f if line.strip() and not line.strip().startswith('#'))
            for row in reader:
                if len(row) < 2:
                    continue
                pts.append((float(row[0]), float(row[1])))
        if len(pts) < 2:
            raise RuntimeError("Waypoint file must contain at least 2 points")
        return pts

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)

        # Find nearest waypoint (search a window forward for speed)
        n = len(self.waypoints)
        search = 200  # max points to scan forward
        best_i = self.last_nearest_i
        best_d2 = float('inf')

        for k in range(search):
            i = (self.last_nearest_i + k) % n
            wx, wy = self.waypoints[i]
            d2 = (wx - x) ** 2 + (wy - y) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_i = i

        self.last_nearest_i = best_i

        # Pick lookahead point ahead along the path
        target_i = best_i
        accum = 0.0
        while accum < self.lookahead:
            nxt = (target_i + 1) % n
            x1, y1 = self.waypoints[target_i]
            x2, y2 = self.waypoints[nxt]
            accum += math.hypot(x2 - x1, y2 - y1)
            target_i = nxt
            if accum > 50.0:  # safety break
                break

        tx, ty = self.waypoints[target_i]

        # Transform target point into vehicle frame
        dx = tx - x
        dy = ty - y

        # rotate by -yaw
        x_v = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        y_v = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        # If target is behind us, do nothing
        if x_v <= 0.05:
            self.publish_drive(0.0, 0.0)
            return

        # Pure pursuit curvature
        Ld2 = x_v * x_v + y_v * y_v
        curvature = (2.0 * y_v) / Ld2
        steering = math.atan(curvature * self.wheelbase)
        steering = clamp(steering, -self.steer_limit, self.steer_limit)

        self.publish_drive(self.speed, steering)

    def publish_drive(self, speed: float, steering: float):
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.drive.speed = float(speed)
        out.drive.steering_angle = float(steering)
        self.pub.publish(out)


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
