#!/usr/bin/env python3
import math
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap')

        # Topics
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/drive')

        # FTG parameters (safe defaults for your sim)
        self.declare_parameter('max_speed', 1.8)
        self.declare_parameter('min_speed', 0.6)
        self.declare_parameter('max_steer', 0.35)          # rad
        self.declare_parameter('lookahead_dist', 2.0)      # meters: used for speed scaling

        self.declare_parameter('fov_deg', 180.0)           # use +/- fov/2 around forward
        self.declare_parameter('bubble_radius', 0.45)      # meters around closest obstacle to clear
        self.declare_parameter('smooth_window', 5)         # moving average window
        self.declare_parameter('gap_threshold', 1.2)       # ranges above this are "open"
        self.declare_parameter('min_range', 0.05)
        self.declare_parameter('max_range', 30.0)

        # Optional steering smoothing
        self.declare_parameter('steer_slew_rate', 2.5)     # rad/s limit
        self.declare_parameter('publish_hz', 30.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)

        self.fov_deg = float(self.get_parameter('fov_deg').value)
        self.bubble_radius = float(self.get_parameter('bubble_radius').value)
        self.smooth_window = int(self.get_parameter('smooth_window').value)
        self.gap_threshold = float(self.get_parameter('gap_threshold').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)

        self.steer_slew_rate = float(self.get_parameter('steer_slew_rate').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)

        self.pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)

        self.last_steer = 0.0
        self.last_time = self.get_clock().now()

        self.get_logger().info(
            f"FTG running | scan={self.scan_topic} drive={self.drive_topic} "
            f"fov={self.fov_deg} bubble={self.bubble_radius} max_speed={self.max_speed}"
        )

    def on_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=self.max_range, posinf=self.max_range, neginf=0.0)
        ranges = np.clip(ranges, self.min_range, self.max_range)

        # Select forward field-of-view
        # LaserScan angles go from angle_min to angle_max; forward is angle=0.
        angles = msg.angle_min + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment
        fov = math.radians(self.fov_deg) / 2.0
        mask = (angles >= -fov) & (angles <= fov)

        r = ranges[mask]
        a = angles[mask]

        if r.size < 10:
            return

        # Smooth ranges (moving average)
        if self.smooth_window > 1:
            w = self.smooth_window
            kernel = np.ones(w, dtype=np.float32) / w
            r = np.convolve(r, kernel, mode='same')

        # Find closest obstacle in FOV and apply "bubble"
        closest_idx = int(np.argmin(r))
        closest_dist = float(r[closest_idx])

        bubble = int(self.bubble_radius / max(msg.angle_increment * max(closest_dist, 0.1), 1e-3))
        start = max(0, closest_idx - bubble)
        end = min(r.size - 1, closest_idx + bubble)
        r_bubbled = r.copy()
        r_bubbled[start:end+1] = self.min_range

        # Find largest contiguous gap above threshold
        open_mask = r_bubbled > self.gap_threshold
        if not np.any(open_mask):
            # If no big gap, just aim at max range direction
            target_idx = int(np.argmax(r_bubbled))
        else:
            # Find runs of True in open_mask and choose the longest
            idx = np.where(open_mask)[0]
            splits = np.where(np.diff(idx) > 1)[0] + 1
            groups = np.split(idx, splits)
            best = max(groups, key=len)
            # Choose best point inside gap: farthest point (or mid)
            target_idx = int(best[np.argmax(r_bubbled[best])])

        target_angle = float(a[target_idx])

        # Convert target angle to steering command (clamp)
        desired_steer = clamp(target_angle, -self.max_steer, self.max_steer)

        # Slew-rate limit steering for stability
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1.0 / self.publish_hz
        max_delta = self.steer_slew_rate * dt
        steer = clamp(desired_steer, self.last_steer - max_delta, self.last_steer + max_delta)
        self.last_steer = steer
        self.last_time = now

        # Speed: slower when turning more
        turn_factor = 1.0 - min(1.0, abs(steer) / self.max_steer)
        speed = self.min_speed + (self.max_speed - self.min_speed) * (0.25 + 0.75 * turn_factor)
        speed = clamp(speed, self.min_speed, self.max_speed)

        out = AckermannDriveStamped()
        out.header.stamp = now.to_msg()
        out.drive.steering_angle = float(steer)
        out.drive.speed = float(speed)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = FollowTheGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
