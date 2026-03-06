#!/usr/bin/env python3
import math
import csv
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from tf2_ros import Buffer, TransformListener, LookupException, \
    ConnectivityException, ExtrapolationException


class WaypointRecorder(Node):
    """
    Records (x,y) waypoints while you drive with keyboard.
    - Writes a point every 'min_dist' meters.
    - Saves to CSV when you Ctrl+C the node.
    - With use_slam_pose=True, records in the map frame (SLAM-corrected).
    - With use_slam_pose=False, records from raw odometry.
    """
    def __init__(self):
        super().__init__('record_waypoints')

        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('out_csv', 'recorded_waypoints.csv')
        self.declare_parameter('min_dist', 0.25)

        # SLAM pose parameters
        self.declare_parameter('use_slam_pose', False)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'ego_racecar/base_link')

        self.out_csv = self.get_parameter('out_csv').value
        self.min_dist = float(self.get_parameter('min_dist').value)
        self.use_slam_pose = self.get_parameter('use_slam_pose').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        out_dir = os.path.dirname(self.out_csv)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        self.last_x: Optional[float] = None
        self.last_y: Optional[float] = None
        self.points = []

        if self.use_slam_pose:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.timer = self.create_timer(0.1, self.tf_cb)
            self.get_logger().info(
                f"Recording waypoints from TF: {self.map_frame} -> {self.base_frame}")
        else:
            odom_topic = self.get_parameter('odom_topic').value
            self.sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
            self.get_logger().info(f"Recording waypoints from {odom_topic}")

        self.get_logger().info(f"Output CSV: {self.out_csv}")
        self.get_logger().info("Drive a full lap, then press Ctrl+C here to save.")

    def tf_cb(self):
        """Get pose from SLAM TF and record."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        self.record_point(t.transform.translation.x, t.transform.translation.y)

    def odom_cb(self, msg: Odometry):
        """Get pose from odometry and record."""
        self.record_point(
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def record_point(self, x: float, y: float):
        if self.last_x is None:
            self.last_x, self.last_y = x, y
            self.points.append((x, y))
            return

        d = math.hypot(x - self.last_x, y - self.last_y)
        if d >= self.min_dist:
            self.points.append((x, y))
            self.last_x, self.last_y = x, y

    def save(self):
        if len(self.points) < 5:
            self.get_logger().warn("Not enough points recorded, not saving.")
            return

        with open(self.out_csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(["# x", "y"])
            for x, y in self.points:
                w.writerow([f"{x:.6f}", f"{y:.6f}"])

        self.get_logger().info(f"Saved {len(self.points)} waypoints to {self.out_csv}")


def main():
    rclpy.init()
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
