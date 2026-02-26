#!/usr/bin/env python3
import math
import csv
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class WaypointRecorder(Node):
    """
    Records (x,y) waypoints from Odometry while you drive with keyboard.
    - Writes a point every 'min_dist' meters.
    - Saves to CSV when you Ctrl+C the node.
    """
    def __init__(self):
        super().__init__('record_waypoints')

        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('out_csv', '/home/team2/f1tenth_team_ws/src/team_planning/waypoints/spielberg_recorded.csv')
        self.declare_parameter('min_dist', 0.25)  # meters between saved points

        self.odom_topic = self.get_parameter('odom_topic').value
        self.out_csv = self.get_parameter('out_csv').value
        self.min_dist = float(self.get_parameter('min_dist').value)

        os.makedirs(os.path.dirname(self.out_csv), exist_ok=True)

        self.last_x: Optional[float] = None
        self.last_y: Optional[float] = None
        self.points = []

        self.sub = self.create_subscription(Odometry, self.odom_topic, self.cb, 10)
        self.get_logger().info(f"Recording waypoints from {self.odom_topic}")
        self.get_logger().info(f"Output CSV: {self.out_csv}")
        self.get_logger().info("Drive a full lap, then press Ctrl+C here to save.")

    def cb(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)

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
