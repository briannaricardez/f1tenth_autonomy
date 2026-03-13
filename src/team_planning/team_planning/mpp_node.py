#!/usr/bin/env python3

import csv
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException


class MPPNode(Node):
    """
    Simple Model Predictive Planning starter node.

    First version:
    - loads global waypoint CSV
    - uses current localized pose
    - finds nearest waypoint
    - publishes a short local horizon ahead as nav_msgs/Path

    This is the planning layer only.
    Pure Pursuit will still be the controller.
    """

    def __init__(self):
        super().__init__('mpp')

        self.declare_parameter('waypoints_csv', '')
        self.declare_parameter('path_topic', '/local_path')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'ego_racecar/base_link')
        self.declare_parameter('horizon_points', 30)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('loop_path', True)

        self.waypoints_csv = self.get_parameter('waypoints_csv').value
        self.path_topic = self.get_parameter('path_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.horizon_points = int(self.get_parameter('horizon_points').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.loop_path = bool(self.get_parameter('loop_path').value)

        if not self.waypoints_csv:
            raise ValueError('waypoints_csv parameter is required')

        self.global_waypoints: List[Tuple[float, float]] = self.load_waypoints(self.waypoints_csv)
        if len(self.global_waypoints) < 2:
            raise ValueError('Need at least 2 waypoints in CSV')

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f'MPP loaded {len(self.global_waypoints)} global waypoints | '
            f'publishing local path to {self.path_topic}'
        )

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

    def get_current_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            return x, y
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    def nearest_index(self, x: float, y: float) -> int:
        best_i = 0
        best_d = float('inf')
        for i, (wx, wy) in enumerate(self.global_waypoints):
            d = (wx - x) ** 2 + (wy - y) ** 2
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def build_local_path(self, start_idx: int) -> List[Tuple[float, float]]:
        pts = []
        n = len(self.global_waypoints)

        for k in range(self.horizon_points):
            idx = start_idx + k
            if self.loop_path:
                idx %= n
            elif idx >= n:
                break
            pts.append(self.global_waypoints[idx])

        return pts

    def timer_callback(self):
        pose = self.get_current_pose()
        if pose is None:
            return

        x, y = pose
        idx = self.nearest_index(x, y)
        local_pts = self.build_local_path(idx)

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        for px, py in local_pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = px
            ps.pose.position.y = py
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)

        self.path_pub.publish(msg)


def main():
    rclpy.init()
    node = MPPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
