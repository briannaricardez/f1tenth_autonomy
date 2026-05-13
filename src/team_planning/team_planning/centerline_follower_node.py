#!/usr/bin/env python3
"""
Centerline Follower.

Map-free planning layer for F1TENTH. Extracts a local track centerline directly
from each LiDAR scan using a forward-slab midpoint method and publishes it as
nav_msgs/Path on /local_path. Pure Pursuit then consumes this topic exactly as
it currently consumes MPP output, but with no SLAM and no global waypoint CSV.

Algorithm (per scan):
  1. Mask scan to forward FOV and valid range.
  2. Convert to Cartesian (x, y) in the LiDAR frame.
  3. For each forward lookahead distance d, take a thin slab [d - h, d + h]:
       y_left  = max positive y in slab  (left wall hit)
       y_right = min (most negative) y in slab  (right wall hit)
       centerline point = (d, (y_left + y_right) / 2)
  4. Optional running-mean smoothing on the lateral component.
  5. Publish ordered points as nav_msgs/Path in ego_racecar/base_link.

The static base_link -> laser transform on this car is identity (see launch
files), so points produced in the laser frame are equivalent to points in
ego_racecar/base_link. Pure Pursuit is configured with map_frame ==
base_frame == ego_racecar/base_link so its TF lookup returns identity and the
local path is followed directly.
"""

import math
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class CenterlineFollower(Node):
    def __init__(self):
        super().__init__('centerline_follower')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('local_path_topic', '/local_path')
        self.declare_parameter('marker_topic', '/centerline_markers')
        self.declare_parameter('output_frame', 'ego_racecar/base_link')
        self.declare_parameter('forward_fov_deg', 180.0)
        self.declare_parameter('lookahead_samples',
                               [0.4, 0.8, 1.2, 1.6, 2.0, 2.5, 3.0])
        self.declare_parameter('slab_half_width', 0.20)
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 10.0)
        self.declare_parameter('smoothing_window', 3)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('max_lateral_offset', 2.5)

        self.scan_topic       = self.get_parameter('scan_topic').value
        self.local_path_topic = self.get_parameter('local_path_topic').value
        self.marker_topic     = self.get_parameter('marker_topic').value
        self.output_frame     = self.get_parameter('output_frame').value
        self.forward_fov_deg  = float(self.get_parameter('forward_fov_deg').value)
        self.lookahead_samples = [float(d) for d in
                                  self.get_parameter('lookahead_samples').value]
        self.slab_half_width  = float(self.get_parameter('slab_half_width').value)
        self.range_min        = float(self.get_parameter('range_min').value)
        self.range_max        = float(self.get_parameter('range_max').value)
        self.smoothing_window = int(self.get_parameter('smoothing_window').value)
        self.publish_markers  = bool(self.get_parameter('publish_markers').value)
        self.max_lateral_offset = float(self.get_parameter('max_lateral_offset').value)

        self.path_pub = self.create_publisher(Path, self.local_path_topic, 10)
        if self.publish_markers:
            self.marker_pub = self.create_publisher(
                MarkerArray, self.marker_topic, 10)

        self.sub = self.create_subscription(
            LaserScan, self.scan_topic, self.on_scan, 10)

        self.get_logger().info(
            f'Centerline follower running | scan={self.scan_topic} '
            f'path={self.local_path_topic} frame={self.output_frame} '
            f'lookaheads={self.lookahead_samples}'
        )

    def on_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.nan_to_num(
            ranges, nan=self.range_max, posinf=self.range_max, neginf=0.0)

        angles = msg.angle_min + np.arange(
            len(ranges), dtype=np.float32) * msg.angle_increment

        fov = math.radians(self.forward_fov_deg) / 2.0
        mask = (
            (angles >= -fov) & (angles <= fov)
            & (ranges >= self.range_min) & (ranges <= self.range_max)
        )

        r = ranges[mask]
        a = angles[mask]

        if r.size < 10:
            return

        xs = r * np.cos(a)
        ys = r * np.sin(a)

        forward_mask = xs > 0.0
        xs = xs[forward_mask]
        ys = ys[forward_mask]

        if xs.size < 10:
            return

        centerline_pts = self._extract_centerline(xs, ys)
        if not centerline_pts:
            return

        centerline_pts = self._smooth(centerline_pts)

        self._publish_path(centerline_pts, msg.header.stamp)
        if self.publish_markers:
            self._publish_markers(centerline_pts, msg.header.stamp)

    def _extract_centerline(
        self, xs: np.ndarray, ys: np.ndarray
    ) -> List[Tuple[float, float]]:
        pts: List[Tuple[float, float]] = []
        last_lateral = 0.0
        h = self.slab_half_width

        for d in self.lookahead_samples:
            slab = (xs >= d - h) & (xs <= d + h)
            if not np.any(slab):
                pts.append((d, last_lateral))
                continue

            ys_slab = ys[slab]
            y_left  = float(np.max(ys_slab))
            y_right = float(np.min(ys_slab))

            if y_left <= 0.0 or y_right >= 0.0:
                pts.append((d, last_lateral))
                continue

            lateral = 0.5 * (y_left + y_right)
            lateral = max(-self.max_lateral_offset,
                          min(self.max_lateral_offset, lateral))
            pts.append((d, lateral))
            last_lateral = lateral

        return pts

    def _smooth(
        self, pts: List[Tuple[float, float]]
    ) -> List[Tuple[float, float]]:
        w = self.smoothing_window
        if w <= 1 or len(pts) < 3:
            return pts

        ys = np.array([p[1] for p in pts], dtype=np.float32)
        kernel = np.ones(w, dtype=np.float32) / w
        ys_smooth = np.convolve(ys, kernel, mode='same')
        return [(pts[i][0], float(ys_smooth[i])) for i in range(len(pts))]

    def _publish_path(self, pts: List[Tuple[float, float]], stamp):
        msg = Path()
        msg.header.stamp = stamp
        msg.header.frame_id = self.output_frame

        for x, y in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)

        self.path_pub.publish(msg)

    def _publish_markers(self, pts: List[Tuple[float, float]], stamp):
        marker_array = MarkerArray()

        line = Marker()
        line.header.stamp = stamp
        line.header.frame_id = self.output_frame
        line.ns = 'centerline'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.05
        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 0.0
        line.color.a = 1.0
        line.pose.orientation.w = 1.0

        for x, y in pts:
            from geometry_msgs.msg import Point
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.0
            line.points.append(pt)

        marker_array.markers.append(line)
        self.marker_pub.publish(marker_array)


def main():
    rclpy.init()
    node = CenterlineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
