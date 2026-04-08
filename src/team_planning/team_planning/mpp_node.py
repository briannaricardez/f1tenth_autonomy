#!/usr/bin/env python3
<<<<<<< HEAD

import csv
import math
from typing import List, Tuple
=======
import csv
import math
from typing import List, Tuple, Optional
>>>>>>> f374ce0 (Add MPP node, PP local-path tracking, and SLAM-only test launches)

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
<<<<<<< HEAD
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
=======
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MPPNode(Node):
    """
    Local planner:
    - loads global waypoint CSV
    - gets SLAM pose from TF
    - publishes a short local horizon path
    - can shift path slightly using LiDAR obstacle check
>>>>>>> f374ce0 (Add MPP node, PP local-path tracking, and SLAM-only test launches)
    """

    def __init__(self):
        super().__init__('mpp')

        self.declare_parameter('waypoints_csv', '')
        self.declare_parameter('path_topic', '/local_path')
<<<<<<< HEAD
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'ego_racecar/base_link')
        self.declare_parameter('horizon_points', 30)
        self.declare_parameter('publish_rate', 10.0)
=======
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'ego_racecar/base_link')
        self.declare_parameter('horizon_points', 25)
        self.declare_parameter('publish_rate', 15.0)
>>>>>>> f374ce0 (Add MPP node, PP local-path tracking, and SLAM-only test launches)
        self.declare_parameter('loop_path', True)

        self.waypoints_csv = self.get_parameter('waypoints_csv').value
        self.path_topic = self.get_parameter('path_topic').value
<<<<<<< HEAD
=======
        self.scan_topic = self.get_parameter('scan_topic').value
>>>>>>> f374ce0 (Add MPP node, PP local-path tracking, and SLAM-only test launches)
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.horizon_points = int(self.get_parameter('horizon_points').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.loop_path = bool(self.get_parameter('loop_path').value)

        if not self.waypoints_csv:
<<<<<<< HEAD
            raise ValueError('waypoints_csv parameter is required')

        self.global_waypoints: List[Tuple[float, float]] = self.load_waypoints(self.waypoints_csv)
        if len(self.global_waypoints) < 2:
            raise ValueError('Need at least 2 waypoints in CSV')

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
=======
            raise RuntimeError("waypoints_csv parameter required")

        self.global_waypoints: List[Tuple[float, float]] = self.load_waypoints(self.waypoints_csv)
        self.last_nearest_i = 0
        self.latest_scan: Optional[LaserScan] = None

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_cb, 10
        )
>>>>>>> f374ce0 (Add MPP node, PP local-path tracking, and SLAM-only test launches)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

<<<<<<< HEAD
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f'MPP loaded {len(self.global_waypoints)} global waypoints | '
            f'publishing local path to {self.path_topic}'
=======
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )

        self.get_logger().info(
            f"MPP loaded {len(self.global_waypoints)} waypoints"
>>>>>>> f374ce0 (Add MPP node, PP local-path tracking, and SLAM-only test launches)
        )

    def load_waypoints(self, csv_path: str) -> List[Tuple[float, float]]:
        pts = []
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row:
                    continue
<<<<<<< HEAD
                if row[0].startswith('#'):
                    continue
                if len(row) < 2:
                    continue
                x = float(row[0].strip())
                y = float(row[1].strip())
                pts.append((x, y))
        return pts

=======
                if row[0].strip().startswith('#'):
                    continue
                if len(row) < 2:
                    continue
                pts.append((float(row[0]), float(row[1])))
        return pts

    def scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

>>>>>>> f374ce0 (Add MPP node, PP local-path tracking, and SLAM-only test launches)
    def get_current_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
<<<<<<< HEAD
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
=======
            yaw = yaw_from_quat(tf.transform.rotation)
            return x, y, yaw
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    def nearest_index(self, x: float, y: float) -> int:
        n = len(self.global_waypoints)
        best_i = self.last_nearest_i
        best_d2 = float('inf')

        for k in range(min(200, n)):
            i = (self.last_nearest_i + k) % n
            wx, wy = self.global_waypoints[i]
            d2 = (wx - x) ** 2 + (wy - y) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_i = i

        self.last_nearest_i = best_i
        return best_i

    def build_local_path(self, start_idx: int):
>>>>>>> f374ce0 (Add MPP node, PP local-path tracking, and SLAM-only test launches)
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

<<<<<<< HEAD
        x, y = pose
=======
        x, y, yaw = pose
>>>>>>> f374ce0 (Add MPP node, PP local-path tracking, and SLAM-only test launches)
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
<<<<<<< HEAD
            ps.pose.position.z = 0.0
=======
>>>>>>> f374ce0 (Add MPP node, PP local-path tracking, and SLAM-only test launches)
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
