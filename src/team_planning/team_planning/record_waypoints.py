#!/usr/bin/env python3

import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class RecordWaypoints(Node):
    def __init__(self):
        super().__init__('record_waypoints')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('out_csv', '/tmp/waypoints.csv')
        self.declare_parameter('min_dist', 0.20)
        self.declare_parameter('auto_stop_on_loop', True)
        self.declare_parameter('closure_radius', 0.60)
        self.declare_parameter('min_lap_distance', 20.0)

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.out_csv = self.get_parameter('out_csv').get_parameter_value().string_value
        self.min_dist = self.get_parameter('min_dist').get_parameter_value().double_value
        self.auto_stop_on_loop = self.get_parameter('auto_stop_on_loop').get_parameter_value().bool_value
        self.closure_radius = self.get_parameter('closure_radius').get_parameter_value().double_value
        self.min_lap_distance = self.get_parameter('min_lap_distance').get_parameter_value().double_value

        self.points = []
        self.start_point = None
        self.last_point = None
        self.total_distance = 0.0
        self.saved = False
        self.shutdown_requested = False

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.get_logger().info(f'Recording waypoints from {self.odom_topic}')
        self.get_logger().info(f'Output CSV: {self.out_csv}')
        if self.auto_stop_on_loop:
            self.get_logger().info(
                'Auto-stop enabled. Recorder will save when the car returns near the start '
                f'(closure_radius={self.closure_radius:.2f} m, '
                f'min_lap_distance={self.min_lap_distance:.2f} m).'
            )
        else:
            self.get_logger().info('Drive a full lap, then press Ctrl+C here to save.')

    @staticmethod
    def dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def save_csv(self):
        if self.saved:
            return

        out_path = Path(self.out_csv)
        out_path.parent.mkdir(parents=True, exist_ok=True)

        with out_path.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['# x', 'y'])
            for x, y in self.points:
                writer.writerow([f'{x:.6f}', f'{y:.6f}'])

        self.saved = True
        self.get_logger().info(f'Saved {len(self.points)} waypoints to {self.out_csv}')

    def maybe_finish_loop(self, current_point):
        if not self.auto_stop_on_loop:
            return

        if self.start_point is None or len(self.points) < 10:
            return

        closure_dist = self.dist(current_point, self.start_point)

        if self.total_distance >= self.min_lap_distance and closure_dist <= self.closure_radius:
            self.get_logger().info(
                f'Loop closure detected: current point is {closure_dist:.3f} m from start '
                f'after {self.total_distance:.3f} m of travel.'
            )
            self.save_csv()
            self.shutdown_requested = True

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        current_point = (x, y)

        if self.start_point is None:
            self.start_point = current_point
            self.last_point = current_point
            self.points.append(current_point)
            self.get_logger().info(
                f'Start point recorded at x={x:.3f}, y={y:.3f}'
            )
            return

        step_dist = self.dist(current_point, self.last_point)

        if step_dist < self.min_dist:
            self.maybe_finish_loop(current_point)
            return

        self.total_distance += step_dist
        self.points.append(current_point)
        self.last_point = current_point

        if len(self.points) % 50 == 0:
            closure_dist = self.dist(current_point, self.start_point)
            self.get_logger().info(
                f'Recorded {len(self.points)} points | '
                f'path_length={self.total_distance:.2f} m | '
                f'dist_to_start={closure_dist:.2f} m'
            )

        self.maybe_finish_loop(current_point)


def main(args=None):
    rclpy.init(args=args)
    node = RecordWaypoints()

    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received. Saving recorded waypoints...')
    finally:
        if len(node.points) > 0 and not node.saved:
            node.save_csv()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
