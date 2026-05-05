#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class SafetySupervisor(Node):
    """
    Publishes control mode:
      - 'pp'  = Pure Pursuit
      - 'ftg' = Follow The Gap

    Logic:
      - switch to FTG if front obstacle is too close
      - switch back to PP once front clears
    """

    def __init__(self):
        super().__init__('safety_supervisor')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('mode_topic', '/control_mode')
        self.declare_parameter('ftg_trigger_dist', 0.8)
        self.declare_parameter('pp_return_dist', 1.0)
        self.declare_parameter('front_half_angle_deg', 20.0)
        self.declare_parameter('default_mode', 'pp')

        self.scan_topic = self.get_parameter('scan_topic').value
        self.mode_topic = self.get_parameter('mode_topic').value
        self.ftg_trigger_dist = float(self.get_parameter('ftg_trigger_dist').value)
        self.pp_return_dist = float(self.get_parameter('pp_return_dist').value)
        self.front_half_angle_deg = float(self.get_parameter('front_half_angle_deg').value)
        self.mode = self.get_parameter('default_mode').value.strip().lower()

        self.mode_pub = self.create_publisher(String, self.mode_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_cb,
            10
        )

        self.get_logger().info(
            f"SafetySupervisor ready | scan={self.scan_topic} mode_topic={self.mode_topic} "
            f"trigger={self.ftg_trigger_dist:.2f} return={self.pp_return_dist:.2f} "
            f"default_mode={self.mode}"
        )

    def publish_mode(self):
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)

    def scan_cb(self, msg: LaserScan):
        if not msg.ranges:
            return

        center_angle = 0.0
        half_angle = math.radians(self.front_half_angle_deg)

        min_front = float('inf')

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            if angle < center_angle - half_angle or angle > center_angle + half_angle:
                continue

            if math.isinf(r) or math.isnan(r):
                continue

            if r < min_front:
                min_front = r

        if min_front == float('inf'):
            self.publish_mode()
            return

        prev_mode = self.mode

        if self.mode == 'pp' and min_front < self.ftg_trigger_dist:
            self.mode = 'ftg'
        elif self.mode == 'ftg' and min_front > self.pp_return_dist:
            self.mode = 'pp'

        if self.mode != prev_mode:
            self.get_logger().info(
                f"Mode switch: {prev_mode.upper()} -> {self.mode.upper()} "
                f"(front_min={min_front:.2f} m)"
            )

        self.publish_mode()


def main():
    rclpy.init()
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
