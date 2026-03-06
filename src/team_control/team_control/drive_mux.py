#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String


class DriveMux(Node):
    """
    Drive multiplexer:
      - subscribes to /drive_pp
      - subscribes to /drive_ftg
      - subscribes to /control_mode  ("pp" or "ftg")
      - republishes selected input to /drive
    """

    def __init__(self):
        super().__init__('drive_mux')

        self.declare_parameter('pp_topic', '/drive_pp')
        self.declare_parameter('ftg_topic', '/drive_ftg')
        self.declare_parameter('mode_topic', '/control_mode')
        self.declare_parameter('out_topic', '/drive')
        self.declare_parameter('default_mode', 'pp')

        self.pp_topic = self.get_parameter('pp_topic').value
        self.ftg_topic = self.get_parameter('ftg_topic').value
        self.mode_topic = self.get_parameter('mode_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.mode = self.get_parameter('default_mode').value

        self.last_pp_msg = None
        self.last_ftg_msg = None

        self.pub = self.create_publisher(AckermannDriveStamped, self.out_topic, 10)

        self.pp_sub = self.create_subscription(
            AckermannDriveStamped,
            self.pp_topic,
            self.pp_cb,
            10
        )

        self.ftg_sub = self.create_subscription(
            AckermannDriveStamped,
            self.ftg_topic,
            self.ftg_cb,
            10
        )

        self.mode_sub = self.create_subscription(
            String,
            self.mode_topic,
            self.mode_cb,
            10
        )

        self.get_logger().info(
            f"DriveMux ready | pp={self.pp_topic} ftg={self.ftg_topic} "
            f"mode={self.mode_topic} out={self.out_topic} default_mode={self.mode}"
        )

    def mode_cb(self, msg: String):
        new_mode = msg.data.strip().lower()
        if new_mode not in ['pp', 'ftg']:
            self.get_logger().warn(f"Ignoring invalid control mode: {new_mode}")
            return

        if new_mode != self.mode:
            self.mode = new_mode
            self.get_logger().info(f"DriveMux switched to {self.mode.upper()}")

    def pp_cb(self, msg: AckermannDriveStamped):
        self.last_pp_msg = msg
        if self.mode == 'pp':
            self.pub.publish(msg)

    def ftg_cb(self, msg: AckermannDriveStamped):
        self.last_ftg_msg = msg
        if self.mode == 'ftg':
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = DriveMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
