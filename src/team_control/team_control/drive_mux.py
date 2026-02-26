#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class DriveMux(Node):
    """
    Minimal drive mux:
      subscribes to /drive_ftg
      republishes to /drive
    """
    def __init__(self):
        super().__init__('drive_mux')

        self.declare_parameter('in_topic', '/drive_ftg')
        self.declare_parameter('out_topic', '/drive')

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value

        self.pub = self.create_publisher(AckermannDriveStamped, out_topic, 10)
        self.sub = self.create_subscription(AckermannDriveStamped, in_topic, self.cb, 10)

        self.get_logger().info(f"DriveMux forwarding {in_topic} -> {out_topic}")

    def cb(self, msg: AckermannDriveStamped):
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
