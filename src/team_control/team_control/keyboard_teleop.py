#!/usr/bin/env python3
import sys
import select
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

HELP = """
Keyboard Teleop (Ackermann) - SIMPLE / RELIABLE

Tap keys (no key-repeat needed):
  w / Up Arrow    : increase speed (latched)
  s / Down Arrow  : decrease speed (latched)
  a / Left Arrow  : steer left (latched)
  d / Right Arrow : steer right (latched)
  space           : stop (speed=0)
  r               : reset speed & steer
  q               : quit

Optional:
- auto_center=True makes steering return to 0 slowly by itself.
- Click THIS terminal before pressing keys.
"""

def get_key(timeout=0.1):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if not rlist:
        return None

    c1 = sys.stdin.read(1)
    if c1 != '\x1b':
        return c1

    if select.select([sys.stdin], [], [], 0.0)[0]:
        c2 = sys.stdin.read(1)
    else:
        return c1

    if c2 == '[' and select.select([sys.stdin], [], [], 0.0)[0]:
        c3 = sys.stdin.read(1)
        return f'\x1b[{c3}'
    return c1 + c2

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('speed_step', 0.15)
        self.declare_parameter('steer_step', 0.02)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('max_steer', 0.22)
        self.declare_parameter('invert_steer', True)

        # NEW: auto-centering steering (does NOT rely on key repeat)
        self.declare_parameter('auto_center', True)
        self.declare_parameter('center_rate', 0.6)   # rad/s toward 0
        self.declare_parameter('publish_hz', 30.0)

        self.drive_topic = self.get_parameter('drive_topic').value
        self.speed_step = float(self.get_parameter('speed_step').value)
        self.steer_step = float(self.get_parameter('steer_step').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.invert_steer = bool(self.get_parameter('invert_steer').value)

        self.auto_center = bool(self.get_parameter('auto_center').value)
        self.center_rate = float(self.get_parameter('center_rate').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)

        self.speed = 0.0
        self.steer = 0.0
        self.last = time.time()

        self.pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_hz, self.on_timer)

        self.get_logger().info(HELP.strip())
        self.get_logger().info(
            f"Publishing to {self.drive_topic} | speed_step={self.speed_step} steer_step={self.steer_step} "
            f"max_speed={self.max_speed} max_steer={self.max_steer} auto_center={self.auto_center}"
        )

    def on_timer(self):
        now = time.time()
        dt = now - self.last
        self.last = now

        # auto-center steering gently
        if self.auto_center and abs(self.steer) > 1e-4:
            step = self.center_rate * max(dt, 0.0)
            if self.steer > 0:
                self.steer = max(0.0, self.steer - step)
            else:
                self.steer = min(0.0, self.steer + step)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(self.speed)
        msg.drive.steering_angle = float(self.steer)
        self.pub.publish(msg)

    def handle_key(self, k):
        if k is None:
            return

        # speed
        if k in ('w', 'W', '\x1b[A'):
            self.speed = clamp(self.speed + self.speed_step, -self.max_speed, self.max_speed)
        elif k in ('s', 'S', '\x1b[B'):
            self.speed = clamp(self.speed - self.speed_step, -self.max_speed, self.max_speed)

        # steer (logical left/right)
        elif k in ('a', 'A', '\x1b[D'):
            delta = -self.steer_step
            if self.invert_steer:
                delta = -delta
            self.steer = clamp(self.steer + delta, -self.max_steer, self.max_steer)
        elif k in ('d', 'D', '\x1b[C'):
            delta = +self.steer_step
            if self.invert_steer:
                delta = -delta
            self.steer = clamp(self.steer + delta, -self.max_steer, self.max_steer)

        elif k == ' ':
            self.speed = 0.0
        elif k in ('r', 'R'):
            self.speed = 0.0
            self.steer = 0.0
        elif k in ('q', 'Q'):
            raise KeyboardInterrupt

        self.get_logger().info(f"speed={self.speed:.2f} steer={self.steer:.3f}", throttle_duration_sec=0.2)

def main():
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    rclpy.init()
    node = KeyboardTeleop()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = get_key(timeout=0.05)
            try:
                node.handle_key(key)
            except KeyboardInterrupt:
                break
    finally:
        node.get_logger().info("Exiting keyboard teleop.")
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)

if __name__ == '__main__':
    main()
