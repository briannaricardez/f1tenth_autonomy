#!/usr/bin/env python3
"""
Drive Arbiter.

Continuously blends Pure Pursuit (/drive_pp) and Follow-the-Gap (/drive_ftg)
into a single /drive command. Replaces the previous binary safety_supervisor +
drive_mux pair, which caused step changes in the steering command at the
PP <-> FTG handoff.

Blend coefficient alpha in [0, 1] is derived from minimum front distance:
  - min_front >= blend_far   -> alpha = 0   (pure PP)
  - min_front <= blend_near  -> alpha = 1   (pure FTG)
  - linear ramp between
Alpha itself is low-pass filtered to suppress single-scan spikes.

Steering = (1 - alpha) * pp_steer + alpha * ftg_steer.
Speed    = min(pp_speed, ftg_speed) when alpha > 0 (NOT a convex blend),
           additionally capped by an alpha-curved speed ceiling.

A speed-dependent low-pass + slew limit is applied to the final steering
output: stronger filtering / tighter slew at high speed, lighter at low
speed. This kills both the high-frequency FTG twitch and any residual step
change from the blend transition.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class DriveArbiter(Node):
    def __init__(self):
        super().__init__('drive_arbiter')

        self.declare_parameter('pp_topic', '/drive_pp')
        self.declare_parameter('ftg_topic', '/drive_ftg')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('out_topic', '/drive')
        self.declare_parameter('blend_topic', '/control_blend')

        self.declare_parameter('arbiter_rate', 50.0)
        self.declare_parameter('msg_stale_timeout', 0.2)
        self.declare_parameter('scan_stale_timeout', 0.5)

        # Blend curve (min_front in meters)
        self.declare_parameter('blend_far', 1.5)
        self.declare_parameter('blend_near', 0.5)
        self.declare_parameter('front_half_angle_deg', 20.0)
        self.declare_parameter('alpha_smooth_beta', 0.4)

        # Speed shaping
        self.declare_parameter('max_speed', 4.0)
        self.declare_parameter('emergency_speed', 0.4)

        # Output smoothing (speed-dependent)
        self.declare_parameter('beta_low', 0.25)
        self.declare_parameter('beta_high', 0.7)
        self.declare_parameter('slew_low', 2.5)
        self.declare_parameter('slew_high', 6.0)
        self.declare_parameter('v_ref', 4.0)

        self.pp_topic = self.get_parameter('pp_topic').value
        self.ftg_topic = self.get_parameter('ftg_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.out_topic = self.get_parameter('out_topic').value
        self.blend_topic = self.get_parameter('blend_topic').value

        self.arbiter_rate = float(self.get_parameter('arbiter_rate').value)
        self.msg_stale_timeout = float(self.get_parameter('msg_stale_timeout').value)
        self.scan_stale_timeout = float(self.get_parameter('scan_stale_timeout').value)

        self.blend_far = float(self.get_parameter('blend_far').value)
        self.blend_near = float(self.get_parameter('blend_near').value)
        self.front_half_angle_deg = float(self.get_parameter('front_half_angle_deg').value)
        self.alpha_smooth_beta = float(self.get_parameter('alpha_smooth_beta').value)

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.emergency_speed = float(self.get_parameter('emergency_speed').value)

        self.beta_low = float(self.get_parameter('beta_low').value)
        self.beta_high = float(self.get_parameter('beta_high').value)
        self.slew_low = float(self.get_parameter('slew_low').value)
        self.slew_high = float(self.get_parameter('slew_high').value)
        self.v_ref = float(self.get_parameter('v_ref').value)

        self.last_pp_msg = None
        self.last_pp_time = None
        self.last_ftg_msg = None
        self.last_ftg_time = None
        self.last_scan_time = None

        self.current_alpha = 0.0
        self.prev_out_steer = 0.0
        self.prev_tick_time = self.get_clock().now()

        self.pub = self.create_publisher(AckermannDriveStamped, self.out_topic, 10)
        self.blend_pub = self.create_publisher(Float32, self.blend_topic, 10)

        self.pp_sub = self.create_subscription(
            AckermannDriveStamped, self.pp_topic, self.pp_cb, 10)
        self.ftg_sub = self.create_subscription(
            AckermannDriveStamped, self.ftg_topic, self.ftg_cb, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_cb, 10)

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.arbiter_rate, self.tick)

        self.get_logger().info(
            f"DriveArbiter ready | pp={self.pp_topic} ftg={self.ftg_topic} "
            f"out={self.out_topic} blend_far={self.blend_far} blend_near={self.blend_near} "
            f"max_speed={self.max_speed} v_ref={self.v_ref}"
        )

    def pp_cb(self, msg: AckermannDriveStamped):
        self.last_pp_msg = msg
        self.last_pp_time = self.get_clock().now()

    def ftg_cb(self, msg: AckermannDriveStamped):
        self.last_ftg_msg = msg
        self.last_ftg_time = self.get_clock().now()

    def scan_cb(self, msg: LaserScan):
        if not msg.ranges:
            return
        self.last_scan_time = self.get_clock().now()

        center_angle = 0.0
        half_angle = math.radians(self.front_half_angle_deg)
        min_front = float('inf')

        for i, rng in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if angle < center_angle - half_angle or angle > center_angle + half_angle:
                continue
            if math.isinf(rng) or math.isnan(rng):
                continue
            if rng < min_front:
                min_front = rng

        if min_front == float('inf'):
            return

        denom = max(self.blend_far - self.blend_near, 1e-3)
        raw_alpha = clamp((self.blend_far - min_front) / denom, 0.0, 1.0)
        self.current_alpha = (
            (1.0 - self.alpha_smooth_beta) * self.current_alpha
            + self.alpha_smooth_beta * raw_alpha
        )

    def _fresh(self, stamp) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age < self.msg_stale_timeout

    def tick(self):
        now = self.get_clock().now()
        if (now - self.start_time).nanoseconds * 1e-9 < 1.5:
            return  # startup lockout
        dt = (now - self.prev_tick_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1.0 / self.arbiter_rate
        self.prev_tick_time = now

        pp_fresh = self._fresh(self.last_pp_time)
        ftg_fresh = self._fresh(self.last_ftg_time)

        if not pp_fresh and not ftg_fresh:
            return

        # Watchdog: stale scan -> assume worst case
        scan_stale = (
            self.last_scan_time is None
            or (now - self.last_scan_time).nanoseconds * 1e-9 > self.scan_stale_timeout
        )

        if not pp_fresh:
            alpha = 1.0
        elif not ftg_fresh:
            alpha = 0.0
        elif scan_stale:
            alpha = 1.0
        else:
            alpha = self.current_alpha

        if pp_fresh:
            pp_steer = float(self.last_pp_msg.drive.steering_angle)
            pp_speed = float(self.last_pp_msg.drive.speed)
        else:
            pp_steer = 0.0
            pp_speed = 0.0

        if ftg_fresh:
            ftg_steer = float(self.last_ftg_msg.drive.steering_angle)
            ftg_speed = float(self.last_ftg_msg.drive.speed)
        else:
            ftg_steer = 0.0
            ftg_speed = 0.0

        blended_steer = (1.0 - alpha) * pp_steer + alpha * ftg_steer

        if alpha <= 0.0:
            blended_speed = pp_speed
        elif alpha >= 1.0:
            blended_speed = ftg_speed
        else:
            blended_speed = min(pp_speed, ftg_speed)

        speed_cap = self.max_speed - alpha * (self.max_speed - self.emergency_speed)
        blended_speed = min(blended_speed, speed_cap)

        v_norm = clamp(blended_speed / max(self.v_ref, 1e-3), 0.0, 1.0)
        beta = self.beta_low + (self.beta_high - self.beta_low) * (1.0 - v_norm)
        slew = self.slew_low + (self.slew_high - self.slew_low) * (1.0 - v_norm)

        smoothed = (1.0 - beta) * self.prev_out_steer + beta * blended_steer
        max_delta = slew * dt
        final_steer = clamp(
            smoothed,
            self.prev_out_steer - max_delta,
            self.prev_out_steer + max_delta
        )
        self.prev_out_steer = final_steer

        out = AckermannDriveStamped()
        out.header.stamp = now.to_msg()
        out.drive.steering_angle = float(final_steer)
        out.drive.speed = float(blended_speed)
        self.pub.publish(out)

        blend_msg = Float32()
        blend_msg.data = float(alpha)
        self.blend_pub.publish(blend_msg)


def main():
    rclpy.init()
    node = DriveArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
