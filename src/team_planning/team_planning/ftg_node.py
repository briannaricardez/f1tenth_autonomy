import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDriveStamped


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('max_speed', 0.7)
        self.declare_parameter('min_speed', 0.3)
        self.declare_parameter('max_steer', 0.35)
        self.declare_parameter('lookahead_dist', 2.0)
        self.declare_parameter('fov_deg', 180.0)
        self.declare_parameter('bubble_radius', 0.6)
        self.declare_parameter('smooth_window', 5)
        self.declare_parameter('gap_threshold', 0.8)
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('steer_slew_rate', 5.0)
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('front_danger_dist', 0.8)
        self.declare_parameter('front_danger_angle_deg', 30.0)

        # Path-bias params
        self.declare_parameter('local_path_topic', '/local_path')
        self.declare_parameter('use_path_bias', True)
        self.declare_parameter('path_lookahead_dist', 1.0)
        self.declare_parameter('path_stale_timeout', 0.5)
        self.declare_parameter('path_bias_weight', 1.5)
        self.declare_parameter('gap_size_weight', 0.5)
        self.declare_parameter('gap_range_weight', 1.0)
        self.declare_parameter('front_danger_gain', 2.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)
        self.fov_deg = float(self.get_parameter('fov_deg').value)
        self.bubble_radius = float(self.get_parameter('bubble_radius').value)
        self.smooth_window = int(self.get_parameter('smooth_window').value)
        self.gap_threshold = float(self.get_parameter('gap_threshold').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.steer_slew_rate = float(self.get_parameter('steer_slew_rate').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.front_danger_dist = float(self.get_parameter('front_danger_dist').value)
        self.front_danger_angle_deg = float(self.get_parameter('front_danger_angle_deg').value)

        self.local_path_topic = self.get_parameter('local_path_topic').value
        self.use_path_bias = bool(self.get_parameter('use_path_bias').value)
        self.path_lookahead_dist = float(self.get_parameter('path_lookahead_dist').value)
        self.path_stale_timeout = float(self.get_parameter('path_stale_timeout').value)
        self.path_bias_weight = float(self.get_parameter('path_bias_weight').value)
        self.gap_size_weight = float(self.get_parameter('gap_size_weight').value)
        self.gap_range_weight = float(self.get_parameter('gap_range_weight').value)
        self.front_danger_gain = float(self.get_parameter('front_danger_gain').value)

        self.pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.path_sub = self.create_subscription(
            Path, self.local_path_topic, self.on_path, 10)

        self.last_steer = 0.0
        self.last_time = self.get_clock().now()

        self.path_target_angle = None
        self.path_stamp = None

        self.get_logger().info(
            f"FTG running | scan={self.scan_topic} drive={self.drive_topic} "
            f"fov={self.fov_deg} bubble={self.bubble_radius} max_speed={self.max_speed} "
            f"front_danger_dist={self.front_danger_dist}m "
            f"use_path_bias={self.use_path_bias} path={self.local_path_topic}"
        )

    def on_path(self, msg: Path):
        if not msg.poses:
            self.path_target_angle = None
            return
        chosen = None
        for ps in msg.poses:
            x = ps.pose.position.x
            y = ps.pose.position.y
            if math.hypot(x, y) >= self.path_lookahead_dist:
                chosen = (x, y)
                break
        if chosen is None:
            ps = msg.poses[-1]
            chosen = (ps.pose.position.x, ps.pose.position.y)
        x, y = chosen
        self.path_target_angle = math.atan2(y, x)
        self.path_stamp = self.get_clock().now()

    def _path_is_fresh(self) -> bool:
        if self.path_target_angle is None or self.path_stamp is None:
            return False
        age = (self.get_clock().now() - self.path_stamp).nanoseconds * 1e-9
        return age < self.path_stale_timeout

    def on_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=self.max_range, posinf=self.max_range, neginf=0.0)
        ranges = np.clip(ranges, self.min_range, self.max_range)

        angles = msg.angle_min + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment
        fov = math.radians(self.fov_deg) / 2.0
        mask = (angles >= -fov) & (angles <= fov)

        r = ranges[mask]
        a = angles[mask]

        if r.size < 10:
            return

        if self.smooth_window > 1:
            w = self.smooth_window
            kernel = np.ones(w, dtype=np.float32) / w
            r = np.convolve(r, kernel, mode='same')

        # Front danger zone check
        front_angle = math.radians(self.front_danger_angle_deg)
        front_mask = np.abs(a) <= front_angle
        front_ranges = r[front_mask]

        if front_ranges.size > 0:
            min_front = float(np.min(front_ranges))
            if min_front < self.front_danger_dist:
                path_fresh = self.use_path_bias and self._path_is_fresh()
                if path_fresh:
                    desired_steer = clamp(
                        self.front_danger_gain * self.path_target_angle,
                        -self.max_steer, self.max_steer
                    )
                    # If path steer is too small (path pointing at obstacle),
                    # fall back to pure gap comparison — pick the side with
                    # more clearance, no arbitrary direction bias
                    if abs(desired_steer) < 0.15:
                        left_mask = a > front_angle
                        right_mask = a < -front_angle
                        left_max = float(np.max(r[left_mask])) if np.any(left_mask) else 0.0
                        right_max = float(np.max(r[right_mask])) if np.any(right_mask) else 0.0
                        if left_max >= right_max:
                            desired_steer = self.max_steer
                        else:
                            desired_steer = -self.max_steer
                else:
                    left_mask = a > front_angle
                    right_mask = a < -front_angle
                    left_max = float(np.max(r[left_mask])) if np.any(left_mask) else 0.0
                    right_max = float(np.max(r[right_mask])) if np.any(right_mask) else 0.0
                    if left_max >= right_max:
                        desired_steer = self.max_steer
                    else:
                        desired_steer = -self.max_steer

                speed = self.min_speed

                now = self.get_clock().now()
                dt = (now - self.last_time).nanoseconds * 1e-9
                if dt <= 0.0:
                    dt = 1.0 / self.publish_hz
                max_delta = self.steer_slew_rate * dt
                steer = clamp(desired_steer, self.last_steer - max_delta, self.last_steer + max_delta)
                self.last_steer = steer
                self.last_time = now

                out = AckermannDriveStamped()
                out.header.stamp = now.to_msg()
                out.drive.steering_angle = float(steer)
                out.drive.speed = float(speed)
                self.pub.publish(out)

                dir_label = 'PATH' if path_fresh else ('LEFT' if desired_steer > 0 else 'RIGHT')
                self.get_logger().warn(
                    f'FRONT DANGER: {min_front:.2f}m | steering {dir_label} '
                    f'({desired_steer:+.2f} rad)',
                    throttle_duration_sec=0.5
                )
                return

        # Normal FTG
        closest_idx = int(np.argmin(r))
        closest_dist = float(r[closest_idx])

        bubble = int(self.bubble_radius / max(msg.angle_increment * max(closest_dist, 0.1), 1e-3))
        start = max(0, closest_idx - bubble)
        end = min(r.size - 1, closest_idx + bubble)
        r_bubbled = r.copy()
        r_bubbled[start:end+1] = self.min_range

        open_mask = r_bubbled > self.gap_threshold
        if not np.any(open_mask):
            target_idx = int(np.argmax(r_bubbled))
        else:
            idx = np.where(open_mask)[0]
            splits = np.where(np.diff(idx) > 1)[0] + 1
            groups = np.split(idx, splits)

            path_fresh = self.use_path_bias and self._path_is_fresh()
            best_score = -float('inf')
            best_target_idx = None
            for grp in groups:
                if grp.size == 0:
                    continue
                grp_size_rad = float(a[grp[-1]] - a[grp[0]])
                grp_max_range = float(np.max(r_bubbled[grp]))
                grp_center_local = int(np.argmax(r_bubbled[grp]))
                grp_center_idx = int(grp[grp_center_local])
                grp_center_angle = float(a[grp_center_idx])

                score = (self.gap_size_weight * grp_size_rad
                         + self.gap_range_weight * grp_max_range)
                if path_fresh:
                    score -= self.path_bias_weight * abs(
                        grp_center_angle - self.path_target_angle)

                if score > best_score:
                    best_score = score
                    best_target_idx = grp_center_idx

            target_idx = best_target_idx if best_target_idx is not None else int(np.argmax(r_bubbled))

        target_angle = float(a[target_idx])
        desired_steer = clamp(target_angle, -self.max_steer, self.max_steer)

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1.0 / self.publish_hz
        max_delta = self.steer_slew_rate * dt
        steer = clamp(desired_steer, self.last_steer - max_delta, self.last_steer + max_delta)
        self.last_steer = steer
        self.last_time = now

        turn_factor = 1.0 - min(1.0, abs(steer) / self.max_steer)
        speed = self.min_speed + (self.max_speed - self.min_speed) * (0.25 + 0.75 * turn_factor)
        speed = clamp(speed, self.min_speed, self.max_speed)

        out = AckermannDriveStamped()
        out.header.stamp = now.to_msg()
        out.drive.steering_angle = float(steer)
        out.drive.speed = float(speed)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = FollowTheGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
