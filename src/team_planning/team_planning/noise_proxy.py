#!/usr/bin/env python3
"""
noise_proxy.py — Sensor noise injection for F1Tenth stack robustness testing.

Subscribes to the simulator's clean /scan and /odom, injects configurable noise, and republishes
on /scan_noisy and /odom_noisy. Algorithm nodes should remap their inputs to these noisy topics when testing.

Usage:
    ros2 run team_planning noise_proxy --ros-args \
        -p lidar_stddev:=0.02 \
        -p lidar_dropout_prob:=0.05 \
        -p odom_pos_stddev:=0.01 \
        -p odom_yaw_stddev:=0.005

Then remap algorithm nodes:
    --ros-args -r /scan:=/scan_noisy -r /odom:=/odom_noisy
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math


class NoiseProxy(Node):
    def __init__(self):
        super().__init__('noise_proxy')

        # ── Lidar noise parameters ─────────────────────────────────────────
        # Gaussian range noise std (metres). 0.02 = ~2cm
        self.declare_parameter('lidar_stddev', 0.02)
        # Per-beam dropout probability (beam returns max_range instead of real value)
        self.declare_parameter('lidar_dropout_prob', 0.02)
        # Probability of a spurious short-range return (simulates glass/mirror)
        self.declare_parameter('lidar_speckle_prob', 0.01)
        # Clamp noisy ranges to [range_min, range_max]
        self.declare_parameter('lidar_clamp', True)

        # ── Odometry noise parameters ──────────────────────────────────────
        # Gaussian XY position noise std (metres)
        self.declare_parameter('odom_pos_stddev', 0.005)
        # Gaussian yaw noise std (radians). 0.005 rad ≈ 0.29 deg
        self.declare_parameter('odom_yaw_stddev', 0.003)
        # Gaussian linear velocity noise std (m/s)
        self.declare_parameter('odom_vel_stddev', 0.01)

        # ── Noise mode ─────────────────────────────────────────────────────
        # 'gaussian'  — classic Gaussian range noise only
        # 'dropout'   — random beam dropouts only
        # 'combined'  — Gaussian + dropout + speckle (most realistic)
        # 'severe'    — high noise for stress-testing
        self.declare_parameter('noise_mode', 'combined')

        self._read_params()

        # Lidar pub/sub
        self._scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10)
        self._scan_pub = self.create_publisher(
            LaserScan, '/scan_noisy', 10)

        # Odometry pub/sub
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)
        self._odom_pub = self.create_publisher(
            Odometry, '/odom_noisy', 10)

        self.get_logger().info(
            f'Noise proxy active | mode={self._mode} | '
            f'lidar_std={self._lidar_std:.3f}m | '
            f'dropout={self._dropout_prob:.2%} | '
            f'odom_pos_std={self._odom_pos_std:.4f}m | '
            f'odom_yaw_std={math.degrees(self._odom_yaw_std):.3f}deg'
        )

    # ──────────────────────────────────────────────────────────────────────
    def _read_params(self):
        self._lidar_std    = self.get_parameter('lidar_stddev').value
        self._dropout_prob = self.get_parameter('lidar_dropout_prob').value
        self._speckle_prob = self.get_parameter('lidar_speckle_prob').value
        self._lidar_clamp  = self.get_parameter('lidar_clamp').value
        self._odom_pos_std = self.get_parameter('odom_pos_stddev').value
        self._odom_yaw_std = self.get_parameter('odom_yaw_stddev').value
        self._odom_vel_std = self.get_parameter('odom_vel_stddev').value
        self._mode         = self.get_parameter('noise_mode').value

        if self._mode == 'severe':
            self._lidar_std    = max(self._lidar_std, 0.08)
            self._dropout_prob = max(self._dropout_prob, 0.15)
            self._speckle_prob = max(self._speckle_prob, 0.05)
            self._odom_pos_std = max(self._odom_pos_std, 0.03)
            self._odom_yaw_std = max(self._odom_yaw_std, 0.02)
            self.get_logger().warn('SEVERE noise mode — expect degraded performance!')

    # ──────────────────────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        n = len(ranges)

        # Replace inf/nan with max_range for cleaner noise application
        bad = ~np.isfinite(ranges)
        ranges[bad] = msg.range_max

        if self._mode in ('gaussian', 'combined', 'severe'):
            # Gaussian range noise
            noise = np.random.normal(0.0, self._lidar_std, n).astype(np.float32)
            ranges += noise

        if self._mode in ('dropout', 'combined', 'severe'):
            # Random beam dropouts → return max_range (unseen obstacle)
            drop_mask = np.random.random(n) < self._dropout_prob
            ranges[drop_mask] = msg.range_max

        if self._mode in ('combined', 'severe'):
            # Spurious short-range returns (simulates glass, sensor glitch)
            speckle_mask = np.random.random(n) < self._speckle_prob
            if speckle_mask.any():
                # Place speckle returns at 5-30% of range_max (looks like a wall)
                ranges[speckle_mask] = np.random.uniform(
                    msg.range_min * 2,
                    msg.range_max * 0.30,
                    speckle_mask.sum()
                ).astype(np.float32)

        # Restore original inf/nan values for truly invalid beams
        ranges[bad] = msg.ranges[np.where(bad)[0][0]] if bad.any() else msg.range_max

        if self._lidar_clamp:
            ranges = np.clip(ranges, msg.range_min, msg.range_max)

        noisy_msg = LaserScan()
        noisy_msg.header         = msg.header
        noisy_msg.angle_min      = msg.angle_min
        noisy_msg.angle_max      = msg.angle_max
        noisy_msg.angle_increment = msg.angle_increment
        noisy_msg.time_increment = msg.time_increment
        noisy_msg.scan_time      = msg.scan_time
        noisy_msg.range_min      = msg.range_min
        noisy_msg.range_max      = msg.range_max
        noisy_msg.ranges         = ranges.tolist()
        noisy_msg.intensities    = msg.intensities  # pass through unchanged

        self._scan_pub.publish(noisy_msg)

    # ──────────────────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        noisy_msg = Odometry()
        noisy_msg.header         = msg.header
        noisy_msg.child_frame_id = msg.child_frame_id

        # ── Position noise ──
        noisy_msg.pose.pose.position.x = (
            msg.pose.pose.position.x +
            np.random.normal(0.0, self._odom_pos_std))
        noisy_msg.pose.pose.position.y = (
            msg.pose.pose.position.y +
            np.random.normal(0.0, self._odom_pos_std))
        noisy_msg.pose.pose.position.z = msg.pose.pose.position.z  # unchanged

        # ── Yaw noise (applied to quaternion) ──
        q = msg.pose.pose.orientation
        # Extract yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        noisy_yaw = yaw + np.random.normal(0.0, self._odom_yaw_std)

        # Convert back to quaternion (roll=pitch=0)
        noisy_msg.pose.pose.orientation.x = 0.0
        noisy_msg.pose.pose.orientation.y = 0.0
        noisy_msg.pose.pose.orientation.z = math.sin(noisy_yaw / 2.0)
        noisy_msg.pose.pose.orientation.w = math.cos(noisy_yaw / 2.0)

        # Pass covariance through unchanged (SLAM toolbox uses this)
        noisy_msg.pose.covariance  = msg.pose.covariance
        noisy_msg.twist.covariance = msg.twist.covariance

        # ── Velocity noise ──
        noisy_msg.twist.twist.linear.x = (
            msg.twist.twist.linear.x +
            np.random.normal(0.0, self._odom_vel_std))
        noisy_msg.twist.twist.linear.y = msg.twist.twist.linear.y
        noisy_msg.twist.twist.angular.z = msg.twist.twist.angular.z

        self._odom_pub.publish(noisy_msg)


# ──────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = NoiseProxy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

