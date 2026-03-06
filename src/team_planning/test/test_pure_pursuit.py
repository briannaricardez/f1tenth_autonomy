"""Unit tests for Pure Pursuit algorithm logic.

Utility functions are duplicated here to avoid importing from the ROS-dependent
source modules, so these tests can run without rclpy installed.
"""
import math
import pytest


def yaw_from_quat(q):
    """Same as pure_pursuit_node.yaw_from_quat."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(x, lo, hi):
    """Same as pure_pursuit_node.clamp."""
    return max(lo, min(hi, x))


class TestYawFromQuat:
    def test_zero_yaw(self):
        class Q:
            w, x, y, z = 1.0, 0.0, 0.0, 0.0

        assert abs(yaw_from_quat(Q())) < 1e-6

    def test_ninety_degrees(self):
        class Q:
            w = math.cos(math.pi / 4)
            x = 0.0
            y = 0.0
            z = math.sin(math.pi / 4)

        assert abs(yaw_from_quat(Q()) - math.pi / 2) < 1e-6


class TestClamp:
    def test_within_range(self):
        assert clamp(0.2, -0.4, 0.4) == 0.2

    def test_above_range(self):
        assert clamp(0.6, -0.4, 0.4) == 0.4

    def test_below_range(self):
        assert clamp(-0.6, -0.4, 0.4) == -0.4


class TestPurePursuitSteering:
    def test_straight_ahead_target(self):
        """Target directly ahead should give ~zero steering."""
        # Vehicle at origin facing +x, target at (2, 0)
        x_v, y_v = 2.0, 0.0
        wheelbase = 0.33
        Ld2 = x_v ** 2 + y_v ** 2
        curvature = (2.0 * y_v) / Ld2
        steering = math.atan(curvature * wheelbase)
        assert abs(steering) < 1e-6

    def test_target_to_left(self):
        """Target to the left should give positive steering."""
        x_v, y_v = 2.0, 1.0
        wheelbase = 0.33
        Ld2 = x_v ** 2 + y_v ** 2
        curvature = (2.0 * y_v) / Ld2
        steering = math.atan(curvature * wheelbase)
        assert steering > 0

    def test_target_to_right(self):
        """Target to the right should give negative steering."""
        x_v, y_v = 2.0, -1.0
        wheelbase = 0.33
        Ld2 = x_v ** 2 + y_v ** 2
        curvature = (2.0 * y_v) / Ld2
        steering = math.atan(curvature * wheelbase)
        assert steering < 0
