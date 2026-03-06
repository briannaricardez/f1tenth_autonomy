"""Unit tests for Follow-The-Gap algorithm logic."""
import math
import numpy as np
import pytest


def find_largest_gap(ranges, threshold):
    """Extract gap-finding logic from FTG for testing."""
    open_mask = ranges > threshold
    if not np.any(open_mask):
        return int(np.argmax(ranges)), False

    idx = np.where(open_mask)[0]
    splits = np.where(np.diff(idx) > 1)[0] + 1
    groups = np.split(idx, splits)
    best = max(groups, key=len)
    target_idx = int(best[np.argmax(ranges[best])])
    return target_idx, True


def apply_bubble(ranges, closest_idx, bubble_size, min_range):
    """Extract bubble masking logic from FTG for testing."""
    r = ranges.copy()
    start = max(0, closest_idx - bubble_size)
    end = min(r.size - 1, closest_idx + bubble_size)
    r[start:end + 1] = min_range
    return r


class TestGapFinding:
    def test_single_gap(self):
        """Gap in the middle, walls on both sides."""
        ranges = np.array([0.5] * 20 + [3.0] * 40 + [0.5] * 20, dtype=np.float32)
        idx, found = find_largest_gap(ranges, threshold=1.2)
        assert found
        assert 20 <= idx < 60  # target should be inside the gap

    def test_two_gaps_picks_largest(self):
        """Two gaps of different sizes; largest should win."""
        ranges = np.array(
            [0.5] * 10 + [3.0] * 10 + [0.5] * 10 + [3.0] * 30 + [0.5] * 10,
            dtype=np.float32,
        )
        idx, found = find_largest_gap(ranges, threshold=1.2)
        assert found
        assert 30 <= idx < 60  # should be in the larger (30-wide) gap

    def test_no_gap(self):
        """All ranges below threshold; fallback to max range."""
        ranges = np.array([0.5, 0.8, 1.0, 0.7, 0.9], dtype=np.float32)
        idx, found = find_largest_gap(ranges, threshold=1.2)
        assert not found
        assert idx == 2  # argmax -> index of 1.0

    def test_all_open(self):
        """Everything above threshold; entire array is one gap."""
        ranges = np.array([5.0] * 50, dtype=np.float32)
        idx, found = find_largest_gap(ranges, threshold=1.2)
        assert found
        assert 0 <= idx < 50


class TestBubbleMasking:
    def test_bubble_zeros_around_closest(self):
        ranges = np.array([3.0] * 20, dtype=np.float32)
        closest = 10
        result = apply_bubble(ranges, closest, bubble_size=3, min_range=0.05)
        # Indices 7..13 should be zeroed
        assert np.all(result[7:14] == 0.05)
        # Others untouched
        assert np.all(result[:7] == 3.0)
        assert np.all(result[14:] == 3.0)

    def test_bubble_at_edge(self):
        ranges = np.array([3.0] * 10, dtype=np.float32)
        result = apply_bubble(ranges, closest_idx=0, bubble_size=3, min_range=0.05)
        assert np.all(result[0:4] == 0.05)
        assert np.all(result[4:] == 3.0)


def clamp(x, lo, hi):
    """Same implementation as ftg_node.clamp, duplicated to avoid rclpy import."""
    return max(lo, min(hi, x))


class TestSteeringClamp:
    def test_clamp_above(self):
        assert clamp(0.5, -0.35, 0.35) == 0.35

    def test_clamp_below(self):
        assert clamp(-0.5, -0.35, 0.35) == -0.35

    def test_clamp_within(self):
        assert clamp(0.1, -0.35, 0.35) == 0.1
