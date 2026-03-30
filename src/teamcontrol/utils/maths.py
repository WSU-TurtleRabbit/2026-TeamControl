"""
Common math helpers used everywhere.
Nothing fancy, just saves typing numpy boilerplate.
"""

import math
import numpy as np


def dist(p1, p2) -> float:
    """Euclidean distance between two points (x, y)."""
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.hypot(dx, dy)


def angle_to(src, dst) -> float:
    """Angle in radians from src to dst. Returns [-pi, pi]."""
    return math.atan2(dst[1] - src[1], dst[0] - src[0])


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def clamp(val, low, high):
    """Clamp a value between low and high."""
    return max(low, min(high, val))


def lerp(a, b, t: float):
    """Linear interpolation between a and b."""
    return a + (b - a) * t


def rotate_point(x, y, angle):
    """Rotate a 2D point around the origin by angle (radians)."""
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return (x * cos_a - y * sin_a, x * sin_a + y * cos_a)


def velocity_from_positions(positions: list, dt: float = 0.016):
    """
    Estimate velocity from a list of recent (x, y) positions.
    Uses simple finite difference on the last two points.
    Returns (vx, vy) in units/sec.
    """
    if len(positions) < 2:
        return (0.0, 0.0)

    p1 = positions[-2]
    p2 = positions[-1]
    vx = (p2[0] - p1[0]) / dt
    vy = (p2[1] - p1[1]) / dt
    return (vx, vy)
