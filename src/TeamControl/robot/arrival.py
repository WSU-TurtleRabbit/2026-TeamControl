import math


def is_close(target_xy: tuple, current_xy: tuple, threshold: float = 50.0) -> bool:
    """Return True if the Euclidean distance between two (x, y) points is under threshold (mm)."""
    dx = target_xy[0] - current_xy[0]
    dy = target_xy[1] - current_xy[1]
    return math.hypot(dx, dy) < threshold


def is_facing_direction(target_theta: float, current_theta: float, threshold: float = 0.05) -> bool:
    """Return True if the shortest angular difference between two angles is under threshold (radians)."""
    diff = (target_theta - current_theta + math.pi) % (2 * math.pi) - math.pi
    return abs(diff) < threshold
