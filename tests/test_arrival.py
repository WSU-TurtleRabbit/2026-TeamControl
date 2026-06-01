import math
from TeamControl.robot.arrival import is_close, is_facing_direction


# ── is_close ──────────────────────────────────────────────────────────────────

def test_close_clearly_within():
    assert is_close((0, 0), (0, 30), threshold=50) is True

def test_close_clearly_outside():
    assert is_close((0, 0), (0, 200), threshold=50) is False

def test_close_exactly_on_threshold():
    # distance == threshold is NOT close (strict <)
    assert is_close((0, 0), (50, 0), threshold=50) is False

def test_close_diagonal():
    # 3-4-5 triangle: distance = 50mm
    assert is_close((0, 0), (30, 40), threshold=51) is True

def test_close_same_point():
    assert is_close((100, 200), (100, 200), threshold=1) is True


# ── is_facing_direction ───────────────────────────────────────────────────────

def test_facing_clearly_aligned():
    assert is_facing_direction(1.0, 1.0, threshold=0.05) is True

def test_facing_clearly_off():
    assert is_facing_direction(0.0, 1.0, threshold=0.05) is False

def test_facing_wrap_across_pi():
    # 3.1 and -3.1 are only ~0.08 rad apart via the short way around
    assert is_facing_direction(3.1, -3.1, threshold=0.1) is True

def test_facing_wrap_negative():
    assert is_facing_direction(-math.pi, math.pi, threshold=0.01) is True

def test_facing_exactly_on_threshold():
    # diff == threshold is NOT facing (strict <)
    assert is_facing_direction(0.0, 0.05, threshold=0.05) is False


if __name__ == "__main__":
    tests = [
        test_close_clearly_within,
        test_close_clearly_outside,
        test_close_exactly_on_threshold,
        test_close_diagonal,
        test_close_same_point,
        test_facing_clearly_aligned,
        test_facing_clearly_off,
        test_facing_wrap_across_pi,
        test_facing_wrap_negative,
        test_facing_exactly_on_threshold,
    ]
    for t in tests:
        try:
            t()
            print(f"[PASS] {t.__name__}")
        except AssertionError as e:
            print(f"[FAIL] {t.__name__}: {e}")
