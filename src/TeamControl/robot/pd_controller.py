import math
import time
from typing import Optional


class PDController:
    """
    A reusable PD (Proportional-Derivative) controller.

    Control law:  u(t) = Kp * e(t) + Kd * de(t)/dt

    Works on either a scalar error (e.g. angle, radians) or a 2-D vector
    error (e.g. (ex, ey) position, mm).  First tick: D = 0 (pure P) until
    a second sample is available.

    Output saturation: if |u| would exceed out_limit it is clipped.  For
    vectors the whole vector is scaled so direction is preserved.
    """

    def __init__(self, kp: float, kd: float, out_limit: Optional[float] = None):
        self.kp = kp
        self.kd = kd
        self.out_limit = out_limit
        self.prev_error = None
        self.prev_t: Optional[float] = None

    def reset(self) -> None:
        """Clear history. Call when target changes abruptly or robot stops."""
        self.prev_error = None
        self.prev_t = None

    def update(self, error, now: Optional[float] = None):
        """Feed a new error sample, return the controller output."""
        if now is None:
            now = time.monotonic()

        is_vec = isinstance(error, (tuple, list))
        err = tuple(float(e) for e in error) if is_vec else float(error)

        if self.prev_t is None or self.prev_error is None:
            d_err = tuple(0.0 for _ in err) if is_vec else 0.0
        else:
            dt = max(now - self.prev_t, 1e-6)
            if is_vec:
                d_err = tuple((e - pe) / dt for e, pe in zip(err, self.prev_error))
            else:
                d_err = (err - self.prev_error) / dt

        self.prev_error = err
        self.prev_t = now

        if is_vec:
            u = tuple(self.kp * e + self.kd * de for e, de in zip(err, d_err))
            if self.out_limit is not None:
                mag = math.sqrt(sum(x * x for x in u))
                if mag > self.out_limit and mag > 0:
                    scale = self.out_limit / mag
                    u = tuple(x * scale for x in u)
            return u

        u = self.kp * err + self.kd * d_err
        if self.out_limit is not None:
            u = max(-self.out_limit, min(self.out_limit, u))
        return u
