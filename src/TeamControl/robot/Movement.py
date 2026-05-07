import math
import time
from typing import Tuple, Optional

from TeamControl.world.transform_cords import world2robot
from TeamControl.robot import constants as C



def _wrap_angle(a: float) -> float:
    """
    Fold any angle (in radians) into the range (-pi, pi].

    ─── WHY ────────────────────────────────────────────────────────────
    Angles are CIRCULAR: +3.5 rad and -2.78 rad describe the same
    direction. Without wrapping:
      - The PD controller would chase a 3.5 rad error by rotating the
        LONG way instead of the 2.78 rad short way.
      - When the raw angle crosses +/- pi, it jumps discontinuously
        (e.g. 3.14 → -3.13), and the D-term computes a huge false rate
        of change — causing omega to "kick" violently.
    Wrapping enforces "always take the shortest rotation."

    ─── FORMULA ────────────────────────────────────────────────────────
        a  ←  (a + pi) % (2*pi)  -  pi
        if a == -pi : a += 2*pi      # force endpoint into (-pi, pi]

    ─── DERIVATION, step by step ───────────────────────────────────────
    Target output range: (-pi, pi].

    Step 1 — shift up by pi:
        Input can be any real; we want the output symmetric around 0.
        Modulo is easier to reason about on a range that starts at 0,
        so first shift so the target becomes (0, 2*pi].

    Step 2 — modulo 2*pi:
        Python's `%` with a positive divisor always returns a result in
        [0, divisor). So (a + pi) % (2*pi) is in [0, 2*pi). This strips
        off any whole number of 2*pi turns regardless of sign — unlike
        C's `%`, Python's modulo is well-behaved for negatives, so no
        special-case needed (e.g. -3 % 10 == 7 in Python).

    Step 3 — shift back down by pi:
        [0, 2*pi) - pi = [-pi, pi).
        Almost what we want, but -pi is included and +pi is not.

    Step 4 — bump endpoint:
        Only one case differs: exactly -pi. We prefer +pi (same angle,
        canonical choice for the range (-pi, pi]). The `if` handles it.

    ─── WORKED EXAMPLES (pi ≈ 3.14159) ─────────────────────────────────
      input  | a + pi | % 2*pi | -pi    | final
      -------|--------|--------|--------|---------------
       0     |  3.14  |  3.14  |  0.00  |   0
       pi/2  |  4.71  |  4.71  |  1.57  |   pi/2
       pi    |  6.28  |  0.00  | -3.14  |  +pi  (bumped)
      -pi    |  0.00  |  0.00  | -3.14  |  +pi  (bumped)
       3.5   |  6.64  |  0.36  | -2.78  |  -2.78  (short way)
       7     | 10.14  |  3.86  |  0.72  |   0.72  (7 - 2*pi)
      -10    | -6.86  |  5.70  |  2.56  |   2.56  (-10 + 2*2*pi)

    ─── ALTERNATIVE FORMULAS ───────────────────────────────────────────
    Equivalent but slower or less convenient:
      A) while a >  pi: a -= 2*pi;  while a < -pi: a += 2*pi
           Correct but O(n) for huge inputs.
      B) math.atan2(math.sin(a), math.cos(a))
           Correct and numerically stable at extreme inputs, but costs
           two transcendental calls vs. one modulo. Overkill here, since
           inputs are usually already near (-pi, pi] from atan2 upstream.
    """
    # Step 1+2+3: shift, modulo, shift back → result is in [-pi, pi).
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    # Step 4: modulo can land exactly on -pi; map it to +pi so the
    # range is (-pi, pi] (open at -pi, closed at +pi).
    if a <= -math.pi:
        a += 2.0 * math.pi
    return a


class PDController:
    """
    A reusable PD (Proportional-Derivative) controller.

    The control law:
        u(t) = Kp * e(t)  +  Kd * de(t)/dt

    In English:
      - e(t) is how far you are from the goal RIGHT NOW.
      - de/dt is how FAST that error is changing.
      - Kp * e is the "push" term: far from goal → push hard, close → push gently.
        If you only used P, the robot would overshoot and oscillate, because
        at the target e=0 but the robot is still moving.
      - Kd * de/dt is the "brake" term: if the error is shrinking quickly (the
        robot is closing in), this term opposes the push and damps the motion.
        Think of it as the shock absorber in a car suspension — without it,
        you'd bounce up and down forever.

    Works on either:
      - a scalar error (e.g. angle error, in radians), or
      - a 2D vector error (e.g. (ex, ey) position error, in mm).

    First tick: we can't compute de/dt from one sample, so D = 0 and we fall
    back to P-only until a second sample lets us measure the rate.

    Output saturation ("saturates" = hits a hard ceiling and stops responding
    to more input): if |u| would exceed out_limit, it's clipped. For vectors,
    the whole vector is scaled down so direction is preserved.
    """

    def __init__(self, kp: float, kd: float, out_limit: Optional[float] = None):
        self.kp = kp
        self.kd = kd
        self.out_limit = out_limit
        # Previous-sample memory. Needed to compute de/dt = (e - e_prev)/dt.
        self.prev_error = None
        self.prev_t: Optional[float] = None

    def reset(self) -> None:
        """
        Forget history. Call this when the target changes abruptly or the
        robot stops, so the next de/dt isn't computed against a stale error
        (which would produce a huge, spurious D-term "kick").
        """
        self.prev_error = None
        self.prev_t = None

    def update(self, error, now: Optional[float] = None):
        """
        Feed a new error sample, get back the controller output.
        """
        if now is None:
            now = time.monotonic()

        # Accept either scalar or vector error; remember which so we output
        # the matching shape.
        is_vec = isinstance(error, (tuple, list))
        err = tuple(float(e) for e in error) if is_vec else float(error)

        # --- D term: de/dt, the rate of change of error -----------------
        if self.prev_t is None or self.prev_error is None:
            # No history yet → can't compute a rate. Use 0 so the first tick
            # is pure P. D starts contributing on the second call onward.
            d_err = tuple(0.0 for _ in err) if is_vec else 0.0
        else:
            # dt is small but nonzero. Clamp to 1e-6 to avoid divide-by-zero
            # if two updates happen in the same microsecond.
            dt = max(now - self.prev_t, 1e-6)
            if is_vec:
                d_err = tuple((e - pe) / dt for e, pe in zip(err, self.prev_error))
            else:
                d_err = (err - self.prev_error) / dt

        # Store for next tick's derivative calculation.
        self.prev_error = err
        self.prev_t = now

        # --- Compute u = Kp*e + Kd*de/dt, then saturate -----------------
        if is_vec:
            u = tuple(self.kp * e + self.kd * de for e, de in zip(err, d_err))
            if self.out_limit is not None:
                # For a vector: if the magnitude is too big, scale BOTH
                # components down by the same factor. This preserves the
                # direction of motion — we just drive slower, not sideways.
                mag = math.sqrt(sum(x * x for x in u))
                if mag > self.out_limit and mag > 0:
                    scale = self.out_limit / mag
                    u = tuple(x * scale for x in u)
            return u

        # Scalar path: symmetric clip to [-out_limit, +out_limit].
        u = self.kp * err + self.kd * d_err
        if self.out_limit is not None:
            if u > self.out_limit:
                u = self.out_limit
            elif u < -self.out_limit:
                u = -self.out_limit
        return u


class RobotMovement:
    """
    Per-robot movement controller: wraps two PDControllers (one for turning,
    one for driving) plus a bundle of pure-geometry helpers.

    IMPORTANT: hold ONE instance per robot for the lifetime of its control
    loop. The PD controllers remember the previous tick's error so they can
    compute de/dt. If you create a fresh RobotMovement every frame, the D
    term is always 0 and you effectively only get P control.

    Pure-geometry helpers (threshold_zone, behind_ball_point, shooting_pos,
    calculate_target_position) remain @staticmethod — they do not touch the
    controller state, so sharing is fine.

    Gains default to constants.py (loaded from tuning.json). Pass overrides
    to __init__ for per-robot tuning.
    """

    def __init__(self,
                 turn_kp: Optional[float] = None,
                 turn_kd: Optional[float] = None,
                 linear_kp: Optional[float] = None,
                 linear_kd: Optional[float] = None):
        self.angular_pd = PDController(
            kp=C.TURN_KP if turn_kp is None else turn_kp,
            kd=C.TURN_KD if turn_kd is None else turn_kd,
            out_limit=C.MAX_W,
        )
        self.linear_pd = PDController(
            kp=C.LINEAR_KP if linear_kp is None else linear_kp,
            kd=C.LINEAR_KD if linear_kd is None else linear_kd,
            out_limit=C.MAX_SPEED,
        )

    def reset(self) -> None:
        """Clear PD history. Call when target changes abruptly or robot stops."""
        self.angular_pd.reset()
        self.linear_pd.reset()

    def velocity_to_target(self,
                           robot_pos: Tuple[float, float, float],
                           target: Tuple[float, float],
                           turning_target: Optional[Tuple[float, float]] = None,
                           speed: Optional[float] = None,
                           stop_threshold: float = 150.0
                           ) -> Tuple[float, float, float]:
        """
        Top-level driver: given where the robot is in the world, compute the
        (vx, vy, w) command that drives it toward `target` while facing
        `turning_target`.

        Math / why:
          - Our robot is holonomic (can translate in any direction AND spin
            at the same time), so we solve the linear and angular problems
            INDEPENDENTLY and hand back both answers.
          - world2robot() converts a world-frame point into the robot's own
            local frame. We do this because the robot's motors only know
            "forward/sideways/spin" — they have no idea what "world +X"
            means. Once everything is in the robot frame, +x is "in front
            of me", +y is "to my left", so vx/vy/w map straight to motor
            commands.
        """
        if robot_pos is None or target is None:
            raise ValueError("Robot pos or Target is None")

        # Where is the target relative to me, right now? (mm in robot frame)
        trans_target = world2robot(robot_pos, target)
        vx, vy = self.go_to_target(trans_target,
                                   speed=speed,
                                   stop_threshold=stop_threshold)
        # Separate turning target lets the robot, e.g., strafe sideways
        # while keeping the kicker pointed at the ball.
        if turning_target is None:
            w = 0.0
        else:
            trans_turn = world2robot(robot_pos, turning_target)
            w = self.turn_to_target(trans_turn)

        return vx, vy, w

    def turn_to_target(self,
                       target: Optional[Tuple[float, float]] = None,
                       epsilon: Optional[float] = None
                       ) -> float:
        """
        Return ω (rad/s, angular velocity) that rotates the robot so its
        kicker side points at `target`.

        Math / why:
          - In the robot frame, the kicker faces +x. So "facing the target"
            means the target lies on the +x axis, i.e. atan2(y, x) = 0.
          - atan2(ty, tx) gives the signed angle from +x to the target:
                target in front    → angle  ≈ 0       (no turn)
                target to the left → angle  = +pi/2   (turn CCW)
                target to the right→ angle  = -pi/2   (turn CW)
                target behind      → angle  = ±pi     (turn either way)
          - We wrap to (-pi, pi] so the controller always picks the short
            rotation direction. Without this, an angle of +4 rad would
            rotate 4 rad the wrong way instead of -2.28 rad the right way.
          - Deadband (|angle| < epsilon): if we're close enough to facing
            the target, output 0 and RESET the PD. Without the reset, the
            error suddenly dropping to 0 would cause a huge negative D-term
            on the next tick and make the robot twitch.
          - PD output is already clamped to ±MAX_W inside the PDController.
        """
        if target is None:
            self.angular_pd.reset()
            return 0.0

        eps = C.ANGLE_EPSILON if epsilon is None else epsilon
        # atan2 tells us WHICH WAY to turn; _wrap_angle guarantees the
        # shortest rotation.
        angle = _wrap_angle(math.atan2(target[1], target[0]))

        if abs(angle) < eps:
            # Good enough — don't jitter on sub-degree errors.
            self.angular_pd.reset()
            return 0.0

        return self.angular_pd.update(angle)

    def go_to_target(self,
                     target_pos: Optional[Tuple[float, float]],
                     speed: Optional[float] = None,
                     stop_threshold: Optional[float] = None
                     ) -> Tuple[float, float]:
        """
        Return (vx, vy) in m/s that drives the robot toward `target_pos`
        (expressed in the ROBOT frame, in mm).

        Math / why:
          - Position error IS the target position in the robot frame, because
            the robot is always at (0,0) in its own frame. So e = (tx, ty).
          - PD runs on the 2D vector error:
                (vx, vy) = Kp·(tx, ty)  +  Kd·d(tx, ty)/dt
            This pushes the robot straight toward the target, and the D-term
            brakes as the target "rushes in" (de/dt is large and negative,
            in the sense of shrinking error magnitude).
          - Unit note: error is in mm, output is in m/s. That's why Kp is
            small (~0.002). A 500 mm error × 0.002 = 1.0 m/s — i.e. the
            P-term SATURATES at 500 mm (hits MAX_SPEED), and beyond that
            the controller just pushes at max speed.
          - Hard safety cap via threshold_zone: even if PD wants to charge
            in fast, we cap the speed inside the kicker zone (0 speed) and
            dribble zone (20 % speed). This matches the old bucket behaviour
            and prevents the robot from slamming the ball.
          - stop_threshold: if we're ALREADY within this many mm of the
            target, stop entirely (don't bother correcting sub-threshold
            noise). Resetting PD here avoids a D-kick when we later move off.
        """
        if target_pos is None:
            return 0.0, 0.0

        max_speed = C.MAX_SPEED if speed is None else speed
        stop = 0.0 if stop_threshold is None else stop_threshold

        # math.hypot = sqrt(x² + y²) — the straight-line distance to target.
        dist = math.hypot(target_pos[0], target_pos[1])
        if dist <= stop:
            # Already close enough; don't chase noise.
            self.linear_pd.reset()
            return 0.0, 0.0

        # Apply distance-based speed cap (kicker zone / dribble zone / open).
        zone_cap = self.threshold_zone(dist, max_speed)
        if zone_cap <= 0.0:
            # We're right on top of the target (inside kicker zone).
            self.linear_pd.reset()
            return 0.0, 0.0

        # Run PD on the 2D error vector (which IS the target in robot frame).
        vx, vy = self.linear_pd.update((target_pos[0], target_pos[1]))

        # If PD would exceed the zone cap, scale the vector down preserving
        # direction — we still drive toward the target, just slower.
        mag = math.hypot(vx, vy)
        if mag > zone_cap and mag > 0:
            s = zone_cap / mag
            vx *= s
            vy *= s
        return vx, vy

    # Backward-compat alias for legacy callers.
    go_To_Target = go_to_target

    # ─── pure geometry helpers (no controller state) ──────────────────

    @staticmethod
    def threshold_zone(distance: float, max_speed: float) -> float:
        """
        Three-zone speed cap based on distance-to-target (mm):
            dist <   70 mm : 0 speed  (kicker zone — we're at the target)
            dist <  400 mm : 20 % of max_speed (dribble zone — ease in)
            otherwise      : full max_speed (open-field zone)

        Why: a pure PD output can still be too aggressive right next to the
        ball. The zones are a belt-and-braces cap that guarantees safe
        behaviour even if gains are mistuned. Think of it as lane-limits on
        top of the PD "driver".
        """
        if distance < C.KICKER_ZONE:
            return 0.0
        if distance < C.DRIBBLE_ZONE:
            return max_speed * C.DRIBBLE_SPEED_FRAC
        return max_speed

    @staticmethod
    def behind_ball_point(ball, goal, buffer_radius):
        """
        Return a point on the ball→goal line, `buffer_radius` mm behind the
        ball (on the OPPOSITE side from the goal).

        Math:
          1) direction vector from ball to goal:  d = goal - ball
          2) normalise to unit length:            d_hat = d / |d|
          3) step BACKWARD from the ball:         p = ball - d_hat * radius

        Why: to shoot the ball toward the goal, the robot needs to approach
        from the side opposite the goal. This function gives the "lineup"
        point to aim for before the actual kick.

            goal ●────── d_hat ──────● ball ←─ buffer ─● behind_ball_point
                                                         (robot heads here)
        """
        bx, by = ball
        gx, gy = goal
        dx = gx - bx
        dy = gy - by
        d = math.hypot(dx, dy)
        if d == 0:
            raise ValueError("Ball and goal cannot be at the same point")
        # Normalise: turn (dx, dy) into a unit vector pointing ball → goal.
        dx /= d
        dy /= d
        # Step the opposite direction from the ball.
        return bx - dx * buffer_radius, by - dy * buffer_radius

    @staticmethod
    def shooting_pos(ball_pos, shootingTarget, robot_offset: float = 200.0):
        """
        Same idea as `behind_ball_point`: returns a standoff point behind
        the ball along the ball→shootingTarget line, `robot_offset` mm
        behind the ball. The only difference is the parameter names.
        """
        # Direction from ball toward where we want to shoot.
        dx = float(shootingTarget[0]) - float(ball_pos[0])
        dy = float(shootingTarget[1]) - float(ball_pos[1])
        norm = math.hypot(dx, dy)
        if norm == 0:
            # Degenerate case: ball IS the target. Return the ball position.
            return (float(ball_pos[0]), float(ball_pos[1]))
        # Normalise to unit vector, then step backward from the ball.
        dx /= norm
        dy /= norm
        return (float(ball_pos[0]) - robot_offset * dx,
                float(ball_pos[1]) - robot_offset * dy)

    @staticmethod
    def calculate_target_position(target, ball, robot_offset):
        """
        WARNING: this function is mathematically different from the two
        above and looks buggy — it subtracts `robot_offset * norm` (a
        scalar times a magnitude) rather than `robot_offset * unit_vector`.
        That means it steps backward by `offset * |d|`, a distance that
        grows with how far the goal is. Preserved as-is for legacy callers;
        prefer `behind_ball_point` / `shooting_pos` for new code.
        """
        dx = float(target[0]) - float(ball[0])
        dy = float(target[1]) - float(ball[1])
        norm = math.hypot(dx, dy)
        bx = float(ball[0]) - robot_offset * norm
        by = float(ball[1]) - robot_offset * norm
        return (bx, by)


# ─── Module-level registry for callers that can't easily hold an instance ────

_MOVEMENT_BY_ROBOT: dict = {}


def get_movement(robot_id, is_yellow: bool = True) -> RobotMovement:
    """
    Fetch (or lazily create) a persistent RobotMovement for (is_yellow, robot_id).

    Use this from free functions that run per-tick but can't easily own the
    instance themselves (e.g. module-level `go_to_ball_and_shoot(...)`).
    Long-lived classes should just hold `self.movement = RobotMovement()`.
    """
    key = (bool(is_yellow), robot_id)
    rm = _MOVEMENT_BY_ROBOT.get(key)
    if rm is None:
        rm = RobotMovement()
        _MOVEMENT_BY_ROBOT[key] = rm
    return rm


class Follow_path:
    def __init__(self):
        self.path = None

    def update_path(self, path: list):
        """
        Adds a path to follow
        Prams --> path as a list[x position , y position]
        """
        self.path = path

    def get_point(self, robot_pos: tuple[float, float]):
        '''
        Gets the fist point of a given path, will remove the first point once reached
        Prams --> the robot position [x position , y position]
        '''
        if self.path == None:
            print("Please update the path before you call this function")
        else:
            diff = math.hypot(self.path[0][0] - robot_pos[0],
                              self.path[0][1] - robot_pos[1])

            if len(self.path) == 1:
                return self.path
            elif diff < 0.5:
                del self.path[0]
                return self.path[0]
            else:
                return self.path[0]


class calculateBallVelocity:
    """
    step() returns a 2-tuple:
      (distance, speed)
    where:
      - distance : float            # world-frame distance to the ball
      - speed    : Optional[float]  # chosen speed (m/s), or None if unreachable
    """

    def __init__(self, time_threshold: float = 1.5):
        self.time_threshold = time_threshold
        self.speed_levels = [0.02, 0.04, 0.06, 0.08, 0.10]

    def _pick_speed(self, distance: float) -> Optional[float]:
        best = None
        for v in self.speed_levels:
            if distance / v <= self.time_threshold:
                if best is None or v < best:
                    best = v
        return best

    def step(
        self,
        robot_pose: Tuple[float, float, float],
        ball_pos:   Tuple[float, float]
    ) -> Tuple[float, Optional[float]]:
        dx = ball_pos[0] - robot_pose[0]
        dy = ball_pos[1] - robot_pose[1]
        distance = math.hypot(dx, dy)
        speed = self._pick_speed(distance)
        return distance, speed
