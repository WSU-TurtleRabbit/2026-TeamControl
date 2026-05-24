"""Coordinator — role assignment, game-state dispatch, and per-robot tree dispatch.

Pipeline:
    Snapshot → Coordinator.tick() → list[Intent]

Game-state handling
-------------------
Before running the normal role trees the Coordinator checks the current
GamePhase from the Snapshot's RefereeState and may override behaviour:

    HALTED / HALF_TIME  → no intents produced (robots coast to a stop)
    STOPPED             → every robot holds position; any robot within
                          STOP_BALL_CLEARANCE of the ball is nudged away
    PREPARE_KICKOFF     → robots move to pre-kickoff positions
    KICKOFF             → attacker goes to ball at centre, others to support spots
    FREE_KICK           → attacker goes to ball, others hold positions
    BALL_PLACEMENT      → all robots keep STOP_BALL_CLEARANCE from ball
    PENALTY_SHOOT       → attacker to penalty spot, others behind ball line
    PENALTY_DEFEND      → goalie tracks ball on goal line, others hold
    RUNNING             → normal role-tree dispatch (existing behaviour)
"""
from __future__ import annotations

import math
from typing import Any

from TeamControl.bt.contracts.blackboard import RobotBlackboard, RoleType
from TeamControl.bt.contracts.intent import Intent, IntentMove
from TeamControl.bt.contracts.snapshot import GamePhase, Snapshot

# ---------------------------------------------------------------------------
# Role assignment — fixed by robot ID
# ---------------------------------------------------------------------------
# index 0 → GOALIE, 1-2 → DEFENDER, 3-4 → SUPPORTER, 5 → ATTACKER
ROLE_ASSIGNMENT: dict[int, RoleType] = {
    0: RoleType.GOALIE,
    1: RoleType.DEFENDER,
    2: RoleType.DEFENDER,
    3: RoleType.SUPPORTER,
    4: RoleType.SUPPORTER,
    5: RoleType.ATTACKER,
}

# ---------------------------------------------------------------------------
# Field constants (metres — matches existing BT skill constants)
# ---------------------------------------------------------------------------
# Legal SSL threshold is 0.5m. We use 0.55m as a safety buffer so robots
# are never right on the limit. 0.5m is what the rules enforce, not 0.55m.
STOP_BALL_CLEARANCE: float = 0.55
LEGAL_BALL_CLEARANCE: float = 0.50   # actual SSL rule threshold

OWN_GOAL: tuple[float, float] = (-4.5, 0.0)
OPP_GOAL: tuple[float, float] = (4.5, 0.0)
OWN_GOAL_LINE_X: float = -4.5

# Div B field (9m × 6m): goal line at x=±4.5m, penalty mark 1m from goal line.
# §2.1.3: penalty mark is 1m from goal line toward centre → x = 4.5 - 1.0 = 3.5m.
# NOTE: ideally read this from wm.field.penalty_spot_from_field_line_dist if available.
PENALTY_SPOT: tuple[float, float] = (3.5, 0.0)

# PREPARE_KICKOFF positions — all robots in own half (negative-x when us_positive=True).
# Rule §5.3.2: one attacker is allowed ANYWHERE inside the centre circle (radius=0.5m).
# We place attacker at (0, 0) — centre of the circle — to be as close to ball as allowed.
# Ball is at centre, placed by human referee before kickoff command. Robots must NOT touch it.
KICKOFF_POSITIONS: dict[int, tuple[float, float]] = {
    0: (-4.0, 0.0),    # goalie — in front of own goal
    1: (-2.0, -1.5),   # defender left
    2: (-2.0,  1.5),   # defender right
    3: (-1.0, -1.0),   # supporter left
    4: (-1.0,  1.0),   # supporter right
    5: (0.0,   0.0),   # attacker — at centre, inside circle (§5.3.2 allows this)
}

# PENALTY_SHOOT: attacker at penalty spot, ALL others ≥ 1m behind ball (x ≤ 2.5).
# Our goalie is NOT the defending keeper — opponent's keeper defends their goal.
# All our non-attacker robots just need to be 1m behind the ball.
PENALTY_SHOOT_POSITIONS: dict[int, tuple[float, float]] = {
    0: (2.0,  0.5),    # our goalie — behind ball
    1: (2.0, -1.5),
    2: (2.0,  1.5),
    3: (2.0, -0.5),
    4: (2.0,  0.5),
    5: PENALTY_SPOT,   # attacker shoots
}

# Positions for penalty defend: goalie on goal line, others in own half
PENALTY_DEFEND_POSITIONS: dict[int, tuple[float, float]] = {
    0: (-4.5, 0.0),    # goalie on goal line (tracks ball y dynamically)
    1: (-2.0, -1.0),
    2: (-2.0,  1.0),
    3: (-1.5, -0.5),
    4: (-1.5,  0.5),
    5: (-1.0,  0.0),
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _find_robot(snapshot: Snapshot, robot_id: int):
    for r in snapshot.own_robots:
        if r.robot_id == robot_id:
            return r
    return None


def _nudge_away_from_ball(
    robot_pos: tuple[float, float],
    ball_pos: tuple[float, float],
    clearance: float,
) -> tuple[float, float]:
    """Return a target position *clearance* metres from the ball, away from robot."""
    dx = robot_pos[0] - ball_pos[0]
    dy = robot_pos[1] - ball_pos[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return (robot_pos[0] - clearance, robot_pos[1])
    scale = clearance / dist
    return (ball_pos[0] + dx * scale, ball_pos[1] + dy * scale)


def _dist_to_segment(
    point: tuple[float, float],
    seg_a: tuple[float, float],
    seg_b: tuple[float, float],
) -> float:
    """Return the shortest distance from *point* to line segment A→B."""
    ax, ay = seg_a
    bx, by = seg_b
    px, py = point
    abx, aby = bx - ax, by - ay
    ab_len_sq = abx * abx + aby * aby
    if ab_len_sq < 1e-12:
        return math.hypot(px - ax, py - ay)
    # Project point onto the line, clamped to [0, 1]
    t = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab_len_sq))
    closest_x = ax + t * abx
    closest_y = ay + t * aby
    return math.hypot(px - closest_x, py - closest_y)


def _nudge_away_from_segment(
    robot_pos: tuple[float, float],
    seg_a: tuple[float, float],
    seg_b: tuple[float, float],
    clearance: float,
) -> tuple[float, float]:
    """Push *robot_pos* perpendicularly away from segment A→B to *clearance* distance."""
    ax, ay = seg_a
    bx, by = seg_b
    px, py = robot_pos
    abx, aby = bx - ax, by - ay
    ab_len_sq = abx * abx + aby * aby
    if ab_len_sq < 1e-12:
        return _nudge_away_from_ball(robot_pos, seg_a, clearance)
    t = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab_len_sq))
    closest_x = ax + t * abx
    closest_y = ay + t * aby
    # Direction from closest point on segment to robot
    dx = px - closest_x
    dy = py - closest_y
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        # Robot is exactly on the line — nudge perpendicular to the segment
        perp_x, perp_y = -aby, abx
        perp_len = math.hypot(perp_x, perp_y)
        return (closest_x + perp_x / perp_len * clearance,
                closest_y + perp_y / perp_len * clearance)
    return (closest_x + dx / dist * clearance, closest_y + dy / dist * clearance)


# ---------------------------------------------------------------------------
# Coordinator
# ---------------------------------------------------------------------------

def _mirror(pos: tuple[float, float]) -> tuple[float, float]:
    """Flip a position to the other half of the field (negate x)."""
    return (-pos[0], pos[1])


def _mirror_positions(
    positions: dict[int, tuple[float, float]],
) -> dict[int, tuple[float, float]]:
    return {k: _mirror(v) for k, v in positions.items()}


class Coordinator:
    """Assigns roles, checks game phase, and dispatches role trees each tick.

    Parameters
    ----------
    trees:
        Mapping of RoleType → tree object.
    us_positive:
        True if we attack toward the positive-x goal (default).
        When False all fixed position maps are mirrored so robots stay
        in the correct half regardless of field orientation.
    """

    def __init__(self, trees: dict[RoleType, Any], us_positive: bool = True) -> None:
        self.trees = trees
        self.blackboards: dict[int, RobotBlackboard] = {}
        # Pre-compute mirrored position tables if we attack the negative goal.
        if us_positive:
            self._kickoff_pos = KICKOFF_POSITIONS
            self._penalty_shoot_pos = PENALTY_SHOOT_POSITIONS
            self._penalty_defend_pos = PENALTY_DEFEND_POSITIONS
        else:
            self._kickoff_pos = _mirror_positions(KICKOFF_POSITIONS)
            self._penalty_shoot_pos = _mirror_positions(PENALTY_SHOOT_POSITIONS)
            self._penalty_defend_pos = _mirror_positions(PENALTY_DEFEND_POSITIONS)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def tick(self, snapshot: Snapshot, robot_ids: list[int]) -> list[Intent]:
        """Tick all robots present in the snapshot.

        Checks the current GamePhase first. For halted/set-piece states a
        dedicated handler writes intents directly onto each robot's blackboard
        instead of running the normal role trees.
        """
        self._ensure_blackboards(snapshot, robot_ids)

        phase = snapshot.referee_state.game_phase

        if phase in (GamePhase.HALTED, GamePhase.HALF_TIME):
            # Robots must not move — produce no intents so the dispatcher
            # lets existing commands time out and robots coast to zero.
            return []

        if phase == GamePhase.STOPPED:
            return self._handle_stopped(snapshot, robot_ids)

        if phase == GamePhase.BALL_PLACEMENT:
            return self._handle_ball_placement(snapshot, robot_ids)

        if phase == GamePhase.PREPARE_KICKOFF:
            return self._handle_fixed_positions(snapshot, robot_ids, self._kickoff_pos)

        if phase == GamePhase.KICKOFF:
            return self._handle_kickoff(snapshot, robot_ids)

        if phase == GamePhase.FREE_KICK:
            return self._handle_free_kick(snapshot, robot_ids)

        if phase == GamePhase.PENALTY_SHOOT:
            return self._handle_fixed_positions(snapshot, robot_ids, self._penalty_shoot_pos)

        if phase == GamePhase.PENALTY_DEFEND:
            return self._handle_penalty_defend(snapshot, robot_ids)

        # RUNNING — normal role-tree dispatch.
        return self._normal_tick(snapshot, robot_ids)

    # ------------------------------------------------------------------
    # Game-phase handlers
    # ------------------------------------------------------------------

    def _handle_stopped(self, snapshot: Snapshot, robot_ids: list[int]) -> list[Intent]:
        """STOPPED / BALL_PLACEMENT: every robot holds position unless it is
        within STOP_BALL_CLEARANCE of the ball, in which case it backs away."""
        intents: list[Intent] = []
        for robot_id in robot_ids:
            robot = _find_robot(snapshot, robot_id)
            if robot is None:
                continue
            bb = self.blackboards[robot_id]
            dist = math.hypot(
                robot.position[0] - snapshot.ball_position[0],
                robot.position[1] - snapshot.ball_position[1],
            )
            if dist < STOP_BALL_CLEARANCE:
                # Too close — nudge away to the clearance boundary.
                target = _nudge_away_from_ball(
                    robot.position, snapshot.ball_position, STOP_BALL_CLEARANCE
                )
            else:
                # Already clear — hold current position.
                target = robot.position
            # max_speed=1.4 enforces the SSL < 1.5 m/s rule during STOPPED
            bb.current_intent = IntentMove(target_pos=target, target_orientation=None, max_speed=1.4)
            intents.append(bb.current_intent)
        return intents

    def _handle_fixed_positions(
        self,
        snapshot: Snapshot,
        robot_ids: list[int],
        positions: dict[int, tuple[float, float]],
    ) -> list[Intent]:
        """Move every robot to its designated position from *positions* map."""
        intents: list[Intent] = []
        for robot_id in robot_ids:
            if _find_robot(snapshot, robot_id) is None:
                continue
            bb = self.blackboards[robot_id]
            target = positions.get(robot_id, KICKOFF_POSITIONS.get(robot_id, (0.0, 0.0)))
            bb.current_intent = IntentMove(target_pos=target, target_orientation=None)
            intents.append(bb.current_intent)
        return intents

    def _handle_kickoff(self, snapshot: Snapshot, robot_ids: list[int]) -> list[Intent]:
        """KICKOFF: attacker moves to ball at centre; others hold kickoff spots."""
        intents: list[Intent] = []
        for robot_id in robot_ids:
            if _find_robot(snapshot, robot_id) is None:
                continue
            bb = self.blackboards[robot_id]
            if ROLE_ASSIGNMENT.get(robot_id) == RoleType.ATTACKER:
                # Drive to the ball (which should be at centre).
                bb.current_intent = IntentMove(
                    target_pos=snapshot.ball_position, target_orientation=None
                )
            else:
                target = self._kickoff_pos.get(robot_id, (0.0, 0.0))
                bb.current_intent = IntentMove(target_pos=target, target_orientation=None)
            intents.append(bb.current_intent)
        return intents

    def _handle_ball_placement(
        self, snapshot: Snapshot, robot_ids: list[int]
    ) -> list[Intent]:
        """BALL_PLACEMENT: attacker moves ball to designated target.

        Placer (attacker, robot 5):
          - If far from ball     → IntentMove to ball
          - If at ball           → IntentDribble to placement target
          - If ball at target    → IntentMove away (§5.2: placer must clear
                                   ≥0.05m after placing for our next free kick)

        All others:
          - Must stay ≥ 0.5 m from the ball AND from the line between
            ball and target (§8.4.3). Nudged away if too close to either.
          - Speed capped at 1.4 m/s.
        """
        target = snapshot.referee_state.ball_placement_pos
        if target is None:
            return self._handle_stopped(snapshot, robot_ids)

        ball = snapshot.ball_position
        # Check if ball has reached the target (within 0.15m — SSL success threshold).
        ball_at_target = math.hypot(ball[0] - target[0], ball[1] - target[1]) < 0.15
        intents: list[Intent] = []

        for robot_id in robot_ids:
            robot = _find_robot(snapshot, robot_id)
            if robot is None:
                continue
            bb = self.blackboards[robot_id]

            if ROLE_ASSIGNMENT.get(robot_id) == RoleType.ATTACKER:
                dist_to_ball = math.hypot(
                    robot.position[0] - ball[0],
                    robot.position[1] - ball[1],
                )
                if ball_at_target:
                    # Ball placed — clear to ≥0.05m (use LEGAL_BALL_CLEARANCE
                    # to ensure we don't interfere with the next free kick).
                    clear_target = _nudge_away_from_ball(
                        robot.position, ball, LEGAL_BALL_CLEARANCE
                    )
                    bb.current_intent = IntentMove(
                        target_pos=clear_target, target_orientation=None
                    )
                elif dist_to_ball > 0.25:
                    bb.current_intent = IntentMove(
                        target_pos=ball, target_orientation=None
                    )
                else:
                    from TeamControl.bt.contracts.intent import IntentDribble
                    bb.current_intent = IntentDribble(target_pos=target)
            else:
                # Non-placer: keep clear of ball and the ball→target line.
                pos = robot.position
                dist_ball = math.hypot(pos[0] - ball[0], pos[1] - ball[1])
                dist_line = _dist_to_segment(pos, ball, target)

                if dist_ball < STOP_BALL_CLEARANCE:
                    move_target = _nudge_away_from_ball(pos, ball, STOP_BALL_CLEARANCE)
                elif dist_line < STOP_BALL_CLEARANCE:
                    move_target = _nudge_away_from_segment(pos, ball, target, STOP_BALL_CLEARANCE)
                else:
                    move_target = pos  # already clear — hold position

                bb.current_intent = IntentMove(
                    target_pos=move_target, target_orientation=None, max_speed=1.4
                )

            intents.append(bb.current_intent)
        return intents

    def _handle_free_kick(self, snapshot: Snapshot, robot_ids: list[int]) -> list[Intent]:
        """FREE_KICK: attacker drives to ball; others hold current positions."""
        intents: list[Intent] = []
        for robot_id in robot_ids:
            robot = _find_robot(snapshot, robot_id)
            if robot is None:
                continue
            bb = self.blackboards[robot_id]
            if ROLE_ASSIGNMENT.get(robot_id) == RoleType.ATTACKER:
                bb.current_intent = IntentMove(
                    target_pos=snapshot.ball_position, target_orientation=None
                )
            else:
                bb.current_intent = IntentMove(
                    target_pos=robot.position, target_orientation=None
                )
            intents.append(bb.current_intent)
        return intents

    def _handle_penalty_defend(
        self, snapshot: Snapshot, robot_ids: list[int]
    ) -> list[Intent]:
        """PENALTY_DEFEND: goalie tracks ball on goal line; others hold."""
        intents: list[Intent] = []
        for robot_id in robot_ids:
            if _find_robot(snapshot, robot_id) is None:
                continue
            bb = self.blackboards[robot_id]
            if ROLE_ASSIGNMENT.get(robot_id) == RoleType.GOALIE:
                # Stay on goal line, track ball's y coordinate.
                target = (OWN_GOAL_LINE_X, snapshot.ball_position[1])
            else:
                target = self._penalty_defend_pos.get(robot_id, (-1.0, 0.0))
            bb.current_intent = IntentMove(target_pos=target, target_orientation=None)
            intents.append(bb.current_intent)
        return intents

    # ------------------------------------------------------------------
    # Normal role-tree dispatch (RUNNING state)
    # ------------------------------------------------------------------

    def _normal_tick(self, snapshot: Snapshot, robot_ids: list[int]) -> list[Intent]:
        """Run each robot's role tree — the original coordinator behaviour."""
        snapshot_ids: set[int] = {r.robot_id for r in snapshot.own_robots}
        intents: list[Intent] = []

        for robot_id in robot_ids:
            if robot_id not in snapshot_ids:
                continue

            bb = self.blackboards[robot_id]
            bb.last_intent = bb.current_intent
            bb.current_intent = None

            tree = self.trees[bb.current_role]
            if hasattr(tree, "set_snapshot") and hasattr(tree, "tick"):
                tree.set_snapshot(snapshot)
                tree.tick(bb)
            else:
                if hasattr(tree, "_blackboard_ref"):
                    tree._blackboard_ref[0] = bb
                tree.tick_once()

            if bb.current_intent is not None:
                intents.append(bb.current_intent)

        return intents

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _ensure_blackboards(self, snapshot: Snapshot, robot_ids: list[int]) -> None:
        """Create blackboards for any robot we haven't seen before."""
        for robot_id in robot_ids:
            if robot_id not in self.blackboards:
                role = ROLE_ASSIGNMENT.get(robot_id, RoleType.SUPPORTER)
                self.blackboards[robot_id] = RobotBlackboard(
                    robot_id=robot_id,
                    current_role=role,
                )
            else:
                self.blackboards[robot_id].current_role = ROLE_ASSIGNMENT.get(
                    robot_id, RoleType.SUPPORTER
                )
