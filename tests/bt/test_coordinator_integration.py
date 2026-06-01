"""Coordinator full-wiring integration tests — T017, R004.

Validates that the Coordinator correctly dispatches all four role trees
(Attacker, Defender, Supporter, Goalie) and collects a valid Intent for
each robot in the snapshot.

Pipeline under test:
    Snapshot + robot_ids → Coordinator.tick() → list[Intent]

All four trees use the wrapper protocol: set_snapshot() + tick(blackboard).
"""
from __future__ import annotations

import pytest

from TeamControl.bt.contracts.blackboard import RobotBlackboard, RoleType
from TeamControl.bt.contracts.intent import (
    Intent,
    IntentDribble,
    IntentKick,
    IntentMove,
    IntentOrient,
    IntentPass,
    IntentReceive,
)
from TeamControl.bt.contracts.snapshot import GamePhase, RefereeState, RobotState, Snapshot
from TeamControl.bt.coordinator import Coordinator, ROLE_ASSIGNMENT
from TeamControl.bt.trees.attacker import AttackerTree
from TeamControl.bt.trees.defender import DefenderTree
from TeamControl.bt.trees.goalie import GoalieTree
from TeamControl.bt.trees.supporter import SupporterTree

# ---------------------------------------------------------------------------
# Robot IDs per ROLE_ASSIGNMENT
# ---------------------------------------------------------------------------
_GOALIE_ID = 0
_DEFENDER_IDS = (1, 2)
_SUPPORTER_IDS = (3, 4)
_ATTACKER_ID = 5
_ALL_ROBOT_IDS = [_GOALIE_ID, *_DEFENDER_IDS, *_SUPPORTER_IDS, _ATTACKER_ID]


# ---------------------------------------------------------------------------
# Snapshot factory
# ---------------------------------------------------------------------------

def _make_full_snapshot(
    ball_pos: tuple[float, float] = (2.0, 0.0),
    ball_velocity: tuple[float, float] = (0.0, 0.0),
) -> Snapshot:
    """Build a Snapshot containing all six robots at plausible positions."""
    own_robots = [
        RobotState(robot_id=_GOALIE_ID,      position=(-4.0, 0.0), orientation=0.0),
        RobotState(robot_id=_DEFENDER_IDS[0], position=(-2.5, 1.0), orientation=0.0),
        RobotState(robot_id=_DEFENDER_IDS[1], position=(-2.5,-1.0), orientation=0.0),
        RobotState(robot_id=_SUPPORTER_IDS[0], position=(1.0, 2.0),  orientation=0.0),
        RobotState(robot_id=_SUPPORTER_IDS[1], position=(1.0,-2.0),  orientation=0.0),
        RobotState(robot_id=_ATTACKER_ID,      position=(0.0, 0.0),  orientation=0.0),
    ]
    return Snapshot(
        ball_position=ball_pos,
        ball_velocity=ball_velocity,
        own_robots=own_robots,
        opponent_robots=[],
        referee_state=RefereeState(game_phase=GamePhase.RUNNING, score=(0, 0)),
    )


def _make_coordinator() -> Coordinator:
    return Coordinator(
        trees={
            RoleType.ATTACKER: AttackerTree(),
            RoleType.DEFENDER: DefenderTree(),
            RoleType.SUPPORTER: SupporterTree(),
            RoleType.GOALIE: GoalieTree(),
        }
    )


# ---------------------------------------------------------------------------
# Basic dispatch
# ---------------------------------------------------------------------------

class TestCoordinatorDispatch:
    """Coordinator produces exactly one Intent per robot in the snapshot."""

    def setup_method(self) -> None:
        self.coord = _make_coordinator()
        self.snapshot = _make_full_snapshot()

    def test_returns_intent_for_every_robot(self) -> None:
        intents = self.coord.tick(self.snapshot, _ALL_ROBOT_IDS)
        assert len(intents) == len(_ALL_ROBOT_IDS)

    def test_all_intents_are_intent_instances(self) -> None:
        intents = self.coord.tick(self.snapshot, _ALL_ROBOT_IDS)
        for intent in intents:
            assert isinstance(intent, (IntentMove, IntentKick, IntentPass,
                                       IntentDribble, IntentReceive, IntentOrient))

    def test_no_robot_command_fields_in_any_intent(self) -> None:
        intents = self.coord.tick(self.snapshot, _ALL_ROBOT_IDS)
        for intent in intents:
            for field in ("vx", "vy", "vtheta", "kick", "dribbler"):
                assert not hasattr(intent, field), (
                    f"RobotCommand field '{field}' found in {intent!r}"
                )

    def test_tick_is_idempotent_same_snapshot(self) -> None:
        """Two ticks with the same snapshot produce the same intent types."""
        intents_a = self.coord.tick(self.snapshot, _ALL_ROBOT_IDS)
        intents_b = self.coord.tick(self.snapshot, _ALL_ROBOT_IDS)
        assert [type(i) for i in intents_a] == [type(i) for i in intents_b]


# ---------------------------------------------------------------------------
# Per-role intent type checks
# ---------------------------------------------------------------------------

class TestPerRoleIntents:
    """Each role produces the expected intent type for a canonical scenario."""

    def setup_method(self) -> None:
        # Ball far from attacker → attacker moves; ball near defenders → challenge.
        # Defenders are at (-2.5, ±1.0), ball at (2.0, 0.0) → far from defenders.
        self.coord = _make_coordinator()
        self.snapshot = _make_full_snapshot(ball_pos=(2.0, 0.0))

    def _intents_by_role(self) -> dict[int, Intent]:
        intents = self.coord.tick(self.snapshot, _ALL_ROBOT_IDS)
        # Map robot_id → intent using blackboard tracking.
        # Since Coordinator returns intents in robot_ids order (filtered by
        # snapshot presence), and all robots are present, indices match.
        return dict(zip(_ALL_ROBOT_IDS, intents))

    def test_goalie_produces_orient_then_move(self) -> None:
        """Goalie tree: LookAtBall first (writes IntentOrient), then GoToTarget
        overwrites with IntentMove to NEUTRAL_GOAL_POSITION. Final: IntentMove."""
        intent_map = self._intents_by_role()
        goalie_intent = intent_map[_GOALIE_ID]
        assert isinstance(goalie_intent, IntentMove), (
            f"Expected IntentMove from Goalie, got {type(goalie_intent).__name__}"
        )

    def test_supporter_produces_intent_move(self) -> None:
        """Supporter always produces IntentMove(MoveToSpace) in v1 because
        IsBallComing is stubbed to FAILURE."""
        intent_map = self._intents_by_role()
        for sid in _SUPPORTER_IDS:
            supporter_intent = intent_map[sid]
            assert isinstance(supporter_intent, IntentMove), (
                f"Supporter {sid}: expected IntentMove, got {type(supporter_intent).__name__}"
            )

    def test_attacker_produces_intent_move_when_ball_is_far(self) -> None:
        """Ball at (2.0, 0.0), attacker at (0.0, 0.0) → dist 2.0 > threshold → IntentMove."""
        intent_map = self._intents_by_role()
        attacker_intent = intent_map[_ATTACKER_ID]
        assert isinstance(attacker_intent, IntentMove), (
            f"Expected IntentMove from Attacker (ball far), got {type(attacker_intent).__name__}"
        )

    def test_attacker_intent_move_targets_ball(self) -> None:
        intent_map = self._intents_by_role()
        attacker_intent = intent_map[_ATTACKER_ID]
        assert isinstance(attacker_intent, IntentMove)
        assert attacker_intent.target_pos == (2.0, 0.0)

    def test_defender_produces_intent(self) -> None:
        """Each defender produces some intent — type depends on zone/ball proximity."""
        intent_map = self._intents_by_role()
        for did in _DEFENDER_IDS:
            assert intent_map[did] is not None, f"Defender {did} produced no intent"


# ---------------------------------------------------------------------------
# Attacker ball-in-range with supporters → IntentPass
# ---------------------------------------------------------------------------

class TestAttackerWithSupporterInRange:
    """When ball is close to attacker and supporters exist, coordinator returns IntentPass."""

    def setup_method(self) -> None:
        self.coord = _make_coordinator()
        # Ball at (0.3, 0) — within BALL_IN_RANGE_THRESHOLD (0.8) of attacker at (0, 0).
        self.snapshot = _make_full_snapshot(ball_pos=(0.3, 0.0))

    def test_attacker_produces_intent_pass(self) -> None:
        intents = self.coord.tick(self.snapshot, _ALL_ROBOT_IDS)
        intent_map = dict(zip(_ALL_ROBOT_IDS, intents))
        attacker_intent = intent_map[_ATTACKER_ID]
        assert isinstance(attacker_intent, IntentPass), (
            f"Expected IntentPass (ball near + supporters), got {type(attacker_intent).__name__}"
        )

    def test_attacker_passes_to_supporter(self) -> None:
        intents = self.coord.tick(self.snapshot, _ALL_ROBOT_IDS)
        intent_map = dict(zip(_ALL_ROBOT_IDS, intents))
        attacker_intent = intent_map[_ATTACKER_ID]
        assert isinstance(attacker_intent, IntentPass)
        assert attacker_intent.target_robot_id in _SUPPORTER_IDS


# ---------------------------------------------------------------------------
# Missing robots gracefully skipped
# ---------------------------------------------------------------------------

class TestMissingRobotsSkipped:
    """Robots absent from the snapshot are silently skipped."""

    def setup_method(self) -> None:
        self.coord = _make_coordinator()

    def test_missing_robot_not_in_output(self) -> None:
        # Only include the attacker in the snapshot.
        snapshot = Snapshot(
            ball_position=(2.0, 0.0),
            ball_velocity=(0.0, 0.0),
            own_robots=[
                RobotState(robot_id=_ATTACKER_ID, position=(0.0, 0.0), orientation=0.0)
            ],
            opponent_robots=[],
            referee_state=RefereeState(game_phase=GamePhase.RUNNING, score=(0, 0)),
        )
        intents = self.coord.tick(snapshot, _ALL_ROBOT_IDS)
        # Only one robot in snapshot → only one intent.
        assert len(intents) == 1

    def test_empty_robot_ids_produces_no_intents(self) -> None:
        snapshot = _make_full_snapshot()
        intents = self.coord.tick(snapshot, [])
        assert intents == []

    def test_robot_id_in_list_but_not_snapshot(self) -> None:
        snapshot = Snapshot(
            ball_position=(0.0, 0.0),
            ball_velocity=(0.0, 0.0),
            own_robots=[],
            opponent_robots=[],
            referee_state=RefereeState(game_phase=GamePhase.RUNNING, score=(0, 0)),
        )
        # robot_id=5 requested but not in snapshot → no output.
        intents = self.coord.tick(snapshot, [_ATTACKER_ID])
        assert intents == []


# ---------------------------------------------------------------------------
# Blackboard state after ticking
# ---------------------------------------------------------------------------

class TestBlackboardUpdatedAfterTick:
    """Coordinator updates per-robot blackboards on each tick."""

    def test_blackboard_created_and_populated(self) -> None:
        coord = _make_coordinator()
        snapshot = _make_full_snapshot()
        coord.tick(snapshot, [_ATTACKER_ID])
        bb = coord.blackboards[_ATTACKER_ID]
        assert bb is not None
        assert bb.robot_id == _ATTACKER_ID
        assert bb.current_role == RoleType.ATTACKER
        assert bb.current_intent is not None

    def test_last_intent_shifted_on_second_tick(self) -> None:
        coord = _make_coordinator()
        snapshot = _make_full_snapshot()

        coord.tick(snapshot, [_ATTACKER_ID])
        first_intent = coord.blackboards[_ATTACKER_ID].current_intent

        coord.tick(snapshot, [_ATTACKER_ID])
        bb = coord.blackboards[_ATTACKER_ID]
        assert bb.last_intent == first_intent

    def test_separate_blackboard_per_robot(self) -> None:
        coord = _make_coordinator()
        snapshot = _make_full_snapshot()
        coord.tick(snapshot, [_ATTACKER_ID, _GOALIE_ID])
        assert coord.blackboards[_ATTACKER_ID] is not coord.blackboards[_GOALIE_ID]


# ---------------------------------------------------------------------------
# No RobotCommand leakage — source-level checks
# ---------------------------------------------------------------------------

class TestNoRobotCommandLeakage:
    """No tree module or coordinator introduces raw motor command fields."""

    def _check_source(self, mod) -> None:
        import inspect
        src = inspect.getsource(mod)
        assert "RobotCommand" not in src, f"'RobotCommand' found in {mod.__name__}"

    def test_coordinator_source_clean(self) -> None:
        import TeamControl.bt.coordinator as mod
        self._check_source(mod)

    def test_attacker_source_clean(self) -> None:
        import TeamControl.bt.trees.attacker as mod
        self._check_source(mod)

    def test_defender_source_clean(self) -> None:
        import TeamControl.bt.trees.defender as mod
        self._check_source(mod)

    def test_supporter_source_clean(self) -> None:
        import TeamControl.bt.trees.supporter as mod
        self._check_source(mod)

    def test_goalie_source_clean(self) -> None:
        import TeamControl.bt.trees.goalie as mod
        self._check_source(mod)
