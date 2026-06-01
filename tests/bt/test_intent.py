"""Tests for Intent dataclasses and IntentType enum (T002, R002).

Written TDD-first: these tests define the contract before the implementation exists.
"""
from __future__ import annotations

import dataclasses
import pytest

from TeamControl.bt.contracts.intent import (
    Intent,
    IntentDribble,
    IntentKick,
    IntentMove,
    IntentOrient,
    IntentPass,
    IntentReceive,
    IntentType,
)


# ---------------------------------------------------------------------------
# IntentType enum
# ---------------------------------------------------------------------------

class TestIntentTypeEnum:
    def test_enum_members_exist(self) -> None:
        assert IntentType.MOVE is not None
        assert IntentType.KICK is not None
        assert IntentType.RECEIVE is not None
        assert IntentType.PASS is not None
        assert IntentType.DRIBBLE is not None
        assert IntentType.ORIENT is not None

    def test_enum_values(self) -> None:
        assert IntentType.MOVE.value == 1
        assert IntentType.KICK.value == 2
        assert IntentType.RECEIVE.value == 3
        assert IntentType.PASS.value == 4
        assert IntentType.DRIBBLE.value == 5
        assert IntentType.ORIENT.value == 6

    def test_exactly_six_members(self) -> None:
        assert len(IntentType) == 6


# ---------------------------------------------------------------------------
# IntentMove
# ---------------------------------------------------------------------------

class TestIntentMove:
    def test_construction_with_orientation(self) -> None:
        intent = IntentMove(target_pos=(1.0, 2.0), target_orientation=0.5)
        assert intent.target_pos == (1.0, 2.0)
        assert intent.target_orientation == 0.5

    def test_construction_without_orientation(self) -> None:
        intent = IntentMove(target_pos=(3.0, 4.0), target_orientation=None)
        assert intent.target_pos == (3.0, 4.0)
        assert intent.target_orientation is None

    def test_frozen(self) -> None:
        intent = IntentMove(target_pos=(1.0, 2.0), target_orientation=None)
        with pytest.raises((dataclasses.FrozenInstanceError, AttributeError)):
            intent.target_pos = (0.0, 0.0)  # type: ignore[misc]

    def test_no_motor_command_fields(self) -> None:
        fields = {f.name for f in dataclasses.fields(IntentMove)}
        assert "vx" not in fields
        assert "vy" not in fields
        assert "vtheta" not in fields
        assert "kick" not in fields
        assert "dribbler" not in fields

    def test_is_dataclass(self) -> None:
        assert dataclasses.is_dataclass(IntentMove)


# ---------------------------------------------------------------------------
# IntentKick
# ---------------------------------------------------------------------------

class TestIntentKick:
    def test_construction(self) -> None:
        intent = IntentKick(target_pos=(5.0, 6.0))
        assert intent.target_pos == (5.0, 6.0)

    def test_frozen(self) -> None:
        intent = IntentKick(target_pos=(1.0, 2.0))
        with pytest.raises((dataclasses.FrozenInstanceError, AttributeError)):
            intent.target_pos = (0.0, 0.0)  # type: ignore[misc]

    def test_no_motor_command_fields(self) -> None:
        fields = {f.name for f in dataclasses.fields(IntentKick)}
        for forbidden in ("vx", "vy", "vtheta", "kick", "dribbler"):
            assert forbidden not in fields

    def test_is_dataclass(self) -> None:
        assert dataclasses.is_dataclass(IntentKick)


# ---------------------------------------------------------------------------
# IntentPass
# ---------------------------------------------------------------------------

class TestIntentPass:
    def test_construction(self) -> None:
        intent = IntentPass(target_robot_id=3, target_pos=(7.0, 8.0))
        assert intent.target_robot_id == 3
        assert intent.target_pos == (7.0, 8.0)

    def test_frozen(self) -> None:
        intent = IntentPass(target_robot_id=1, target_pos=(0.0, 0.0))
        with pytest.raises((dataclasses.FrozenInstanceError, AttributeError)):
            intent.target_robot_id = 99  # type: ignore[misc]

    def test_no_motor_command_fields(self) -> None:
        fields = {f.name for f in dataclasses.fields(IntentPass)}
        for forbidden in ("vx", "vy", "vtheta", "kick", "dribbler"):
            assert forbidden not in fields

    def test_is_dataclass(self) -> None:
        assert dataclasses.is_dataclass(IntentPass)


# ---------------------------------------------------------------------------
# IntentDribble
# ---------------------------------------------------------------------------

class TestIntentDribble:
    def test_construction(self) -> None:
        intent = IntentDribble(target_pos=(9.0, 10.0))
        assert intent.target_pos == (9.0, 10.0)

    def test_frozen(self) -> None:
        intent = IntentDribble(target_pos=(1.0, 2.0))
        with pytest.raises((dataclasses.FrozenInstanceError, AttributeError)):
            intent.target_pos = (0.0, 0.0)  # type: ignore[misc]

    def test_no_motor_command_fields(self) -> None:
        fields = {f.name for f in dataclasses.fields(IntentDribble)}
        for forbidden in ("vx", "vy", "vtheta", "kick", "dribbler"):
            assert forbidden not in fields

    def test_is_dataclass(self) -> None:
        assert dataclasses.is_dataclass(IntentDribble)


# ---------------------------------------------------------------------------
# IntentReceive
# ---------------------------------------------------------------------------

class TestIntentReceive:
    def test_construction_no_args(self) -> None:
        intent = IntentReceive()
        assert intent is not None

    def test_no_fields(self) -> None:
        assert dataclasses.fields(IntentReceive) == ()

    def test_frozen(self) -> None:
        intent = IntentReceive()
        # Frozen dataclass with no fields — verify it is indeed frozen
        assert dataclasses.is_dataclass(intent)
        # Attempting to set a non-existent attribute should raise
        with pytest.raises((dataclasses.FrozenInstanceError, AttributeError)):
            intent.some_field = 1  # type: ignore[attr-defined]

    def test_no_motor_command_fields(self) -> None:
        fields = {f.name for f in dataclasses.fields(IntentReceive)}
        for forbidden in ("vx", "vy", "vtheta", "kick", "dribbler"):
            assert forbidden not in fields

    def test_is_dataclass(self) -> None:
        assert dataclasses.is_dataclass(IntentReceive)


# ---------------------------------------------------------------------------
# IntentOrient
# ---------------------------------------------------------------------------

class TestIntentOrient:
    def test_construction(self) -> None:
        intent = IntentOrient(target_orientation=1.57)
        assert intent.target_orientation == pytest.approx(1.57)

    def test_frozen(self) -> None:
        intent = IntentOrient(target_orientation=0.0)
        with pytest.raises((dataclasses.FrozenInstanceError, AttributeError)):
            intent.target_orientation = 99.0  # type: ignore[misc]

    def test_no_motor_command_fields(self) -> None:
        fields = {f.name for f in dataclasses.fields(IntentOrient)}
        for forbidden in ("vx", "vy", "vtheta", "kick", "dribbler"):
            assert forbidden not in fields

    def test_is_dataclass(self) -> None:
        assert dataclasses.is_dataclass(IntentOrient)


# ---------------------------------------------------------------------------
# Intent union type
# ---------------------------------------------------------------------------

class TestIntentUnion:
    """The Intent type alias must cover all six variants for type-checking purposes."""

    def test_all_variants_are_valid_intent_instances(self) -> None:
        variants: list[Intent] = [
            IntentMove(target_pos=(0.0, 0.0), target_orientation=None),
            IntentKick(target_pos=(1.0, 1.0)),
            IntentPass(target_robot_id=0, target_pos=(2.0, 2.0)),
            IntentDribble(target_pos=(3.0, 3.0)),
            IntentReceive(),
            IntentOrient(target_orientation=0.0),
        ]
        assert len(variants) == 6

    def test_intent_union_is_type_alias(self) -> None:
        # Intent should be importable and not raise
        import TeamControl.bt.contracts.intent as mod
        assert hasattr(mod, "Intent")
