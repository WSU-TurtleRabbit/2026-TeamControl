"""Tests for MotionTarget dataclass — R009 (partial).

TDD: these tests are written before the implementation.
All tests should fail until src/bt/contracts/motion_target.py is implemented.
"""
from __future__ import annotations

import dataclasses
import pytest

from TeamControl.bt.contracts.motion_target import MotionTarget


class TestMotionTargetConstruction:
    def test_construction_basic(self):
        mt = MotionTarget(
            target_velocity=(1.0, 0.5),
            target_orientation=0.0,
            arrival_mode="normal",
        )
        assert mt.target_velocity == (1.0, 0.5)
        assert mt.target_orientation == 0.0
        assert mt.arrival_mode == "normal"

    def test_construction_precision_mode(self):
        mt = MotionTarget(
            target_velocity=(0.0, 0.0),
            target_orientation=1.57,
            arrival_mode="precision",
        )
        assert mt.arrival_mode == "precision"

    def test_construction_fast_mode(self):
        mt = MotionTarget(
            target_velocity=(3.0, -1.0),
            target_orientation=-0.5,
            arrival_mode="fast",
        )
        assert mt.arrival_mode == "fast"

    def test_target_velocity_is_tuple(self):
        mt = MotionTarget(
            target_velocity=(2.0, -2.0),
            target_orientation=0.0,
            arrival_mode="normal",
        )
        assert isinstance(mt.target_velocity, tuple)
        assert len(mt.target_velocity) == 2

    def test_target_orientation_is_float(self):
        mt = MotionTarget(
            target_velocity=(0.0, 0.0),
            target_orientation=3.14,
            arrival_mode="normal",
        )
        assert isinstance(mt.target_orientation, float)

    def test_arrival_mode_is_str(self):
        mt = MotionTarget(
            target_velocity=(0.0, 0.0),
            target_orientation=0.0,
            arrival_mode="normal",
        )
        assert isinstance(mt.arrival_mode, str)


class TestMotionTargetImmutability:
    def test_frozen_target_velocity(self):
        mt = MotionTarget(
            target_velocity=(1.0, 0.0),
            target_orientation=0.0,
            arrival_mode="normal",
        )
        with pytest.raises((dataclasses.FrozenInstanceError, AttributeError)):
            mt.target_velocity = (9.0, 9.0)  # type: ignore[misc]

    def test_frozen_target_orientation(self):
        mt = MotionTarget(
            target_velocity=(0.0, 0.0),
            target_orientation=0.0,
            arrival_mode="normal",
        )
        with pytest.raises((dataclasses.FrozenInstanceError, AttributeError)):
            mt.target_orientation = 99.0  # type: ignore[misc]

    def test_frozen_arrival_mode(self):
        mt = MotionTarget(
            target_velocity=(0.0, 0.0),
            target_orientation=0.0,
            arrival_mode="normal",
        )
        with pytest.raises((dataclasses.FrozenInstanceError, AttributeError)):
            mt.arrival_mode = "fast"  # type: ignore[misc]


class TestMotionTargetDataclassProperties:
    def test_is_dataclass(self):
        assert dataclasses.is_dataclass(MotionTarget)

    def test_equality(self):
        a = MotionTarget(target_velocity=(1.0, 2.0), target_orientation=0.5, arrival_mode="normal")
        b = MotionTarget(target_velocity=(1.0, 2.0), target_orientation=0.5, arrival_mode="normal")
        assert a == b

    def test_inequality_velocity(self):
        a = MotionTarget(target_velocity=(1.0, 0.0), target_orientation=0.0, arrival_mode="normal")
        b = MotionTarget(target_velocity=(2.0, 0.0), target_orientation=0.0, arrival_mode="normal")
        assert a != b

    def test_inequality_orientation(self):
        a = MotionTarget(target_velocity=(0.0, 0.0), target_orientation=0.0, arrival_mode="normal")
        b = MotionTarget(target_velocity=(0.0, 0.0), target_orientation=1.0, arrival_mode="normal")
        assert a != b

    def test_inequality_arrival_mode(self):
        a = MotionTarget(target_velocity=(0.0, 0.0), target_orientation=0.0, arrival_mode="normal")
        b = MotionTarget(target_velocity=(0.0, 0.0), target_orientation=0.0, arrival_mode="fast")
        assert a != b

    def test_repr_contains_fields(self):
        mt = MotionTarget(target_velocity=(1.0, 2.0), target_orientation=0.5, arrival_mode="precision")
        r = repr(mt)
        assert "target_velocity" in r
        assert "target_orientation" in r
        assert "arrival_mode" in r

    def test_hashable(self):
        """Frozen dataclasses should be hashable by default."""
        mt = MotionTarget(target_velocity=(1.0, 0.0), target_orientation=0.0, arrival_mode="normal")
        assert hash(mt) is not None
        # Can be used in a set
        s = {mt}
        assert mt in s
