"""Tests for BT adapter unit normalization."""
from __future__ import annotations

from TeamControl.SSL.game_controller.common import GameState
from TeamControl.bt.adapter import build_snapshot_from_world_model


class _Ball:
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y


class _Robot:
    def __init__(self, robot_id: int, x: float, y: float, o: float) -> None:
        self.id = robot_id
        self.x = x
        self.y = y
        self.o = o


class _Frame:
    def __init__(self) -> None:
        self.ball = _Ball(1500.0, -2500.0)
        self.robots_yellow = [_Robot(0, 1000.0, 2000.0, 1.25)]
        self.robots_blue = [_Robot(1, -3000.0, 500.0, -0.5)]


class _WorldModel:
    def get_latest_frame(self):
        return _Frame()

    def us_yellow(self):
        return True

    def get_game_state(self):
        return GameState.RUNNING


def test_build_snapshot_from_world_model_converts_mm_to_m():
    snapshot = build_snapshot_from_world_model(_WorldModel())

    assert snapshot is not None
    assert snapshot.ball_position == (1.5, -2.5)
    assert snapshot.own_robots[0].position == (1.0, 2.0)
    assert snapshot.opponent_robots[0].position == (-3.0, 0.5)
    assert snapshot.own_robots[0].orientation == 1.25
    assert snapshot.referee_state.game_phase == GameState.RUNNING
