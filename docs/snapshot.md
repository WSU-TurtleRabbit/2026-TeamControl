# `Snapshot` — read-only world state for the BT pipeline

A `Snapshot` is the **sole world-state input** to the behaviour tree on each
tick. Tree nodes and skill functions read from it; they never mutate it.
Decision state (the thing the tree itself is computing) lives on the
per-robot `RobotBlackboard`, not here.

Defined in [src/TeamControl/bt/contracts/snapshot.py](../src/TeamControl/bt/contracts/snapshot.py).

---

## Lifecycle (per tick)

```
WorldModel (network ingest)
        │
        ▼
build_snapshot_from_world_model(wm)   ← adapter.py — produces ONE Snapshot per tick
        │
        ▼
Coordinator.tick(snapshot, robot_ids)
        │
        ▼
tree.set_snapshot(snapshot); tree.tick(bb)
        │
        ▼
skills (move_to / kick_at / …) read snapshot.own_robots, ball_position, etc.
```

A new `Snapshot` is built each tick and discarded after dispatch. Frozen
(`dataclasses.dataclass(frozen=True)`) — no mutation possible after
construction.

---

## Fields

| Field | Type | Unit | Notes |
|---|---|---|---|
| `ball_position` | `tuple[float, float]` | metres, field frame | `(x, y)`. Origin at field centre. |
| `ball_velocity` | `tuple[float, float]` | m/s, field frame | Currently hard-coded `(0, 0)` in `adapter.py` — see TODO at the bottom of this file. |
| `own_robots` | `tuple[RobotState, ...]` | — | Our team's robots **that vision can currently see**. Robots off-field or undetected do not appear. |
| `opponent_robots` | `tuple[RobotState, ...]` | — | Same shape, the other team. |
| `referee_state` | `RefereeState` | — | Game phase, score, ball-placement target. See below. |

### `RobotState`

| Field | Type | Unit |
|---|---|---|
| `robot_id` | `int` | shellID (0–5 typically) |
| `position` | `tuple[float, float]` | metres |
| `orientation` | `float` | radians, CCW from +x, wrapped to `[-π, π]` |

### `RefereeState`

| Field | Type | Notes |
|---|---|---|
| `game_phase` | `GamePhase` | One of `HALTED`, `HALF_TIME`, `STOPPED`, `PREPARE_KICKOFF`, `KICKOFF`, `FREE_KICK`, `BALL_PLACEMENT`, `PENALTY_SHOOT`, `PENALTY_DEFEND`, `RUNNING`. The GC FSM has already resolved ours-vs-theirs before populating this — there's no separate "is it our kickoff?" flag. |
| `score` | `tuple[int, int]` | `(own, opponent)`. Currently hard-coded `(0, 0)` in `adapter.py`. |
| `ball_placement_pos` | `tuple[float, float] \| None` | Target during `BALL_PLACEMENT`; `None` otherwise. |

### `GamePhase`

`HALTED`/`HALF_TIME` are full stops (no movement). `STOPPED` requires 0.5 m
ball clearance. The set-piece states (`PREPARE_KICKOFF`, `KICKOFF`,
`FREE_KICK`, `BALL_PLACEMENT`, `PENALTY_SHOOT`, `PENALTY_DEFEND`) only fire
when the privilege is ours. `RUNNING` is normal play.

---

## How to consume a Snapshot

From inside a tree node:

```python
class MyNode(py_trees.behaviour.Behaviour):
    def update(self):
        snap = self._tree._snapshot
        bb = self._tree._blackboard_ref[0]
        if snap is None or bb is None:
            return py_trees.common.Status.FAILURE

        # Find your own robot
        me = next((r for r in snap.own_robots if r.robot_id == bb.robot_id), None)
        if me is None:
            return py_trees.common.Status.FAILURE   # I'm off the field this tick

        ball = snap.ball_position
        phase = snap.referee_state.game_phase
        # … decide, then write to bb.current_intent and bb.intent_source
```

Conventions worth knowing:

- A robot can be **absent from `own_robots`** if vision didn't see it this
  tick. Always handle the `None` case.
- Positions are world-frame metres. Velocities to the **skill layer** are
  also world-frame; the adapter rotates them into body-frame for grSim
  command serialisation.
- The Snapshot is **shared** across all robots being ticked. Treat it as
  read-only — `tuple` and `frozen=True` will catch most accidental mutation.

---

## How to produce a Snapshot (or alternate implementations)

The canonical producer is `build_snapshot_from_world_model(wm)` in
[src/TeamControl/bt/adapter.py](../src/TeamControl/bt/adapter.py). It reads
the latest vision frame and GC state from a shared `WorldModel` and emits a
single `Snapshot`. Returns `None` when no vision frame has been received yet
— callers should skip the tick.

Anything that conforms to this shape works:

```python
from TeamControl.bt.contracts.snapshot import (
    GamePhase, RefereeState, RobotState, Snapshot,
)

snap = Snapshot(
    ball_position=(0.5, -1.2),
    ball_velocity=(0.0, 0.0),
    own_robots=(
        RobotState(robot_id=0, position=(-4.0,  0.0), orientation=0.0),
        RobotState(robot_id=5, position=( 0.5, -1.1), orientation=1.57),
    ),
    opponent_robots=(),
    referee_state=RefereeState(
        game_phase=GamePhase.RUNNING,
        score=(0, 0),
        ball_placement_pos=None,
    ),
)
```

Common alternate producers:

- **Unit tests** — build small fixed Snapshots directly (see
  [tests/bt/test_attacker_tree.py](../tests/bt/test_attacker_tree.py) for
  examples).
- **Offline replay** — read positions from a CSV/JSONL log and yield one
  `Snapshot` per row to replay a recorded session through the BT.
- **Alternate vision sources** — write your own `build_snapshot_from_*`
  reading from whatever source you have; the rest of the pipeline doesn't
  care where the data came from as long as the Snapshot is well-formed.

If you add a new field, make sure to:
1. Add it to the dataclass with a sensible default.
2. Update `build_snapshot_from_world_model` to populate it.
3. Keep it frozen / immutable — `tuple` for sequences, never `list`.

---

## Known gaps (don't trust these fields yet)

- **`ball_velocity`** is hard-coded `(0, 0)` in `adapter.py`. Anything that
  reasons about ball motion (e.g. a real `IsBallComing` predicate) will see
  a stationary ball even when it's rolling.
- **`referee_state.score`** is hard-coded `(0, 0)`.
