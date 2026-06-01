# Behaviour Tree Contracts — Developer Overview

This document covers the four contract modules in `src/bt/contracts/`. These are the shared data types that every other layer in the system builds against. If you are new to this codebase, read this first.

---

## Why contracts first?

The old architecture had trees, skill functions, and the dispatcher all reaching into each other. A tree node would call `cmd_mgr.pack_and_send()` directly. World state (ball position, robot positions) lived on the blackboard alongside decision state. There was no clear boundary between "what is happening in the world" and "what the robot has decided to do."

The refactor draws three hard boundaries:

```
Snapshot (world state, read-only)
    ↓
Behaviour Tree → Intent (what to do)
    ↓
Skill Functions → MotionTarget (how to do it)
    ↓
Motion Controller → RobotCommand (raw velocities)
    ↓
Dispatcher
```

Each layer communicates with the next through one of these contract types — never by direct coupling.

---

## Snapshot (`snapshot.py`)

**What it is:** A frozen (immutable) snapshot of the world at a single tick. This is the only world state that enters the decision pipeline.

**Fields:**
- `ball_position`, `ball_velocity` — ball coordinates and velocity in world frame
- `own_robots` — list of our robots with position and orientation
- `opponent_robots` — list of opponent robots
- `referee_state` — current game phase (`RUNNING` / `STOPPED` / `HALTED`) and score

**Key decisions:**
- **Frozen dataclass.** Mutation after construction raises `FrozenInstanceError`. Tree nodes cannot accidentally modify shared world state.
- **Lists coerced to tuples.** The `__init__` accepts any `Sequence` for `own_robots` / `opponent_robots` but stores them as `tuple`. This keeps the frozen guarantee even when the caller passes a list.
- **`GamePhase` is a `str` enum.** Matches the old `GameState` string values (`"RUNNING"` etc.), making migration from the old architecture easier.

**Rule:** No tree node or skill function is allowed to hold a reference to the previous tick's Snapshot. Each tick gets a fresh one.

---

## Intent (`intent.py`)

**What it is:** The output of a behaviour tree. Represents *what* a robot should do, not *how* to do it.

**Variants:**

| Class | Meaning | Key fields |
|-------|---------|------------|
| `IntentMove` | Move to a position | `target_pos`, `target_orientation` (optional) |
| `IntentKick` | Kick the ball toward a point | `target_pos` |
| `IntentPass` | Pass to a specific teammate | `target_robot_id`, `target_pos` |
| `IntentDribble` | Dribble ball toward a position | `target_pos` |
| `IntentReceive` | Signal readiness to receive | *(no fields)* |
| `IntentOrient` | Rotate in place | `target_orientation` |

The `IntentType` enum (`MOVE=1`, `KICK=2`, etc.) exists for logging and routing — you do not need to match on it in most code, just use `isinstance()` checks.

The `Intent` alias at the bottom of the file is the union type:
```python
Intent = IntentMove | IntentKick | IntentPass | IntentDribble | IntentReceive | IntentOrient
```
Use this as the type annotation wherever you accept or return any intent.

**Key decisions:**
- **No raw command fields.** There is no `vx`, `vy`, `kick`, or `dribbler` anywhere in this module. If you find yourself adding one, you are in the wrong layer — that belongs in `RobotCommand`, downstream of the skill functions.
- **All variants are frozen.** Intents are produced by the tree and consumed by the skill layer — they should never be mutated in transit.

---

## RobotBlackboard (`blackboard.py`)

**What it is:** Per-robot mutable state that persists across ticks. One instance per robot, held by the Coordinator.

**Fields:**
- `robot_id` — which robot this belongs to
- `current_role` — `RoleType` enum (`ATTACKER`, `DEFENDER`, `SUPPORTER`, `GOALIE`)
- `current_intent` — the `Intent` produced on the most recent tick (or `None`)
- `last_intent` — the `Intent` from the previous tick (shifted each tick)

**The tick update pattern:**
```python
blackboard.last_intent = blackboard.current_intent
# ... tree ticks and writes new intent ...
blackboard.current_intent = new_intent
```

**Key decisions:**
- **Mutable, not frozen.** Unlike Snapshot and Intent, the blackboard needs to be updated every tick. It is intentionally a plain (non-frozen) dataclass.
- **Decision state only.** The blackboard must never hold world state. If you need ball position inside a tree node, read it from the `Snapshot` that is passed in — do not cache it on the blackboard. This was the main violation in the old architecture and is the reason things were hard to debug.
- **`current_intent` / `last_intent` are `Intent | None`.** The `None` case represents "tree has not yet produced an intent" — valid on the first tick before the tree has run.

---

## MotionTarget (`motion_target.py`)

**What it is:** The output of a skill function. Represents *how* the robot should move — velocity, heading, and arrival style.

**Fields:**
- `target_velocity` — `(vx, vy)` in m/s, robot local frame
- `target_orientation` — desired heading in radians, world frame
- `arrival_mode` — one of `"precision"`, `"normal"`, `"fast"`

**Arrival modes:**
- `"precision"` — slows to an accurate stop at the target (use for positioning)
- `"normal"` — balanced speed and accuracy (general movement)
- `"fast"` — maximum speed, less accurate stop (use for chasing ball)

**Key decisions:**
- **Frozen.** Skill functions are pure — same inputs produce same outputs. The result should not be mutated after it is returned.
- **No robot ID.** `MotionTarget` describes motion for one robot. The caller (motion controller) already knows which robot it is driving — the ID does not need to travel with the target.
- **`arrival_mode` is a string for now.** This may become a proper enum in a future sprint if the motion controller needs stricter typing.

---

## What lives where — quick reference

| Question | Answer |
|----------|--------|
| Where is the ball? | `snapshot.ball_position` |
| What role is robot 3 playing? | `blackboard.current_role` (for robot 3's blackboard) |
| What did the tree decide? | `blackboard.current_intent` |
| How should the robot move? | `MotionTarget` returned by skill function |
| Where does a raw velocity command live? | `RobotCommand` — downstream of all of this |

---

## Tests

Each contract has a dedicated test file in `tests/`:

| Contract | Test file | Tests |
|----------|-----------|-------|
| Snapshot | `tests/test_snapshot.py` | 18 |
| Intent | `tests/test_intent.py` | 31 |
| Blackboard | `tests/test_blackboard.py` | 18 |
| MotionTarget | `tests/test_motion_target.py` | 16 |

Run all contract tests: `python -m pytest tests/ -v`
