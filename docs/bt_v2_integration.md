# TurtleRabbitBT (v2 BT) — Integration into 2026 TeamControl

This document records the merge of the standalone **TurtleRabbitBT** repo
(`vscode-workspace/TurtleRabbitBT`) into the 2026 TeamControl repo so the
new behaviour tree can be exercised against grSim using the existing
network / world-model stack.

It is intentionally lightweight: it documents what moved, where, and how
to run a tick loop. Deeper design notes for the BT itself live in
[bt_contracts.md](bt_contracts.md) (copied verbatim from the source repo).

---

## What was moved

| Source (TurtleRabbitBT)                | Destination (this repo)                          |
|----------------------------------------|--------------------------------------------------|
| `src/bt/contracts/*`                   | `src/TeamControl/bt/contracts/*`                 |
| `src/bt/skills/*`                      | `src/TeamControl/bt/skills/*`                    |
| `src/bt/trees/*`                       | `src/TeamControl/bt/trees/*`                     |
| `src/bt/coordinator.py`                | `src/TeamControl/bt/coordinator.py`              |
| `tests/test_*.py`                      | `tests/bt/test_*.py`                             |
| `docs/contracts.md`                    | `docs/bt_contracts.md`                           |

All `src.bt.*` imports in the moved files were rewritten to
`TeamControl.bt.*`. Nothing else in the source files was edited; the BT
itself stays as it was authored.

## New files added in this repo

Two integration-only modules, plus this doc:

| File                                           | Purpose                                                                 |
|-----------------------------------------------|-------------------------------------------------------------------------|
| `src/TeamControl/bt/adapter.py`               | Translates `WorldModel ⇄ Snapshot` and `Intent → RobotCommand`.         |
| `src/TeamControl/bt/run_bt_v2_process.py`     | Multiprocess entry point — mirrors `behaviour_tree/run_bt_process.py`.  |

The legacy `behaviour_tree/` package (`MainTree`, `cmd_mgr`, etc.) is left
untouched so existing harnesses still work while the v2 BT is exercised
side-by-side.

---

## Pipeline

```
SSL-Vision  →  WorldModel  ─┐
                            ├─ build_snapshot_from_world_model(wm)  →  Snapshot
                            │
                            ▼
                      Coordinator.tick(snapshot, robot_ids)
                            │
                            ▼
                  per-robot blackboards (each carries one Intent)
                            │
                            ▼
                dispatch_coordinator_output(...)
                            │  intent_to_robot_command per robot
                            ▼
                       dispatcher_q  →  dispatcher  →  grSim
```

### Field mapping (`adapter.py`)

| `Snapshot` field        | Source in `WorldModel`                                         |
|-------------------------|----------------------------------------------------------------|
| `ball_position`         | `frame.ball.x`, `frame.ball.y`                                 |
| `ball_velocity`         | `(0, 0)` — **TODO** wire `velocity_est`                        |
| `own_robots`            | `frame.robots_yellow` or `_blue` depending on `wm.us_yellow()` |
| `opponent_robots`       | the other team                                                 |
| `referee_state.phase`   | `wm.get_game_state()` mapped via `_PHASE_MAP`                  |
| `referee_state.score`   | `(0, 0)` — **TODO** read from `wm.ref_data` once exposed       |

### Intent → MotionTarget → RobotCommand

| Intent          | Skill function used     | `kick` | `dribble` |
|-----------------|-------------------------|--------|-----------|
| `IntentMove`    | `move_to`               | 0      | 0         |
| `IntentKick`    | `kick_at`               | 1      | 0         |
| `IntentPass`    | `kick_at` (target_pos)  | 1      | 0         |
| `IntentDribble` | `move_to` (placeholder) | 0      | 1         |
| `IntentOrient`  | zero-velocity, set `w`  | 0      | 0         |
| `IntentReceive` | zero-velocity           | 0      | 0         |

Angular velocity is a proportional wrap-around-pi controller in
`_angular_velocity_to_target`; the dispatcher / motion layer is expected
to consume `RobotCommand.w` directly.

---

## How to run against grSim

The current `src/TeamControl/SSL/grSim/sandbox.py` still wires the legacy
`run_bt_process` from `behaviour_tree/`. To exercise the v2 BT, edit
`sandbox.py` to swap that line for the new runner:

```python
# replace:
#   from behaviour_tree.run_bt_process import run_bt_process
#   bt = Process(target=run_bt_process, args=(wm, dispatcher_q,))
# with:
from TeamControl.bt.run_bt_v2_process import run_bt_v2_process
bt = Process(target=run_bt_v2_process, args=(is_running, wm, dispatcher_q,))
```

Then run as usual:

```powershell
.\run.bat              # or:  python -m TeamControl.SSL.grSim.sandbox
```

The runner builds one tree per role (Goalie / Defender / Supporter /
Attacker) and ticks robots `0..5`. Role assignment is fixed in
`Coordinator.ROLE_ASSIGNMENT` (`0=GOALIE, 1-2=DEFENDER, 3-4=SUPPORTER,
5=ATTACKER`).

The loop sleeps `TICK_PERIOD = 0.01 s` (100 Hz target). If `WorldModel`
has not received its first vision frame yet, the tick is skipped.

---

## Running the moved tests

```powershell
pytest tests/bt -v
```

The original TurtleRabbitBT test suite is preserved as-is; only the
imports were rewritten. They do not require the rest of the TeamControl
stack to run — `py_trees` is the only non-stdlib dep.

---

## Known gaps / follow-ups

1. **Ball velocity** is hard-coded to `(0, 0)`. Wire `velocity_est` or
   compute from frame history so `IsBallComing` and pass logic can use
   real motion data.
2. **Referee score** is hard-coded `(0, 0)`; expose from `wm.ref_data`.
3. **Dribble** is mapped to `move_to`; build a real dribble skill that
   keeps the ball glued to the kicker.
4. **`IntentReceive`** holds station — should reposition to the predicted
   reception point once the pass play is wired.
5. **Angular controller** in the adapter is a simple P-controller. If the
   motion layer expects a target heading rather than `w`, push the
   conversion downstream and stop setting `w` here.
6. **Snapshot per-tick allocation**: each tick currently builds a fresh
   `Snapshot` (frozen dataclass + tuple coercions). If this shows up in
   profiling, batch the conversion or expose `WorldModel` accessors that
   yield the underlying objects in-place.

---

## File locations cheat sheet

```
src/TeamControl/bt/
├── __init__.py            # intentionally light — adapter NOT auto-imported
├── adapter.py             # WorldModel↔Snapshot, Intent→RobotCommand
├── coordinator.py         # role assignment + per-tick dispatch
├── run_bt_v2_process.py   # multiprocess entry point
├── contracts/             # Snapshot, Intent, Blackboard, MotionTarget
├── skills/                # move_to, kick_at, receive_ball
└── trees/                 # attacker, defender, supporter, goalie

tests/bt/                  # original test suite (imports rewritten)
docs/bt_contracts.md       # full contracts walkthrough (from source repo)
docs/bt_v2_integration.md  # this file
```
