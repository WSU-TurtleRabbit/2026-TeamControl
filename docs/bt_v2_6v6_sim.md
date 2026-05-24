# v2 BT 6v6 Simulation

Launches **two** v2 (TurtleRabbitBT) behaviour-tree processes against a
single grSim instance — one driving the yellow team, one driving the blue
team — so we can watch the BT play itself.

For background on the v2 BT itself (contracts, adapter, single-team
launcher), see [bt_v2_integration.md](bt_v2_integration.md).

---

## What's involved

| Path                                                | Purpose                                                                 |
|-----------------------------------------------------|-------------------------------------------------------------------------|
| `main_bt_6v6.py`                                    | Repo-root launcher. Spawns vision + WMWorker + Dispatcher + two BTs.    |
| `src/TeamControl/utils/sim_6v6.yaml`                | 6v6 scenario config (per-team robot IDs, role assignment, tick period). |
| `src/TeamControl/utils/sim_config.py`               | `Sim6v6Config` loader for the yaml above.                               |
| `src/TeamControl/bt/run_bt_v2_process.py`           | Now takes `is_yellow`, `robot_ids`, `role_assignment`, `tick_period`.   |
| `src/TeamControl/bt/adapter.py`                     | `build_snapshot_from_world_model` takes explicit `is_yellow`.           |
| `src/TeamControl/bt/coordinator.py`                 | `Coordinator(...)` accepts optional `role_assignment` dict.             |

Network settings (grSim address, vision multicast/port, `send_to_grSim`)
still come from the existing `src/TeamControl/utils/ipconfig.yaml`. The
6v6 yaml deliberately only holds scenario-specific knobs so the two
concerns don't get mixed up.

## How to run

In an activated venv:

```powershell
python main_bt_6v6.py
```

That starts (in one process tree):

```
Vision  → vision_q → WMWorker → wm (shared, via WorldModelManager)
                              │
              ┌───────────────┴───────────────┐
              │                               │
 run_bt_v2_process(is_yellow=True)   run_bt_v2_process(is_yellow=False)
              │                               │
              └────── dispatch_q ─────────────┘
                              │
                          Dispatcher  →  grSim (UDP)
```

Type `exit` (or Ctrl+C) to stop. The Dispatcher emits a stop-command to
every controlled robot on shutdown.

## Customising the scenario

Edit `src/TeamControl/utils/sim_6v6.yaml`:

```yaml
yellow:
  robot_ids: [0, 1, 2, 3, 4, 5]   # remove some to thin the team
blue:
  robot_ids: [0, 1, 2, 3, 4, 5]

roles:
  0: goalie
  1: defender
  2: defender
  3: supporter
  4: supporter
  5: attacker

tick_period: 0.01   # 100 Hz
```

Valid roles are the names of `RoleType` from `bt/contracts/blackboard.py`
(case-insensitive): `goalie`, `defender`, `supporter`, `attacker`. Robot
IDs not listed under `roles` fall back to `SUPPORTER` in the Coordinator.

The same `roles` map is applied to both teams. If the two sides ever need
different role layouts, split `roles` into `yellow_roles` / `blue_roles`
in the yaml and add the second field to `Sim6v6Config`.

## Notes for follow-up work

- **No shared blackboard between teams.** Each BT process owns its own
  `Coordinator` and per-robot blackboards. They share only the read-only
  `WorldModel` (via `WorldModelManager`) and the write-only
  `dispatch_q` — exactly what you want for adversarial play.
- **Both teams emit `RobotCommand.isYellow` correctly** because
  `intent_to_robot_command` takes the `is_yellow` flag from the
  per-process arg, not from the world model.
- **Game-controller events are not wired in** (no `GCfsm` process here).
  Both BTs see `GamePhase.RUNNING` permanently. Once we want kickoffs /
  stop / halt to affect 6v6 play, add `GCfsm.run_worker` to `main_bt_6v6.py`
  and let `WMWorker` populate `wm._state`.
- **Unit caveat carries over.** Snapshots use the raw SSL-Vision units
  (mm in grSim). The BT tunables (`BALL_IN_RANGE_THRESHOLD = 0.8`, etc.)
  were authored in metres. Velocities get clamped to `MAX_SPEED`, so
  robots still move sensibly, but in-range checks behave as if "in range
  ≈ touching." Rescale either the constants or the snapshot when
  tightening behaviour.
