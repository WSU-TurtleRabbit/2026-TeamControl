# Codex Context - 2026 TeamControl

Working notes for Codex on this repository. This is a cleaned-up copy of the repo onboarding context with the details that are most useful during implementation.

## What this repo is
- RoboCup SSL team-control software for WSU TurtleRabbits, 2026 season.
- Python codebase for SSL-Vision and Game Controller ingestion, world-model construction, and robot/grSim command output.
- Includes a PySide6 operator UI plus headless launch paths for running behavior modes.

## Entry points
- `main.py` - headless launcher for the multiprocessing pipeline and selected robot mode.
- `main_bt_6v6.py` - v2 behavior-tree vs behavior-tree 6v6 simulation launcher.
- `ui_main.py` - starts the PySide6 operator UI via `TeamControl.ui.main_window.MainWindow`.
- `run.bat` - Windows convenience launcher.

## Package layout
Source lives under `src/TeamControl/`.

- `SSL/` - SSL-Vision, Game Controller, and grSim protocol/helpers.
- `network/` - UDP sockets, senders/receivers, command packet assembly, generated protobufs in `proto2/`.
- `cache/` - in-memory caches for ball, robot, team, game state, ticks, and onboard-ball observations.
- `robot/` - motion primitives and behavior entry points such as goalie, striker, navigator, coop, team, and ball navigation.
- `Formation/` - formations and strategic positioning helpers.
- `dispatcher/` - converts high-level robot commands into network output.
- `process_workers/` - multiprocessing workers for vision, GC FSM, world model, and robot receive loops.
- `onboard_vision/` - support for per-robot onboard camera observations.
- `world/` - world-model storage, frame history, and geometry/game-state helpers.
- `ui/` - PySide6 dashboard, calibration, network, robot, and test panels.
- `bt/` - behavior tree contracts, skills, trees, and coordinator logic.
- `harness/` - runner and smoke-test scaffolding.
- `voronoi_planner/` - path-planning helpers.

## Runtime shape
- The main process builds queues, a shared `WorldModelManager`, and a set of worker processes.
- `WorldModelManager` is a `multiprocessing.managers.BaseManager` wrapper around `WorldModel`, so state is shared through the manager rather than copied.
- The world model keeps a frame history and a version counter; consumers should use its accessors rather than reaching into internals.
- `BaseWorker` in `src/TeamControl/process_workers/worker.py` provides the common worker loop, error handling, and shutdown pattern.
- The dispatcher sends commands to both real robots and grSim depending on configuration, and rate-limits per robot.

## Configuration files
- `calibration.json` at the repo root contains drive calibration.
- `tuning.json` at the repo root contains runtime tuning values.
- `src/TeamControl/utils/ipconfig.yaml` contains robot IPs, grSim endpoints, and SSL network settings.
- `Config` in `src/TeamControl/utils/yaml_config.py` loads `ipconfig.yaml` relative to the module path, not the current working directory.
- `Sim6v6Config` in `src/TeamControl/utils/sim_config.py` loads `src/TeamControl/utils/sim_6v6.yaml` and keeps 6v6 scenario knobs separate from network config.

## Important operational notes
- Multiprocess workers do not share normal Python state. If a process needs state from another process, use queues, shared manager objects, or an explicit IPC path.
- After editing drive calibration programmatically, call `ball_nav._reload_calibration()` so live motion code picks up the new values.
- `src/TeamControl/network/proto2/` is generated protobuf code. Regenerate from the `.proto` sources; do not hand-edit the generated files.
- The UI and some runtime components depend on PySide6, multiprocessing freeze support, and Windows-friendly startup behavior.
- `main.py` is interactive and waits for `exit` on stdin to stop the process tree.

## Network defaults and expectations
- Default game-controller multicast is `224.5.23.1:10003`.
- Default SSL-Vision multicast is `224.5.23.2:10006`.
- Default grSim vision is also commonly pointed at `224.5.23.2:10020` in the docs, but the actual runtime config comes from `ipconfig.yaml`.
- Default grSim command port in the checked-in `ipconfig.yaml` is `20011`.
- `ipconfig.yaml` also controls team color (`us_yellow`) and field side (`us_positive`).

## v2 BT 6v6 simulation
- `main_bt_6v6.py` runs two independent v2 BT processes, one for yellow and one for blue, against the same shared `WorldModel` and `Dispatcher`.
- `run_bt_v2_process(...)` takes `is_yellow`, `robot_ids`, `role_assignment`, and `tick_period`; the 6v6 launcher passes explicit team perspective instead of relying on the world model default.
- `build_snapshot_from_world_model(...)` also accepts explicit `is_yellow`, which matters when both teams share one world model.
- `src/TeamControl/utils/sim_6v6.yaml` holds scenario-specific robot IDs, role assignment, and tick period only. Network settings still come from `ipconfig.yaml`.
- The documented behavior-tree flow does not wire in `GCfsm`, so the 6v6 BT sim currently behaves as if the game is permanently `RUNNING`.
- BT snapshots use raw SSL-Vision units. The doc calls out a unit mismatch risk: BT tunables were authored in metres, while grSim/vision snapshot positions are effectively in millimetres, so in-range logic can be much looser than intended.

## Behavior-tree work
- The `bt/` package is test-driven and has explicit contracts for intents, snapshots, blackboards, and role assignment.
- Tests expect coordinator logic to operate on intent objects, not `RobotCommand`.
- If you change coordinator or tree dispatch, review the tests under `tests/bt/` first.
- If you change the 6v6 BT path, review `main_bt_6v6.py`, `src/TeamControl/utils/sim_config.py`, `src/TeamControl/bt/run_bt_v2_process.py`, and `src/TeamControl/bt/adapter.py` together because the responsibility split is deliberate.

## v2 BT integration notes
- The `TeamControl.bt` package was imported from a standalone TurtleRabbitBT repo and rewritten from `src.bt.*` imports to `TeamControl.bt.*`.
- The imported core files are `bt/contracts/*`, `bt/skills/*`, `bt/trees/*`, and `bt/coordinator.py`; the repo-specific glue is `bt/adapter.py` and `bt/run_bt_v2_process.py`.
- The adapter path is `WorldModel -> Snapshot -> Coordinator.tick() -> per-robot blackboards -> RobotCommand -> dispatcher_q`.
- The role trees are intent producers, not command emitters. Actual command translation happens later in `bt/adapter.py`.
- Snapshot ball velocity is still stubbed as `(0, 0)` and referee score is still hard-coded `(0, 0)`; those are explicit follow-up gaps in the integration doc.
- `IntentMove`, `IntentKick`, `IntentPass`, `IntentDribble`, `IntentOrient`, and `IntentReceive` are the currently mapped intent types in the adapter.
- `IntentDribble` is currently a placeholder that reuses `move_to`, and `IntentReceive` currently holds station.
- The angular controller in the adapter is a proportional wrap-around-pi controller that writes directly to `RobotCommand.w`.
- The legacy `behaviour_tree/` package is intentionally left untouched so older harnesses still work alongside the v2 path.
- The current grSim sandbox integration point is still the legacy `run_bt_process` import; the doc says swapping to `run_bt_v2_process` is the way to exercise the v2 tree there.
- `run_bt_v2_process` defaults to robots `0..5`, 100 Hz tick rate, and the coordinator's role assignment unless overridden.
- Tests for the moved BT code are expected to run with `pytest tests/bt -v` and rely only on `py_trees` plus the copied contracts/skills/trees.

## Units caveat for v2 BT
- The BT contracts and skill docs describe positions and thresholds in metres.
- The BT adapter now normalizes raw SSL/grSim mm values into metres before constructing `Snapshot`.
- Confirmed runtime contract: grSim / SSL-Vision values in this repo are treated as raw world coordinates in millimetres. Older utilities such as `src/TeamControl/world/Trajectory.py` and `src/TeamControl/robot/constants.py` also use mm-sized field constants.
- The v2 role trees use hard-coded targets like `(-4.0, 0.0)`, `(1.0, 2.0)`, `(4.5, 0.0)`, and thresholds like `0.8` that only make sense if the snapshot is in metres.
- Before changing tree logic, confirm the coordinate unit contract for `WorldModel`, `Snapshot`, and `MotionTarget`, then normalize in one place.

## Practical edit rules
- Prefer `rg` / `rg --files` for search and file discovery.
- Avoid editing generated protobufs directly.
- Be careful with shared config and calibration files because they are read at runtime and can affect both UI and headless runs.
- If a change touches multiprocessing, verify the worker start/shutdown path as well as the code you are directly editing.
- If a change touches networking, confirm the port and address assumptions in `ipconfig.yaml` and `docs/SSL-NetworkPorts.md`.
- If a behavior-tree issue looks like all robots converging to the same place, check units before assuming the role tree is wrong.

## Docs worth checking
- `CONTRIBUTING.md`
- `docs/getting-started.md`
- `docs/HowToWriteCode.md`
- `docs/Multiprocessing.md`
- `docs/bt_v2_integration.md`
- `docs/bt_v2_6v6_sim.md`
- `docs/SSL-NetworkPorts.md`
- `tests/README.md`
- `src/TeamControl/network/proto2/README.md`

## Notes on repo docs
- Some docs contain typos or stale wording; prefer the actual source tree and runtime config over README spelling or path errors.
- The repository currently has a few duplicated or loosely maintained onboarding notes, so this file is the preferred working summary for Codex.
