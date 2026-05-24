# Repo Context — 2026-TeamControl

Onboarding notes for a Claude agent picking up work on this repo.

## What this is
RoboCup SSL (Small Size League) team control software for **WSU TurtleRabbits**, 2026 season. Python codebase that drives a fleet of soccer robots via SSL-Vision input and grSim / real-robot output. Has a PyQt UI for operators (dashboard, calibration, tuning, testing).

Entry points:
- [main.py](main.py) — headless / harness entry.
- [ui_main.py](ui_main.py) — launches the PyQt operator UI ([src/TeamControl/ui/main_window.py](src/TeamControl/ui/main_window.py)).
- [run.bat](run.bat) — Windows convenience launcher.

## Layout (`src/TeamControl/`)
- `SSL/` — SSL-Vision parsing (`vision/`), Game Controller protocol (`game_controller/`), grSim sandbox helpers (`grSim/`).
- `network/` — UDP sockets, senders/receivers, grSim packet factory, and `proto2/` (generated protobufs for SSL-Vision, grSim, GC).
- `cache/` — shared in-memory state: `ball_cache`, `robot_cache`, `team_cache`, `game_state_cache`, `tick_cache`, plus `onboard_ball_cache` for on-robot vision.
- `robot/` — per-robot behaviors and primitives: `ball_nav`, `Movement`, `goalie`, `goal`, `coop`, etc. Drive calibration is applied in [ball_nav.py](src/TeamControl/robot/ball_nav.py).
- `Formation/` — team formations and strategic positioning.
- `dispatcher/` — turns high-level intent into wire packets.
- `harness/` — test/runner scaffolding (`harness.py`, `grSim_runner.py`, `smoke_test.py`, `csv_logger.py`).
- `process_workers/` — multiprocess workers (vision, world model, GC FSM, robot recv). See [docs/Multiprocessing.md](docs/Multiprocessing.md).
- `onboard_vision/` — receiver/observation/store for on-robot camera data.
- `ui/` — PyQt operator app: `main_window`, `dashboard_page`, `calibration_page`, etc.
- `plotter/` — visualization helpers.
- `bt/` — **v2 BT** (TurtleRabbitBT integration). `contracts/`, `skills/`, `trees/`, `coordinator.py`, plus `adapter.py` (WorldModel↔Snapshot, Intent→RobotCommand) and `run_bt_v2_process.py` (multiprocess runner). The legacy `behaviour_tree/` package at the repo root still ships alongside it.

## Config files at repo root
- `calibration.json` — drive calibration (speed scale + drift). Read at import by [ball_nav.py:41](src/TeamControl/robot/ball_nav.py#L41); written by the Calibration UI tab.
- `tuning.json` — runtime tuning knobs.
- `pyproject.toml` — package metadata / deps.

## Docs worth reading first
- [CONTRIBUTING.md](CONTRIBUTING.md)
- [docs/getting-started.md](docs/getting-started.md)
- [docs/HowToWriteCode.md](docs/HowToWriteCode.md)
- [docs/Multiprocessing.md](docs/Multiprocessing.md) — process worker architecture
- [docs/SSL-NetworkPorts.md](docs/SSL-NetworkPorts.md) — UDP ports / multicast groups
- [docs/maintainers.md](docs/maintainers.md)
- [docs/bt_contracts.md](docs/bt_contracts.md) — v2 BT contract types (Snapshot / Intent / MotionTarget / Blackboard)
- [docs/bt_v2_integration.md](docs/bt_v2_integration.md) — how the v2 BT (TurtleRabbitBT) plugs into the rest of the stack
- [docs/bt_v2_6v6_sim.md](docs/bt_v2_6v6_sim.md) — BT-vs-BT 6v6 launcher and its `sim_6v6.yaml`
- [src/TeamControl/network/proto2/README.md](src/TeamControl/network/proto2/README.md) — protobuf regeneration

## Environment
- Windows dev machine (PowerShell). Bash is available via the Bash tool.
- Python project; protobufs are pre-generated under `network/proto2/`.
- Windows builds: [scripts/build_windows.ps1](scripts/build_windows.ps1) bundles `calibration.json` alongside the binary.
- Tests live in [tests/](tests/) — see [tests/README.md](tests/README.md). Standalone smoke scripts at repo root: [test_grsim_send.py](test_grsim_send.py), [test_vision_recv.py](test_vision_recv.py).

## Gotchas
- Two unrelated "calibration" concepts: **drive calibration** (`calibration.json` + `ball_nav`) vs **camera calibration** (`CameraCalibration` in [SSL/vision/field.py:135](src/TeamControl/SSL/vision/field.py#L135), parsed from SSL-Vision geometry protobuf).
- Multiprocess workers don't share Python state — caches are passed via IPC. Don't assume singletons cross process boundaries.
- After editing `calibration.json` programmatically, call `ball_nav._reload_calibration()` so live behaviors pick up new values (UI already does this).
- `proto2/` files are generated — regenerate from `.proto` rather than hand-editing.