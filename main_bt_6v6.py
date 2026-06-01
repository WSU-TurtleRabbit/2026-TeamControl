#!/usr/bin/env python
"""6v6 BT-vs-BT launcher — two independent v2 behaviour-tree processes,
one driving yellow and one driving blue, both pointed at the same grSim.

Pipeline:
    Vision   → vision_q  → WMWorker → wm (shared)
    wm (yellow view) → run_bt_v2_process(is_yellow=True)  ─┐
    wm (blue   view) → run_bt_v2_process(is_yellow=False) ─┤→ dispatch_q → Dispatcher → grSim

Network / send settings come from ``ipconfig.yaml``.
6v6-specific settings (per-team robot IDs, per-robot roles, tick period)
come from ``src/TeamControl/utils/sim_6v6.yaml``.

Type "exit" (or Ctrl+C) to stop.
"""
from __future__ import annotations

from multiprocessing import Event, Process, Queue

from TeamControl.bt.run_bt_v2_process import run_bt_v2_process
from TeamControl.dispatcher.dispatch import Dispatcher
from TeamControl.process_workers.vision_runner import VisionProcess
from TeamControl.process_workers.wm_runner import WMWorker
from TeamControl.utils.sim_config import Sim6v6Config
from TeamControl.utils.yaml_config import Config
from TeamControl.world.model_manager import WorldModelManager


def main() -> None:
    preset = Config()
    sim = Sim6v6Config()
    logger = None

    is_running = Event()
    is_running.set()

    vision_q = Queue()
    gc_q = Queue()
    recv_q = Queue()
    dispatch_q = Queue()

    wm_manager = WorldModelManager()
    wm_manager.start()
    wm = wm_manager.WorldModel()

    procs = [
        Process(target=VisionProcess.run_worker,
                args=(is_running, logger, vision_q,
                      preset.use_grSim_vision, preset.vision[1])),
        Process(target=WMWorker.run_worker,
                args=(is_running, logger, wm, vision_q, gc_q,
                      recv_q, {})),
        Process(target=Dispatcher.run_worker,
                args=(is_running, logger, dispatch_q, preset)),
        # Yellow BT
        Process(target=run_bt_v2_process,
                args=(is_running, wm, dispatch_q),
                kwargs=dict(is_yellow=True,
                            robot_ids=sim.yellow_ids,
                            role_assignment=sim.roles,
                            tick_period=sim.tick_period),
                name="bt_yellow"),
        # Blue BT
        Process(target=run_bt_v2_process,
                args=(is_running, wm, dispatch_q),
                kwargs=dict(is_yellow=False,
                            robot_ids=sim.blue_ids,
                            role_assignment=sim.roles,
                            tick_period=sim.tick_period),
                name="bt_blue"),
    ]

    print(f"[main_bt_6v6] starting — yellow={sim.yellow_ids} blue={sim.blue_ids}")
    print(f"[main_bt_6v6] roles={ {k: v.name for k, v in sim.roles.items()} } "
          f"tick={sim.tick_period}s")
    for p in procs:
        p.start()

    try:
        while is_running.is_set():
            if input("Type 'exit' to quit: ").strip().lower() == "exit":
                break
    except KeyboardInterrupt:
        pass

    print("[main_bt_6v6] shutting down...")
    is_running.clear()
    for p in procs:
        p.join(timeout=5)
    print("[main_bt_6v6] done")


if __name__ == "__main__":
    main()
