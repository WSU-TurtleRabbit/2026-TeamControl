import sys
import time
from multiprocessing import Process, Queue, Event, freeze_support
from TeamControl.process_workers.vision_runner import VisionProcess
from TeamControl.process_workers.gcfsm_runner import GCfsm
from TeamControl.world.model_manager import WorldModelManager
from TeamControl.process_workers.wm_runner import WMWorker
from TeamControl.SSL.grSim.sandbox_process import run_grsim_sandbox_process
from TeamControl.bt.run_bt_v2_process import run_bt_v2_process

from TeamControl.dispatcher.dispatch import Dispatcher
from TeamControl.utils.yaml_config import Config


def _wait_for_vision(wm, timeout=15.0):
    """Block until the world model has received at least one vision frame."""
    deadline = time.time() + timeout
    print("[sandbox] Waiting for first vision frame...", flush=True)
    while time.time() < deadline:
        frame = wm.get_latest_frame()
        if frame is not None:
            n_yellow = sum(1 for _ in frame.robots_yellow)
            n_blue   = sum(1 for _ in frame.robots_blue)
            ball     = frame.ball
            ball_str = f"({ball.x:.0f}, {ball.y:.0f}) mm" if ball is not None else "not visible"
            print(
                f"[sandbox] Vision confirmed — "
                f"yellow robots: {n_yellow}, blue robots: {n_blue}, ball: {ball_str}",
                flush=True,
            )
            return True
        time.sleep(0.05)
    print(f"[sandbox] ERROR: No vision data after {timeout:.0f}s — is grSim running?", flush=True)
    return False


def main():
    freeze_support()
    vision_port = 10006
    is_running = Event()
    is_running.set()
    vision_q = Queue()
    gc_q = Queue()
    dispatcher_q = Queue()

    config_file = sys.argv[1] if len(sys.argv) > 1 else "ipconfig.yaml"
    preset = Config(config_file)

    wm_manager = WorldModelManager()
    wm_manager.start()
    wm = wm_manager.WorldModel(us_yellow=preset.us_yellow, us_positive=preset.us_positive)

    vision_wkr = Process(
        target=VisionProcess.run_worker,
        args=(is_running, None, vision_q, True, vision_port),
        name="VisionProcess",
    )
    gc_wkr = Process(
        target=GCfsm.run_worker,
        args=(is_running, None, gc_q, preset.us_yellow, preset.us_positive, preset.team_name),
        name="GCfsm",
    )
    wmr = Process(
        target=WMWorker.run_worker,
        args=(is_running, None, wm, vision_q, gc_q),
        name="WMWorker",
    )
    bt = Process(
        target=run_bt_v2_process,
        args=(is_running, wm, dispatcher_q),
        kwargs={"config_file": config_file},
        name="BT",
    )
    dispatcher = Process(
        target=Dispatcher.run_worker,
        args=(is_running, None, dispatcher_q, preset),
        name="Dispatcher",
    )

    # Start vision + world model first, wait for a frame before launching BT
    for p in (vision_wkr, gc_wkr, wmr, dispatcher):
        p.start()

    if not _wait_for_vision(wm, timeout=15.0):
        print("[sandbox] Aborting — no vision data.", flush=True)
        is_running.clear()
        for p in (vision_wkr, gc_wkr, wmr, dispatcher):
            p.join(timeout=5)
        sys.exit(1)

    bt.start()

    # Watchdog: print a warning if any process dies unexpectedly
    try:
        while is_running.is_set():
            for p in (vision_wkr, gc_wkr, wmr, bt, dispatcher):
                if not p.is_alive():
                    print(f"[sandbox] WARNING: process '{p.name}' died", flush=True)
            time.sleep(2.0)
    except KeyboardInterrupt:
        print("[sandbox] Shutting down...", flush=True)
        is_running.clear()

    for p in (vision_wkr, gc_wkr, wmr, bt, dispatcher):
        p.join(timeout=5)


if __name__ == "__main__":
    main()