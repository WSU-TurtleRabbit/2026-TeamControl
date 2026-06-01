"""Run both yellow and blue sandboxes from a single terminal.

Usage:
    python sandbox_both.py
    python sandbox_both.py ipconfig.yaml ipconfig_blue.yaml
"""
import multiprocessing
import sys
import time
from multiprocessing import Event, Process, Queue

from TeamControl.bt.run_bt_v2_process import run_bt_v2_process
from TeamControl.dispatcher.dispatch import Dispatcher
from TeamControl.process_workers.gcfsm_runner import GCfsm
from TeamControl.process_workers.vision_runner import VisionProcess
from TeamControl.process_workers.wm_runner import WMWorker
from TeamControl.utils.yaml_config import Config
from TeamControl.world.model_manager import WorldModelManager


def _start_team(is_running: Event, vision_q: Queue, config_file: str):
    preset = Config(config_file)
    gc_q = Queue()
    dispatcher_q = Queue()

    gc_wkr = Process(
        target=GCfsm.run_worker,
        args=(is_running, None, gc_q, preset.us_yellow, preset.us_positive, preset.team_name),
    )

    wm_manager = WorldModelManager()
    wm_manager.start()
    time.sleep(0.5)
    wm = wm_manager.WorldModel(us_yellow=preset.us_yellow, us_positive=preset.us_positive)

    wmr = Process(
        target=WMWorker.run_worker,
        args=(is_running, None, wm, vision_q, gc_q),
    )

    bt = Process(
        target=run_bt_v2_process,
        args=(is_running, wm, dispatcher_q, None, config_file),
    )

    dispatcher = Process(
        target=Dispatcher.run_worker,
        args=(is_running, None, dispatcher_q, preset),
    )

    gc_wkr.start()
    wmr.start()
    bt.start()
    dispatcher.start()

    return wm_manager, wm, [gc_wkr, wmr, bt, dispatcher]


def main():
    multiprocessing.freeze_support()

    yellow_config = sys.argv[1] if len(sys.argv) > 1 else "ipconfig.yaml"
    blue_config = sys.argv[2] if len(sys.argv) > 2 else "ipconfig_blue.yaml"

    is_running = Event()
    is_running.set()

    # Single shared vision listener for both teams
    vision_q_yellow = Queue()
    vision_q_blue = Queue()

    vision_wkr_yellow = Process(
        target=VisionProcess.run_worker,
        args=(is_running, None, vision_q_yellow, True, 10006),
    )
    vision_wkr_blue = Process(
        target=VisionProcess.run_worker,
        args=(is_running, None, vision_q_blue, True, 10006),
    )

    vision_wkr_yellow.start()
    vision_wkr_blue.start()

    yellow_mgr, yellow_wm, yellow_procs = _start_team(is_running, vision_q_yellow, yellow_config)
    blue_mgr, blue_wm, blue_procs = _start_team(is_running, vision_q_blue, blue_config)

    for p in [vision_wkr_yellow, vision_wkr_blue] + yellow_procs + blue_procs:
        p.join()


if __name__ == "__main__":
    main()
