from multiprocessing import Process, Queue, Event, freeze_support
from TeamControl.process_workers.vision_runner import VisionProcess
from TeamControl.process_workers.gcfsm_runner import GCfsm
from TeamControl.world.model_manager import WorldModelManager
from TeamControl.process_workers.wm_runner import WMWorker
from TeamControl.SSL.grSim.sandbox_process import run_grsim_sandbox_process
from TeamControl.bt.run_bt_v2_process import run_bt_v2_process

from TeamControl.dispatcher.dispatch import Dispatcher
from TeamControl.utils.yaml_config import Config


def main():
    import sys
    freeze_support()
    vision_port = 10006
    is_running = Event()
    is_running.set()
    vision_q = Queue()
    gc_q = Queue()
    dispatcher_q = Queue()

    config_file = sys.argv[1] if len(sys.argv) > 1 else "ipconfig.yaml"
    preset = Config(config_file)

    vision_wkr = Process(
        target=VisionProcess.run_worker,
        args=(is_running, None, vision_q, True, vision_port),
    )
    gc_wkr = Process(
        target=GCfsm.run_worker,
        args=(is_running, None, gc_q, preset.us_yellow, preset.us_positive, preset.team_name),
    )
    wm_manager = WorldModelManager()
    wm_manager.start()
    wm = wm_manager.WorldModel()
    wmr = Process(
        target=WMWorker.run_worker,
        args=(is_running, None, wm, vision_q, gc_q),
    )
    bt = Process(
        target=run_bt_v2_process,
        args=(is_running, wm, dispatcher_q),
        kwargs={"config_file": config_file},
    )
    dispatcher = Process(
        target=Dispatcher.run_worker,
        args=(is_running, None, dispatcher_q, preset),
    )

    vision_wkr.start()
    gc_wkr.start()
    wmr.start()
    bt.start()
    dispatcher.start()

    vision_wkr.join()
    gc_wkr.join()
    wmr.join()
    bt.join()
    dispatcher.join()


if __name__ == "__main__":
    main()