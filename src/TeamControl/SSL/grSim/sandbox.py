from multiprocessing import Process, Queue,Event
from TeamControl.process_workers.vision_runner import VisionProcess
from TeamControl.world.model_manager import WorldModelManager
from TeamControl.process_workers.wm_runner import WMWorker
# from TeamControl.utils.dummy_process import DummyReader
from TeamControl.SSL.grSim.sandbox_process import run_grsim_sandbox_process
from TeamControl.bt.run_bt_v2_process import run_bt_v2_process

# in multiprocessing this can only be a simple process

def main():
    use_sim = True
    vision_port = 10006
    is_running = Event()
    is_running.set()
    vision_q = Queue()
    # no game controller 
    gc_q = Queue()
    dispatcher_q = Queue()
    
    # inputs
    vision_wkr = Process(target=VisionProcess.run_worker, args=(is_running,None,vision_q,True,vision_port,))
    
    # world model
    wm_manager = WorldModelManager()
    wm_manager.start()
    wm = wm_manager.WorldModel()
    wmr = Process(target=WMWorker.run_worker, args=(is_running,None,wm,vision_q,gc_q,))
    sandbox = Process(target=run_grsim_sandbox_process, args=(wm,) )
    # new version
    bt = Process(target=run_bt_v2_process, args=(is_running, wm, dispatcher_q,))

    vision_wkr.start()
    wmr.start()
    # sandbox.start()
    bt.start()
    # some_other_process2.start()
    
    vision_wkr.join()
    wmr.join()
    # sandbox.join()
    bt.join()

    # some_other_process2.join()

if __name__ == "__main__":
    main()