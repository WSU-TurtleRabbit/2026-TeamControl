import os 
import time 

from TeamControl.harness.csv_logger import CSVLogger
from TeamControl.harness.grSim_runner import GrSimRunner

DEFAULT_LOG_DIR = DEFAULT_LOG_DIR = os.path.join(os.path.dirname(__file__), "logs")
DEFAULT_COLS = ["t", "state_x", "state_y", "state_theta",
                      "cmd_vx", "cmd_vy", "cmd_w"]

class Harness: 
    def __init__(self, robot_id, is_yellow,
                 log_dir = DEFAULT_LOG_DIR,
                 columns = DEFAULT_COLS,
                 sim_ip = "127.0.0.1",
                 cmd_port = 20011,
                 vision_port = 10020):
        self._runner = GrSimRunner(robot_id, is_yellow,
                                   sim_ip = sim_ip,
                                   cmd_port= cmd_port,
                                   vision_port = vision_port)
        self._logger = CSVLogger(log_dir, columns)
        self._t0 = None

    def start(self, test_description):
        self._runner.start()
        path = self._logger.start(test_description)
        self._t0 = time.monotonic()
        return path

    def send(self, vx, vy, w, kick = 0, dribble = 0):
        self._runner.send(vx, vy, w, kick = kick, dribble = dribble)
        state = self._runner.read_state()
        if state is None: 
            sx = sy = stheta = ""
        else: 
            sx, sy, stheta = state
        t = time.monotonic() - self._t0
        self._logger.log(t=t, state_x = sx, state_y = sy, state_theta = stheta,
                         cmd_vx = vx, cmd_vy = vy, cmd_w = w)
    
    def read_state(self):
        return self._runner.read_state()
    
    def set_logging(self, on):
        self._logger.set_enabled(on)
    
    def stop(self):
        self._logger.stop()
        self._runner.stop()
        self._t0 = None 

         