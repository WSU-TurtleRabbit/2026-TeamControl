import os
import time

from TeamControl.harness.csv_logger import CSVLogger
from TeamControl.harness.grSim_runner import GrSimRunner
from TeamControl.harness.constants import SIM_IP, CMD_PORT, VISION_PORT

DEFAULT_LOG_DIR = os.path.join(os.path.dirname(__file__), "logs")
DEFAULT_COLS = ["t_ms", "x_pos", "y_pos", "theta_pos",
                "cmd_vx", "cmd_vy", "cmd_w"]

class Harness:
    def __init__(self, robot_id, is_yellow,
                 log_dir = DEFAULT_LOG_DIR,
                 columns = DEFAULT_COLS,
                 sim_ip = SIM_IP,
                 cmd_port = CMD_PORT,
                 vision_port = VISION_PORT):
        self._runner = GrSimRunner(robot_id, is_yellow,
                                   sim_ip = sim_ip,
                                   cmd_port = cmd_port,
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
        position = self._runner.read_position()
        if position is None:
            x_pos = y_pos = theta_pos = ""
        else:
            x_pos, y_pos, theta_pos = position
        t_ms = (time.monotonic() - self._t0) * 1000
        self._logger.log(t_ms=t_ms, x_pos=x_pos, y_pos=y_pos, theta_pos=theta_pos,
                         cmd_vx=vx, cmd_vy=vy, cmd_w=w)

    def read_position(self):
        return self._runner.read_position()

    def set_logging(self, on):
        self._logger.set_enabled(on)

    def stop(self):
        self._logger.stop()
        self._runner.stop()
        self._t0 = None
