import threading 
from multiprocessing import Event as MPEvent 
from TeamControl.network.ssl_sockets import grSimSender, grSimVision
from TeamControl.network.robot_command import RobotCommand

class GrSimRunner:
    def __init__(self, robot_id, is_yellow,
                 sim_ip = "127.0.0.1",
                 cmd_port = 20011,
                 vision_port = 10020):
        self.robot_id = robot_id
        self.is_yellow = bool(is_yellow)
        self.sim_ip = sim_ip
        self.cmd_port = cmd_port 
        self.vision_port = vision_port 
    
        self._sender = None 
        self._vision = None 
        self._is_running = None 
        self._thread = None 
        self._latest_state = None 
        self._started = False

    def _vision_loop(self): 
        # Background thread: convert vision packets into tuples and update _latest_state" 
        while self._is_running.is_set(): 
            try: 
                packet = self._vision.listen() 
                if packet is None: 
                    continue 
                robots = (packet.detection.robots_yellow if self.is_yellow
                          else packet.detection.robots_blue)
                for r in robots: 
                    if r.robot_id == self.robot_id: 
                        self._latest_state = (r.x,r.y, r.orientation)
                        break 
            except Exception as e: 
                print(f"[GrSimRunner] vision thread error: {e}")
    
    def start(self):
        if self._started: 
            raise RuntimeError("GrSim Runner already started - call stop() first")
        self._is_running = MPEvent() 
        self._is_running.set()
        self._sender = grSimSender(ip = self.sim_ip, port = self.cmd_port)
        self._vision = grSimVision(is_running = self._is_running, port = self.vision_port)
        self._thread = threading.Thread(target = self._vision_loop, daemon = True)
        self._thread.start()
        self._started = True

    def send(self, vx, vy, w, kick = 0, dribble = 0):
        if not self._started: 
            raise RuntimeError("GrSimRunner.send() called before start()")
        cmd = RobotCommand(robot_id = self.robot_id,
                           vx = vx, vy = vy, w = w, kick = kick, dribble = dribble,
                           isYellow = self.is_yellow)
        self._sender.send_robot_command(cmd)
    
    def read_state(self):
        return self._latest_state
    
    def stop(self):
        if not self._started: 
            return 
        self._is_running.clear()
        if self._thread is not None: 
            self._thread.join(timeout = 2.0)
        if self._vision is not None: 
            try: 
                self._vision.sock.close()
            except Exception: 
                pass 
        if self._sender is not None: 
            try: 
                self._sender.sock.close()
            except Exception: 
                pass 
        self._sender = None 
        self._vision = None 
        self._is_running = None 
        self._thread = None
        self._started = False 
