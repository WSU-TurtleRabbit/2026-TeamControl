

from TeamControl.network.robot_command import RobotCommand
from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.Movement import RobotMovement
from TeamControl.utils.Logger import LogSaver 

from TeamControl.world.model import WorldModel
from TeamControl.SSL.vision.frame import Frame
from multiprocessing import Queue

import time
class follow_ball_dummy():
    def __init__(self, dispatch_q:Queue, 
                 wm:WorldModel,
                 robot_id:int,
                 us_yellow):
        self.wm:WorldModel = wm
        self.dispatch_q = dispatch_q
        self.robot_id = robot_id
        self.us_yellow = us_yellow
        self.last_version = 0
        self.logs = LogSaver(process_name="rc_Process",id=robot_id)
        self.movement = RobotMovement()

        
        
    def run(self):
        if self.robot_id is None:
            self.logs.error("ROBOT ID IS NONE EXITING . . .")
        last_update = time.time()
        vx,vy,w,k,d = 0,0,0,0,0
        frame = None
        while True:

            current_version:int = self.wm.get_version()
            # print(current_version)
            if current_version > self.last_version:
                frame= self.wm.get_latest_frame()
                self.logs.debug(f"{time.time() - last_update}, {frame.frame_number}")
                last_update = time.time()
                self.last_version = current_version
            try:
                robot = frame.get_yellow_robots(isYellow=self.us_yellow,robot_id=self.robot_id)
                robot_pos = robot.position
                ball = frame.ball.position
                
                
            except Exception :
                robot_pos = 0,0,0
                ball = 0,0
            vx, vy,w = self.movement.velocity_to_target(robot_pos=robot_pos, target=ball)

            cmd = RobotCommand(self.robot_id, vx, vy, w, 0, 0)
            # puts command into queue
            self.dispatch_q.put((cmd, 0.2)) # 0.5 seconds runtime
                        
                

    
def run_follow_ball_dummy(dispatch_q,wm:WorldModel,robot_id,is_yellow):
    dummy = follow_ball_dummy(dispatch_q, wm=wm,robot_id=robot_id,us_yellow=is_yellow)
    dummy.run()
