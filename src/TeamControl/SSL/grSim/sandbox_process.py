from TeamControl.network.ssl_sockets import grSimSender
from TeamControl.network.robot_command import RobotCommand 
from TeamControl.robot.Movement import RobotMovement

from TeamControl.world.time_to_intercept import time_to_intercept
from TeamControl.world.velocity_est import velocity_est

class GrSimSandbox:
    def __init__(self,wm):
        self.wm = wm
        self.sim_ip = "127.0.0.1"
        self.cmd_listen_port = 20011
        self.is_yellow = True
        self.robot_id = 1
        self.sender = grSimSender(ip=self.sim_ip,port=self.cmd_listen_port)
        self.version = 0 # vision version
        self.ball_last_known = (0,0)
        self.movement = RobotMovement()

    def running(self):
        vx, vy, w= 0, 0 , 0
        while True:
            # check version update 
            has_update = self.get_update()
            # This is running
            
            if has_update is True:
                robot_pos, ball_hist = self.get_objects()
                # do functions here 
                time_to_intercept(ball_pos=self.ball_last_known,target=None, ball_hist=ball_hist)
                velocity_est(ball_hist = ball_hist)
                # Robot : calculate velocity to target : ball
                vx, vy, w= self.movement.velocity_to_target(robot_pos=robot_pos, target=ball_hist[0])
            
            # send the command after
            cmd = RobotCommand(self.robot_id, vx, vy, w, 0, 0) 
            self.sender.send_robot_command(cmd)
            
    def get_update(self):
        # get_update
        if self.version < self.wm.get_version():
            self.version = self.wm.get_version()
            self.frame = self.wm.get_latest_frame()
            self.frames =self.wm.get_last_n_frames(10) # you can pick how many frames to get
            if self.frame != None:
                # verified that you have a frame
                return True
        return False
    
    def get_objects(self):
        # getting a robot object 
        robot_obj = self.frame.get_yellow_robots(isYellow=self.is_yellow,robot_id=self.robot_id) 
        # getting a team of robots that is yellow
        robots = self.frame.get_yellow_robots(isYellow=self.is_yellow,robot_id=None)
        # accessing the specific robot position
        robot_pos = robot_obj.position
        # use this ball position if the ball position is not None otherwise use the last known position
        # ball = self.frame.ball.position if self.frame.ball.position is not None else self.ball_last_known
        # if self.frame.ball.position is not None:
        #     # update this
        self.ball_last_known = self.frame.ball.position
        # getting ball history
        ball_hist = []

        for f in self.frames: 
            ball_hist.append(f.ball.position)
            
        # select what you want to return    
        return robot_pos, ball_hist
    

## running as a multiprocessing process
def run_grsim_sandbox_process(wm):
    sandbox = GrSimSandbox(wm)
    sandbox.running()