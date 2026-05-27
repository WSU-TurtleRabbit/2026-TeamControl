from TeamControl.network.robot_command import RobotCommand
from TeamControl.robot.constants import CMD_RUNTIME
from .cmd_mgr import CommandManager
import py_trees
import random

class GetWorldPositionUpdate(py_trees.behaviour.Behaviour):
    def __init__(self,wm,isYellow=True):
        name = "GetWorldPositionUpdate"
        self.wm = wm
        self.isYellow = isYellow
        super().__init__(name)
    
    def setup(self,logger=None):
        if logger is not None: # use this instead
            self.logger = logger 
        self.version = self.wm.get_version()
        self.frame = self.wm.get_latest_frame()
        # self.ball_last_known = (0,0)
        self.bb = py_trees.blackboard.Client(name="GetWorldPositionUpdate")
        self.bb.register_key(key="ball_pos", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="our_robots", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="isYellow", access=py_trees.common.Access.WRITE)
        self.bb.isYellow = self.isYellow
        self.bb.ball_pos = (0,0)
        if self.frame is not None:
            self.bb.our_robots = self.frame.get_yellow_robots(isYellow=self.isYellow)
        else: 
            self.bb.our_robots = None
        print(f"[GetWorldPositionUpdate] is self.frame() None?  {self.frame == None}")

    def initialise(self):
        pass
        
    def update(self) -> py_trees.common.Status:
        self.isYellow = self.bb.isYellow   

        new_version = self.wm.get_version()
        if self.version.value < new_version.value:
            self.version = new_version
            self.frame = self.wm.get_latest_frame()
            if self.frame is not None:
                if self.frame.ball is not None:
                    # self.ball_last_known = self.frame.ball.position
                    # self.bb.ball_pos = self.ball_last_known
                    # print(self.ball_last_known)
                    self.bb.ball_pos = self.frame.ball.position
                    self.bb.our_robots = self.frame.get_yellow_robots(isYellow=self.isYellow)
                    
            return py_trees.common.Status.SUCCESS
        # otherwise keep running
        return py_trees.common.Status.RUNNING

# mock behaviour for testing main tree
class GetBallPosition(py_trees.behaviour.Behaviour):
    def __init__(self, robot_id):
        name = f"GetBallPosition (Robot ID {robot_id})"
        self.robot_id = robot_id
        self.condition = random.randint(0, 1) # 0: failure, 1: success
        super().__init__(name)
        
    def setup(self,logger=None):
        if logger is not None:
            self.logger = logger
        self.bb = py_trees.blackboard.Client(name="GetBallPosition")
        self.bb.register_key(key="ball_pos", access=py_trees.common.Access.READ)
        # self.bb.register_key(key="ball_position", access=py_trees.common.Access.WRITE)
        
    def update(self) -> py_trees.common.Status:
        ball_pos = self.bb.ball_pos
        if ball_pos is not None and self.condition == 1:
            # self.bb.ball_position = ball_pos
            self.logger.info(f"[GetBallPosition] Ball position: {ball_pos}\tCONDITION PASS")
            return py_trees.common.Status.SUCCESS
        else:
            # self.bb.ball_position = (0,0)
            # self.logger.info(f"[GetBallPosition] Failed to get ball position. CONDITION: {self.condition}")
            return py_trees.common.Status.FAILURE

class SendRobotCommand(py_trees.behaviour.Behaviour):
    def __init__(self,dispatcher_q,runtime=CMD_RUNTIME):
        name = "SendRobotCommand"
        self.dispatcher_q = dispatcher_q
        self.runtime = runtime
        super().__init__(name)
    
    def setup(self,logger=None):
        if logger is not None:
            self.logger = logger
        self.bb = py_trees.blackboard.Client(name="SendRobotCommand")
        self.bb.register_key(key="robot_id", access=py_trees.common.Access.READ)
        self.bb.register_key(key="isYellow", access=py_trees.common.Access.READ)
        self.bb.register_key(key="vx", access=py_trees.common.Access.READ)
        self.bb.register_key(key="vy", access=py_trees.common.Access.READ)
        self.bb.register_key(key="w", access=py_trees.common.Access.READ)
        self.bb.register_key(key="kick", access=py_trees.common.Access.READ)
        self.bb.register_key(key="dribble", access=py_trees.common.Access.READ)
        self.bb.register_key(key="command", access=py_trees.common.Access.WRITE)
        
        
        
    def initialise(self):
        # preset
        self.last_command = getattr(self.bb,"command",None)
        
        robot_id = self.bb.robot_id
        isYellow = self.bb.isYellow
        vx = self.bb.vx if self.bb.exists("vx") else 0.0
        vy = self.bb.vy if self.bb.exists("vy") else 0.0
        w = self.bb.w if self.bb.exists("w") else 0.0
        kick = self.bb.kick if self.bb.exists("kick") else 0
        dribble = self.bb.dribble if self.bb.exists("dribble") else 0
        self.bb.command = RobotCommand(robot_id=robot_id,
                                   vx=vx,vy=vy,w=w,
                                   kick=kick,dribble=dribble,
                                   isYellow=isYellow
                                   ) 
        
    
    
    def update(self) -> py_trees.common.Status:
        command = self.bb.command
        # if self.last_command.to_dict() == command.to_dict() : 
        #     print(f"[SendRobotCommand] No new command")
        #     return py_trees.common.Status.SUCCESS
        
        packet = (command, self.runtime)
        # print(f"[SendRobotCommand] Sending command: {command}")
        if not self.dispatcher_q.full():
            self.dispatcher_q.put(packet)
            self.last_command = command
            return py_trees.common.Status.SUCCESS
        
        else:
            print("[SendRobotCommand] Dispatcher queue is full, cannot send command")
            return py_trees.common.Status.FAILURE
        # this is always success until we put more stuff to check here.
        

class GetRobotIDPosition(py_trees.behaviour.Behaviour):
    def __init__(self,robot_id:int=0):
        self.robot_id = robot_id
        name = f"GetRobotIDPosition{robot_id}"
        super().__init__(name)
        
        
    def setup(self,logger=None):
        if logger is not None:
            self.logger = logger
        self.bb = py_trees.blackboard.Client(name=f"GetRobotIDPosition{self.robot_id}")
        # self.bb.register_key(key="robot_id", access=py_trees.common.Access.READ)
        self.bb.register_key(key="our_robots", access=py_trees.common.Access.READ)
        self.bb.register_key(key="robot_pos", access=py_trees.common.Access.WRITE)
        pass
    
    def update(self) -> py_trees.common.Status:
        # gets the robot position base on robot_id
        our_robots = self.bb.our_robots
        if our_robots is not None and isinstance(our_robots[self.robot_id],object):
            robot = our_robots[self.robot_id]
            # store position
            self.bb.robot_pos = robot.position
            self.logger.info(f"[GetRobotIDPosition] Robot {self.robot_id} position: {robot.position}")
            return py_trees.common.Status.SUCCESS
        
        else:
            # otherwise 0,0
            self.bb.robot_pos = (0,0)
            return py_trees.common.Status.FAILURE


################# PLACEHOLDERS ###################

class GoToBall(py_trees.behaviour.Behaviour):
    def __init__(self):
        name = "GoToBall"
        super().__init__(name)
        
    def setup(self):
        pass
    
    def update(self) -> py_trees.common.Status:
        # this is where the go to ball logic will go
        # simple random success/failure
        if random.randint(0,1) == 1:
            print("[GoToBall] Successfully went to the ball!")
            return py_trees.common.Status.SUCCESS
        else:
            print("[GoToBall] Failed to go to the ball.")
            return py_trees.common.Status.FAILURE

class GoToFormation(py_trees.behaviour.Behaviour):
    def __init__(self):
        name = "GoToFormation"
        super().__init__(name)
        
    def setup(self):
        pass
    
    def update(self) -> py_trees.common.Status:
        if random.randint(0,1) == 1:
            print("[GoToFormation] Successfully went to formation!")
            return py_trees.common.Status.SUCCESS
        else:
            print("[GoToFormation] Failed to go to formation.")
            return py_trees.common.Status.FAILURE
 
class GoToInterception(py_trees.behaviour.Behaviour):
    def __init__(self):
        name = "GoToInterception"
        super().__init__(name)
        
    def setup(self):
        pass
    
    def update(self) -> py_trees.common.Status:
        if random.randint(0,1) == 1:
            print("[GoToInterception] Successfully went to interception!")
            return py_trees.common.Status.SUCCESS
        else:
            print("[GoToInterception] Failed to go to interception.")
            return py_trees.common.Status.FAILURE

class PassBall(py_trees.behaviour.Behaviour):
    def __init__(self):
        name = "PassBall"
        super().__init__(name)
        
    def setup(self):
        pass
    
    def update(self) -> py_trees.common.Status:
        if random.randint(0,1) == 1:
            print("[PassBall] Successfully passed the ball!")
            return py_trees.common.Status.SUCCESS
        else:
            print("[PassBall] Failed to pass the ball.")
            return py_trees.common.Status.FAILURE   
        
class GetBall(py_trees.behaviour.Behaviour):
    def __init__(self):
        name = "GetBall"
        super().__init__(name)
        
    def setup(self):
        pass
    
    def update(self) -> py_trees.common.Status:
        if random.randint(0,1) == 1:
            print("[GetBall] Successfully got the ball!")
            return py_trees.common.Status.SUCCESS
        else:
            print("[GetBall] Failed to get the ball.")
            return py_trees.common.Status.FAILURE   
        
class RotateWithBall(py_trees.behaviour.Behaviour):
    def __init__(self):
        name = "RotateWithBall"
        super().__init__(name)
        
    def setup(self):
        pass
    
    def update(self) -> py_trees.common.Status:
        if random.randint(0,1) == 1:
            print("[RotateWithBall] Successfully rotated with the ball!")
            return py_trees.common.Status.SUCCESS
        else:
            print("[RotateWithBall] Failed to rotate with the ball.")
            return py_trees.common.Status.FAILURE

class KickBall(py_trees.behaviour.Behaviour):
    def __init__(self):
        name = "KickBall"
        super().__init__(name)
        
    def setup(self):
        pass
    
    def update(self) -> py_trees.common.Status:
        if random.randint(0,1) == 1:
            print("[KickBall] Successfully kicked the ball!")
            return py_trees.common.Status.SUCCESS
        else:
            print("[KickBall] Failed to kick the ball.")
            return py_trees.common.Status.FAILURE