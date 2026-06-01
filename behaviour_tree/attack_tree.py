from time import time
import py_trees
import numpy as np
from behaviour_tree.cmd_mgr import CommandManager
from TeamControl.network.robot_command import RobotCommand
from TeamControl.world.transform_cords import world2robot
from behaviour_tree.test_tree import GoToTarget
from behaviour_tree.velocity import Mode, turn_to_target

#dribble_threshold=0.07

class AttackSeq(py_trees.composites.Sequence):
    def __init__(self,wm,dispatcher_q,robot_id,isYellow,isPositive=None,logger=None):
        if logger is not None:
            self.logger = logger
        name = "Attack Sequence"
        self.wm=wm
        self.dispatcher_q = dispatcher_q            
        self.robot_id = robot_id
        self.isYellow = isYellow
        self.isPositive = isPositive if isPositive is not None else isYellow
        self.cmd_mgr = CommandManager(isYellow=isYellow,robot_id=robot_id,dispatcher_q=dispatcher_q)
        
        self.bb = py_trees.blackboard.Client(name=name)
        super(AttackSeq,self).__init__(name=name,memory=True)
        
    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.bb.register_key(key="robot_id", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="isYellow", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="isPositive", access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="cmd_mgr", access=py_trees.common.Access.WRITE)
        self.bb.robot_id = self.robot_id
        self.bb.isYellow = self.isYellow
        self.bb.isPositive = self.isPositive    
        self.bb.cmd_mgr = self.cmd_mgr
        
    
        self.add_children([
            GetBallSelector(self.dispatcher_q),
            # NextActionSelector(self.dispatcher_q)
            
    
        ])
    def initialise(self):
        
        for c in self.children:
            c.setup()
            
class GetBallSelector(py_trees.composites.Selector):
    def __init__(self,dispatcher_q):
        super().__init__(name="GetBallSelector", memory=True)
        self.bb = py_trees.blackboard.Client(name=self.name)
        
        self.add_children([
            HasBall(dribble_threshold=150.0),
            AcquireBall(dribble_threshold=150.0, facing_threshold=0.015, dispatcher_q=dispatcher_q),
            GoToBall(dribble_threshold=150.0, mode=Mode.Percision, dispatcher_q=dispatcher_q)
            
        ])
        
        
class HasBall(py_trees.behaviour.Behaviour):
    def __init__(self, dribble_threshold:float):
        super().__init__("HasBall")
        self.dribble_threshold = dribble_threshold
        self.bb = py_trees.blackboard.Client(name=self.name)
        self.bb.register_key(key="robot_pos",access=py_trees.common.Access.READ)
        self.bb.register_key(key="target_pos",access=py_trees.common.Access.READ)
        self.bb.register_key("dribble", py_trees.common.Access.WRITE)
        self.bb.register_key("vx", py_trees.common.Access.WRITE)
        

    def update(self):
        relative_target_arr = world2robot(robot_position=self.bb.robot_pos, target_position=self.bb.target_pos)
        relative_distance = np.linalg.norm(relative_target_arr) 
        
        if relative_distance < self.dribble_threshold:
            print("Has ball")
            self.bb.dribble = 1
            self.bb.vx = 0.0
            return py_trees.common.Status.SUCCESS
        else:
            print("Does not have ball")
            return py_trees.common.Status.FAILURE
        
class AcquireBall(py_trees.behaviour.Behaviour):
    def __init__(self, dribble_threshold:float, facing_threshold:float, dispatcher_q):
        super().__init__("AcquireBall")
        self.dribble_threshold = dribble_threshold
        self.facing_threshold = facing_threshold
        self.dispatcher_q = dispatcher_q

        self.bb = py_trees.blackboard.Client(name=self.name)
        self.bb.register_key(key="robot_pos",access=py_trees.common.Access.READ)
        self.bb.register_key(key="target_pos",access=py_trees.common.Access.READ)
        self.bb.register_key("dribble", py_trees.common.Access.WRITE)
        self.bb.register_key("vx", py_trees.common.Access.WRITE)

    def update(self):
        relative_target_arr = world2robot(robot_position=self.bb.robot_pos, target_position=self.bb.target_pos)
        relative_distance = np.linalg.norm(relative_target_arr) 
        
        angle_diff = np.arctan2(relative_target_arr[1], relative_target_arr[0])
        
        if relative_distance <= self.dribble_threshold and abs(angle_diff) <= self.facing_threshold:

            print("Acquiring ball")
            self.bb.dribble=1
            self.bb.vx=0.1
            send_robot_command(self.dispatcher_q, self.bb, runtime=0.5)
            
            return py_trees.common.Status.SUCCESS
        else:   
            print("Not acquiring ball")
            self.bb.dribble=0
            self.bb.vx=0.0  
            send_robot_command(self.dispatcher_q, self.bb, runtime=0.5)
            return py_trees.common.Status.FAILURE

class GoToBall(py_trees.behaviour.Behaviour):
    def __init__(self,dribble_threshold:float, mode: Mode, dispatcher_q):
        super().__init__("GoToBall")
        self.goToTarget=GoToTarget(threshold=dribble_threshold, mode=mode)
        self.dispatcher_q = dispatcher_q
        
        self.bb = py_trees.blackboard.Client(name=self.name)
        
        self.bb.register_key(key="robot_id", access=py_trees.common.Access.READ)
        self.bb.register_key(key="isYellow", access=py_trees.common.Access.READ)
        self.bb.register_key(key="vx", access=py_trees.common.Access.READ)
        self.bb.register_key(key="vy", access=py_trees.common.Access.READ)
        self.bb.register_key(key="w", access=py_trees.common.Access.READ)
        self.bb.register_key(key="kick", access=py_trees.common.Access.READ)
        self.bb.register_key(key="dribble", access=py_trees.common.Access.READ)

    def setup(self, logger=None):
        if logger:
            self.logger = logger
        self.goToTarget.setup(logger)

    def update(self):
        self.goToTarget.tick_once()
        send_robot_command(self.dispatcher_q, self.bb, runtime=0.5)
        print("Going to ball")
        return py_trees.common.Status.SUCCESS
    
    
def send_robot_command(dispatcher_q, bb, runtime=2):
    """
    Sends a RobotCommand using the blackboard values.

    Args:
        dispatcher_q: queue.Queue or similar, where commands are sent.
        bb: py_trees.blackboard.Client instance with required keys.
        runtime: float, duration the command should run.
    
    Returns:
        bool: True if command was successfully sent, False if queue is full.
    """
    robot_id = bb.robot_id
    isYellow = bb.isYellow

    vx = bb.vx if bb.exists("vx") else 0.0
    vy = bb.vy if bb.exists("vy") else 0.0
    w = bb.w if bb.exists("w") else 0.0
    kick = bb.kick if bb.exists("kick") else 0
    dribble = bb.dribble if bb.exists("dribble") else 0

    command = RobotCommand(
        robot_id=robot_id,
        vx=vx, vy=vy, w=w,
        kick=kick,
        dribble=dribble,
        isYellow=isYellow
    )

    packet = (command, runtime)

    if not dispatcher_q.full():
        dispatcher_q.put(packet)
        return True
    else:
        print("[send_robot_command] Dispatcher queue is full")
        return False
# class NextActionSelector(py_trees.composites.Selector):
#     def __init__(self, dispatcher_q):
#         super().__init__(name="NextActionSelector", memory=True)
#         self.bb = py_trees.blackboard.Client(name=self.name)

class CanScore(py_trees.composites.Sequence):
    def __init__(self):
        super().__init__("CanScore")
        self.bb = py_trees.blackboard.Client(name=self.name)
        self.bb.register_key(key="robot_pos", access=py_trees.common.Access.READ)
        self.bb.register_key(key="target_pos", access=py_trees.common.Access.READ)
        
    def setup(self,**kwargs):
        super().setup(**kwargs)
        # outline all variables here
        
        
        self.add_children([
            AlignBallWithGoal(dispatcher_q=self.dispatcher_q),
            Kick(kick_threshold=0.07, kick_angle=0.1)
            
        ])
        
    def initialise(self):
        
        for c in self.children:
            c.setup()
        
class AlignBallWithGoal(py_trees.behaviour.Behaviour):
    def __init__(self, mode: Mode):
        name = "AlignBallWithGoal"
        super().__init__(name)
        self.mode = mode
        self.bb = py_trees.blackboard.Client(name=name)
        
    def setup(self):
        self.bb.register_key("robot_pos",access=py_trees.common.Access.READ) # in GetRobotIDPosition
        self.bb.register_key("target_pos", access=py_trees.common.Access.READ)
        self.bb.register_key("w",access=py_trees.common.Access.WRITE)
        self.bb.register_key("d_theta",access=py_trees.common.Access.WRITE)
        self.bb.register_key("dribble",access=py_trees.common.Access.WRITE)
        
    def update(self):
        w=turn_to_target(robot_pos=self.bb.robot_pos,
                         target_pos=self.bb.target_pos,
                         mode=self.mode)
        self.bb.w = w
        
        relative = world2robot(
            robot_position=self.bb.robot_pos,
            target_position=self.bb.target_pos
        )
        self.bb.d_theta = np.arctan2(relative[1], relative[0])
        self.bb.dribble = 1
        
        
        return py_trees.common.Status.SUCCESS


