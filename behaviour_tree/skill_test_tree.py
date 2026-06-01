"""
Skill test tree — drop-in replacement for TestTreeSeq.

Wires a single skill node into the standard pipeline:
    GetWorldPositionUpdate -> GetRobotIDPosition -> <skill> -> SendRobotCommand

Usage (pick one skill to test):

    from behaviour_tree.skill_test_tree import SkillTestSeq
    from behaviour_tree.skill_nodes import MoveTo, KickAt, ReceiveBall, DribbleBackwards

    # move robot 0 to centre field
    skill = MoveTo("MoveTo_R0", target_pos=(0, 0))

    # kick toward opponent goal (robot 0)
    skill = KickAt("KickAt_R0", aim_pos=(4500, 0))

    # wait for ball at a receiving position
    skill = ReceiveBall("ReceiveBall_R0", target_pos=(-500, 400))

    # dribble backwards to a retreat position
    skill = DribbleBackwards("DribbleBack_R0", target_pos=(-300, 0))

    seq = SkillTestSeq(wm, dispatcher_q, robot_id=0, skill_node=skill)

Or use the convenience constructors at the bottom of this file.
"""

import py_trees

from TeamControl.network.robot_command import RobotCommand
from behaviour_tree.common_trees import GetWorldPositionUpdate, GetRobotIDPosition
from behaviour_tree.skill_nodes import MoveTo, KickAt, ReceiveBall, DribbleBackwards
from TeamControl.robot.constants import CRUISE_SPEED, DRIBBLE_SPEED, BALL_NEAR


class SkillTestSeq(py_trees.composites.Sequence):
    """
    Self-contained test harness for a single skill node.

    Mirrors TestTreeSeq from test_tree.py but accepts any skill_node
    instead of the hard-coded GoToBallSeq.

    The sequence always loops (memory=False on the parent keeps re-entering
    after SUCCESS/FAILURE so the robot keeps executing the skill).
    """

    def __init__(self, wm, dispatcher_q, robot_id: int, skill_node,
                 isYellow: bool = True, logger=None):
        color = "YELLOW" if isYellow else "BLUE"
        name = f"SkillTest R{robot_id} {color}"
        super().__init__(name, memory=True)

        self.wm = wm
        self.dispatcher_q = dispatcher_q
        self.robot_id = robot_id
        self.isYellow = isYellow
        self.skill_node = skill_node

        if logger is not None:
            self.logger = logger

        self.bb = py_trees.blackboard.Client(name=name)

    def setup(self, **kwargs):
        # Write robot identity onto the blackboard so SendRobotCommand can read it
        self.bb.register_key(key="robot_id",  access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="isYellow",  access=py_trees.common.Access.WRITE)
        self.bb.register_key(key="command",   access=py_trees.common.Access.WRITE)
        self.bb.robot_id  = self.robot_id
        self.bb.isYellow  = self.isYellow
        self.bb.command   = RobotCommand(self.robot_id, isYellow=self.isYellow)

        self.add_children([
            GetWorldPositionUpdate(self.wm, isYellow=self.isYellow),
            GetRobotIDPosition(robot_id=self.robot_id),
            self.skill_node,
            _SendCommand(self.dispatcher_q),
        ])

        for child in self.children:
            child.setup(getattr(self, "logger", None))

    def initialise(self):
        for child in self.children:
            child.setup(getattr(self, "logger", None))


# ─────────────────────────────────────────────────────────────────────────────
#  Convenience constructors — one per skill
# ─────────────────────────────────────────────────────────────────────────────

def make_move_to_test(wm, dispatcher_q, robot_id: int,
                      target_pos, isYellow: bool = True, **kwargs):
    """Test move_to: drive robot_id to target_pos."""
    skill = MoveTo(f"MoveTo_R{robot_id}", target_pos=target_pos, **kwargs)
    return SkillTestSeq(wm, dispatcher_q, robot_id, skill, isYellow)


def make_kick_at_test(wm, dispatcher_q, robot_id: int,
                      aim_pos, isYellow: bool = True):
    """Test kick_at: approach ball and kick toward aim_pos."""
    skill = KickAt(f"KickAt_R{robot_id}", aim_pos=aim_pos)
    return SkillTestSeq(wm, dispatcher_q, robot_id, skill, isYellow)


def make_receive_ball_test(wm, dispatcher_q, robot_id: int,
                           target_pos, isYellow: bool = True, **kwargs):
    """Test receive_ball: wait at target_pos and catch incoming ball."""
    skill = ReceiveBall(f"ReceiveBall_R{robot_id}", target_pos=target_pos, **kwargs)
    return SkillTestSeq(wm, dispatcher_q, robot_id, skill, isYellow)


def make_dribble_backwards_test(wm, dispatcher_q, robot_id: int,
                                target_pos, isYellow: bool = True, **kwargs):
    """Test dribble_backwards: reverse to target_pos with dribbler on."""
    skill = DribbleBackwards(f"DribbleBack_R{robot_id}", target_pos=target_pos, **kwargs)
    return SkillTestSeq(wm, dispatcher_q, robot_id, skill, isYellow)


# ─────────────────────────────────────────────────────────────────────────────
#  Local SendCommand — identical to the one in test_tree.py
#  (avoids importing from common_trees which has a slightly different signature)
# ─────────────────────────────────────────────────────────────────────────────

class _SendCommand(py_trees.behaviour.Behaviour):
    def __init__(self, dispatcher_q, runtime: int = 1):
        super().__init__("SendCommand")
        self.dispatcher_q = dispatcher_q
        self.runtime = runtime

    def setup(self, logger=None):
        if logger is not None:
            self.logger = logger
        self.bb = py_trees.blackboard.Client(name="SendCommand")
        for key in ("robot_id", "isYellow", "vx", "vy", "w", "kick", "dribble"):
            self.bb.register_key(key=key, access=py_trees.common.Access.READ)
        self.bb.register_key(key="command", access=py_trees.common.Access.WRITE)

    def initialise(self):
        robot_id  = self.bb.robot_id
        isYellow  = self.bb.isYellow
        vx        = self.bb.vx      if self.bb.exists("vx")      else 0.0
        vy        = self.bb.vy      if self.bb.exists("vy")      else 0.0
        w         = self.bb.w       if self.bb.exists("w")       else 0.0
        kick      = self.bb.kick    if self.bb.exists("kick")    else 0
        dribble   = self.bb.dribble if self.bb.exists("dribble") else 0
        self.bb.command = RobotCommand(robot_id=robot_id, vx=vx, vy=vy, w=w,
                                       kick=kick, dribble=dribble, isYellow=isYellow)

    def update(self) -> py_trees.common.Status:
        if not self.dispatcher_q.full():
            self.dispatcher_q.put((self.bb.command, self.runtime))
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
