"""Command Composer — Task 4.

Packages PDOutput + ActuatorOutput + robot identity into a final RobotCommand.
Does not contain PD math, actuator zone rules, or SkillIntent logic.
"""
from __future__ import annotations

from dataclasses import dataclass

from TeamControl.bt.actuator import ActuatorOutput
from TeamControl.bt.contracts.pd_output import PDOutput
from TeamControl.network.robot_command import RobotCommand


@dataclass(frozen=True)
class CommandComposerInput:
    robot_id: int
    is_yellow: bool
    pd_output: PDOutput
    actuator_output: ActuatorOutput


def compose_command(inp: CommandComposerInput) -> RobotCommand:
    """Build a RobotCommand from PD and Actuator outputs."""
    return RobotCommand(
        robot_id=inp.robot_id,
        isYellow=inp.is_yellow,
        vx=inp.pd_output.vx,
        vy=inp.pd_output.vy,
        w=inp.pd_output.w,
        kick=int(inp.actuator_output.kick),
        dribble=int(inp.actuator_output.dribble),
    )
