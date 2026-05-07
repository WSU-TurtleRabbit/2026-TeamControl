
import math
import numpy as np

from TeamControl.network.robot_command import RobotCommand
from TeamControl.world.model import WorldModel as wm
from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.Movement import RobotMovement, get_movement


def go_to_ball_and_shoot(world_model: wm, isYellow: bool, robot_id: int,
                         shoot_right_goal: bool = True):
    # 1. Get ball and robot pose from the world model
    ball = world_model.get_ball()
    robot_pos = world_model.get_robot(isYellow=isYellow, robot_id=robot_id)

    if ball is None or robot_pos is None:
        # No info → send stop command
        return RobotCommand(robot_id=robot_id)

    ball_pos = (ball.x, ball.y)

    # 2. Work out goal position from field geometry
    geom = world_model.geometry_data
    field = geom.field_size

    # right goal is +x, left goal is -x – adjust based on your team side
    goal_x = field.field_length / 2.0 if shoot_right_goal else -field.field_length / 2.0
    goal_y = 0.0
    goal_pos = (goal_x, goal_y)

    # 3. Position where robot should stand to shoot
    shooting_pos = RobotMovement.shooting_pos(ball_pos, goal_pos, robot_offset=200.0)

    # 4. Compute velocity to go to that shooting position and face the goal
    vx, vy, w = get_movement(robot_id, is_yellow=isYellow).velocity_to_target(
        robot_pos=robot_pos,          # (x, y, theta) in world coords
        target=shooting_pos,          # where to stand
        turning_target=goal_pos,      # what to face
        stop_threshold=150.0
    )

    # 5. Decide if we should kick now
    #    Condition: close to ball AND roughly aligned to goal direction
    ball_rel = world2robot(robot_position=robot_pos, target_position=ball_pos)
    dist_to_ball = math.hypot(ball_rel[0], ball_rel[1])

    # Direction to goal in robot frame
    goal_rel = world2robot(robot_position=robot_pos, target_position=goal_pos)
    angle_to_goal = math.atan2(goal_rel[1], goal_rel[0])

    close_enough = dist_to_ball < 120.0        # mm threshold – tune this
    facing_goal = abs(angle_to_goal) < 0.15    # ~8.5 degrees – tune this

    kick = 1 if (close_enough and facing_goal) else 0

    return RobotCommand(
        robot_id=robot_id,
        vx=vx,
        vy=vy,
        w=w,
        kick=kick,
        dribble=0
    )
