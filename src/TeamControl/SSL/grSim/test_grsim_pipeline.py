"""grSim pipeline test — drives one robot directly through the new pipeline.

No BT coordinator involved. Edit the TEST CONFIG section below to change
what you're testing, then run:

    python src/TeamControl/SSL/grSim/test_grsim_pipeline.py

Controls:
    Ctrl+C to stop.
"""
import sys
import time
from multiprocessing import Event, Process, Queue, freeze_support

from TeamControl.bt.actuator import ActuatorMode
from TeamControl.bt.contracts.geometry import Point2D, Pose2D, Velocity2D
from TeamControl.bt.contracts.skill_intent import PDMode, SkillIntent
from TeamControl.bt.skill_intent_executor import execute
from TeamControl.dispatcher.dispatch import Dispatcher
from TeamControl.network.robot_command import RobotCommand
from TeamControl.process_workers.vision_runner import VisionProcess
from TeamControl.process_workers.wm_runner import WMWorker
from TeamControl.utils.yaml_config import Config
from TeamControl.world.model_manager import WorldModelManager

# ---------------------------------------------------------------------------
# TEST CONFIG — edit this to change what you're testing
# ---------------------------------------------------------------------------

ROBOT_ID   = 1          # which robot to control (0 = goalie, 1 = attacker)
IS_YELLOW  = True       # match your ipconfig.yaml us_yellow

# SkillIntent to test:
#   STOP, MOVE_TO_POINT, GO_TO_BALL, GO_TO_BALL_AND_KICK,
#   FACE_TARGET, DRIBBLE_TO_POINT
SKILL = SkillIntent.MOVE_TO_POINT

# Target position and heading (metres, radians)
TARGET = Pose2D(x=1.0, y=0.0, theta=0.0)

TICK_HZ  = 100           # control loop rate
CONFIG   = "ipconfig.yaml"

# ---------------------------------------------------------------------------


def _run_test(is_running: Event, wm, dispatcher_q: Queue) -> None:
    tick_period = 1.0 / TICK_HZ
    state = None
    tick = 0

    print(f"[TEST] robot={ROBOT_ID} yellow={IS_YELLOW} skill={SKILL.name} target={TARGET}")

    while is_running.is_set():
        frame = wm.get_latest_frame()
        if frame is None:
            time.sleep(tick_period)
            continue

        # Read robot pose
        team = frame.robots_yellow if IS_YELLOW else frame.robots_blue
        robot = next((r for r in team if int(r.id) == ROBOT_ID), None)
        if robot is None:
            time.sleep(tick_period)
            continue

        current_pose = Pose2D(
            x=float(robot.x) / 1000.0,
            y=float(robot.y) / 1000.0,
            theta=float(robot.o),
        )

        # Read ball
        ball = frame.ball
        ball_pos  = Point2D(float(ball.x) / 1000.0, float(ball.y) / 1000.0) if ball else None
        ball_vel  = Velocity2D(0.0, 0.0)

        cmd, state = execute(
            skill_intent=SKILL,
            robot_id=ROBOT_ID,
            is_yellow=IS_YELLOW,
            current_pose=current_pose,
            target_pose=TARGET,
            ball_pos=ball_pos,
            ball_vel=ball_vel,
            state=state,
        )

        if not dispatcher_q.full():
            dispatcher_q.put([cmd, 1.0])

        # Print every second
        if tick % TICK_HZ == 0:
            dist = state.pd_output_dist if hasattr(state, "pd_output_dist") else "?"
            print(
                f"[TEST] tick={tick} "
                f"pos=({current_pose.x:.2f},{current_pose.y:.2f}) "
                f"heading={current_pose.theta:.2f}rad "
                f"reached={state.robot_reached_target} "
                f"moving={state.is_moving} dribble={state.is_dribbling} kick={state.is_kicking} "
                f"vx={cmd.vx:.2f} vy={cmd.vy:.2f} w={cmd.w:.2f} "
                f"kick_cmd={cmd.kick} dribble_cmd={cmd.dribble}",
                flush=True,
            )

        tick += 1
        time.sleep(tick_period)


def main():
    freeze_support()

    config_file = sys.argv[1] if len(sys.argv) > 1 else CONFIG
    preset = Config(config_file)

    is_running = Event()
    is_running.set()

    vision_q     = Queue()
    gc_q         = Queue()   # unused but WMWorker requires a real Queue
    dispatcher_q = Queue()

    vision_wkr = Process(
        target=VisionProcess.run_worker,
        args=(is_running, None, vision_q, True, 10006),
    )

    wm_manager = WorldModelManager()
    wm_manager.start()
    wm = wm_manager.WorldModel()

    wm_wkr = Process(
        target=WMWorker.run_worker,
        args=(is_running, None, wm, vision_q, gc_q),
    )

    dispatcher = Process(
        target=Dispatcher.run_worker,
        args=(is_running, None, dispatcher_q, preset),
    )

    vision_wkr.start()
    wm_wkr.start()
    dispatcher.start()

    print("[TEST] waiting for vision...", flush=True)
    time.sleep(1.5)

    try:
        _run_test(is_running, wm, dispatcher_q)
    except KeyboardInterrupt:
        print("\n[TEST] stopping...", flush=True)
    finally:
        is_running.clear()

    for p in [vision_wkr, wm_wkr, dispatcher]:
        p.terminate()
        p.join()


if __name__ == "__main__":
    main()
