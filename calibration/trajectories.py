import time
from typing import Callable, List

from src.TeamControl.robot.Movement import Intent, RobotState, RobotMovement

TICK_DT   = 0.02   # 20ms — 50Hz control loop
TIMEOUT   = 5.0    # seconds before giving up


def _run(controller: RobotMovement,
         intent: Intent,
         get_state: Callable[[], RobotState],
         send_command: Callable[[float, float, float], None],
         timeout: float = TIMEOUT) -> List[dict]:
    """
    Core loop shared by all three tests.
    - Ticks at 50Hz, logs each tick, stops on arrival or timeout.
    - Returns list of log entries: {t, state, error_xy, error_theta, vx, vy, w}
    """
    log = []
    controller.reset()
    start = time.monotonic()

    while True:
        now = time.monotonic()
        elapsed = now - start

        if elapsed > timeout:
            break

        state = get_state()
        target_xy = (intent.target[0], intent.target[1])
        error_xy = (target_xy[0] - state.x, target_xy[1] - state.y)
        error_theta = intent.target[2] - state.theta

        result = controller.step(intent, state)

        if result is None:
            # Arrived — log final state then stop.
            log.append({"t": elapsed, "state": state,
                        "error_xy": error_xy, "error_theta": error_theta,
                        "vx": 0.0, "vy": 0.0, "w": 0.0})
            break

        vx, vy, w = result
        send_command(vx, vy, w)
        log.append({"t": elapsed, "state": state,
                    "error_xy": error_xy, "error_theta": error_theta,
                    "vx": vx, "vy": vy, "w": w})

        # Sleep for remainder of tick.
        elapsed_after = time.monotonic() - start
        sleep_time = (elapsed // TICK_DT + 1) * TICK_DT - elapsed_after
        if sleep_time > 0:
            time.sleep(sleep_time)

    send_command(0.0, 0.0, 0.0)
    return log


def translation_test(controller: RobotMovement,
                     get_state: Callable[[], RobotState],
                     send_command: Callable[[float, float, float], None],
                     distance_mm: float = 500.0) -> List[dict]:
    """
    Drive straight ahead by `distance_mm`, no rotation.
    - Tests linear PD gains in isolation.
    """
    state = get_state()
    intent = Intent(type="move_to",
                    target=(state.x + distance_mm, state.y, state.theta))
    return _run(controller, intent, get_state, send_command)


def rotational_test(controller: RobotMovement,
                    get_state: Callable[[], RobotState],
                    send_command: Callable[[float, float, float], None],
                    delta_theta: float = 1.5708) -> List[dict]:
    """
    Rotate in place by `delta_theta` radians (default 90°), no translation.
    - Tests angular PD gains in isolation.
    """
    import math
    state = get_state()
    target_theta = state.theta + delta_theta
    intent = Intent(type="move_to",
                    target=(state.x, state.y, target_theta))
    return _run(controller, intent, get_state, send_command)


def general_test(controller: RobotMovement,
                 get_state: Callable[[], RobotState],
                 send_command: Callable[[float, float, float], None],
                 distance_mm: float = 500.0,
                 delta_theta: float = 1.5708) -> List[dict]:
    """
    Move forward and rotate simultaneously.
    - Tests linear and angular PD interaction.
    """
    state = get_state()
    intent = Intent(type="move_to",
                    target=(state.x + distance_mm, state.y, state.theta + delta_theta))
    return _run(controller, intent, get_state, send_command)
