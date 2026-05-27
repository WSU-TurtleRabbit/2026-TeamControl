import json
import os
import tempfile

_GAINS_PATH = os.path.join(os.path.dirname(__file__), "pd_gains.json")

_DEFAULT_GAINS = {
    "linear":  {"kp": 1.0, "kd": 1.0},
    "turning": {"kp": 1.0, "kd": 1.0},
}


def load_gains(robot_id: str) -> dict:
    """
    Return gains for `robot_id` from pd_gains.json.
    - Falls back to defaults if the robot is missing or the file is unreadable.
    """
    try:
        with open(_GAINS_PATH, "r") as f:
            data = json.load(f)
        return data.get(robot_id, _DEFAULT_GAINS)
    except (FileNotFoundError, json.JSONDecodeError):
        return _DEFAULT_GAINS


def save_gains(robot_id: str, linear: dict, turning: dict) -> None:
    """
    Write updated gains for `robot_id` back to pd_gains.json.
    - Writes to a temp file first then renames — prevents corruption on crash.
    """
    try:
        with open(_GAINS_PATH, "r") as f:
            data = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        data = {}

    data[robot_id] = {"linear": linear, "turning": turning}

    dir_name = os.path.dirname(_GAINS_PATH)
    with tempfile.NamedTemporaryFile("w", dir=dir_name, delete=False, suffix=".tmp") as tmp:
        json.dump(data, tmp, indent=2)
        tmp_path = tmp.name

    os.replace(tmp_path, _GAINS_PATH)
