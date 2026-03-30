"""
Config loader for TRP 2026.
Loads defaults.yaml first, then overlays config.yaml on top.
"""

from pathlib import Path
import yaml

try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader


_HERE = Path(__file__).resolve().parent
_DEFAULTS = _HERE / "defaults.yaml"


def _deep_merge(base: dict, overlay: dict) -> dict:
    """Merge overlay into base, recursively for nested dicts."""
    merged = base.copy()
    for key, val in overlay.items():
        if key in merged and isinstance(merged[key], dict) and isinstance(val, dict):
            merged[key] = _deep_merge(merged[key], val)
        else:
            merged[key] = val
    return merged


class Config:
    """
    Loads config from yaml files.
    Defaults come from defaults.yaml in this directory.
    If you pass a config_path, those values override the defaults.
    """

    def __init__(self, config_path: str = None):
        # always load defaults first
        with open(_DEFAULTS, "r") as f:
            self._data = yaml.load(f, Loader)

        # overlay user config if one exists
        if config_path is not None:
            user_path = Path(config_path)
            if user_path.exists():
                with open(user_path, "r") as f:
                    user_data = yaml.load(f, Loader)
                if user_data:
                    self._data = _deep_merge(self._data, user_data)

        # pull out the commonly accessed stuff so you dont have to
        # dig through nested dicts every time
        self._unpack()

    def _unpack(self):
        team = self._data["team"]
        self.us_yellow = team["us_yellow"]
        self.us_positive = team["us_positive"]
        self.robots_active = team["robots_active"]

        self.grsim_addr = (self._data["grSim"]["ip"], self._data["grSim"]["port"])

        vis = self._data["vision"]
        self.vision_addr = (vis["multicast_group"], vis["port"])
        self.use_grsim_vision = vis["use_grsim"]
        self.max_cameras = vis["max_cameras"]

        gc = self._data["game_controller"]
        self.gc_addr = (gc["multicast_group"], gc["port"])

        self.send_to_grsim = self._data["send_to_grsim"]

        net = self._data.get("network", {})
        self.robot_ip = net.get("robot_ip", "192.168.1.2")

        cache = self._data.get("cache", {})
        self.cache_max_frames = cache.get("max_frames", 120)
        self.cache_ttl = cache.get("ttl_seconds", 5.0)
        self.cache_cleanup_interval = cache.get("cleanup_interval", 10.0)

    def get(self, key, default=None):
        """Get a raw value from the config dict."""
        return self._data.get(key, default)

    def __repr__(self):
        return f"Config(yellow={self.us_yellow}, grsim={self.grsim_addr})"


if __name__ == "__main__":
    cfg = Config()
    print(cfg)
    print(f"  vision: {cfg.vision_addr}")
    print(f"  gc: {cfg.gc_addr}")
