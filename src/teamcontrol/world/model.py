"""
World Model — the single source of truth.

Everything the system knows about the current state of the game
lives here. Vision data, game controller state, robot positions,
ball tracking — all of it flows through the world model.

This gets wrapped in a multiprocessing Manager so multiple
processes can read/write to it safely. See manager.py.

@author Emma (original), TRP Team (v2 rewrite)
"""

from multiprocessing import Manager
from teamcontrol.core.cache import Cache
import logging

log = logging.getLogger(__name__)


class WorldModel:
    """
    Central game state. Shared across all processes via the manager proxy.
    """

    def __init__(self, us_yellow=True, us_positive=True, max_frames=120):
        self._us_yellow = us_yellow
        self._us_positive = us_positive
        self._robots_active = 6

        # frame storage — just a rolling list for now
        self._frames = []
        self._max_frames = max_frames

        # game controller state
        self._game_state = None
        self._ball_left_field = None

        # geometry gets set once vision sends it
        self._geometry = None
        self._field = None

        # version counter — bumped periodically so watchers know
        # something changed without having to diff frames
        mgr = Manager()
        self._version = mgr.Value("i", 0)
        self._frame_count = 0
        self._update_every = 10  # bump version every N frames

        # per-model cache for computed stuff
        self._cache = Cache(ttl=2.0, max_size=256)

    # ── frames ──────────────────────────────────────────────────

    def add_frame(self, frame):
        """Add a new vision frame."""
        self._frames.append(frame)
        if len(self._frames) > self._max_frames:
            self._frames = self._frames[-self._max_frames:]

        self._frame_count += 1
        if self._frame_count >= self._update_every:
            self._version.value += 1
            self._frame_count = 0

        # invalidate cached stuff that depends on frame data
        self._cache.invalidate_ns("computed")

    @property
    def latest_frame(self):
        if not self._frames:
            return None
        return self._frames[-1]

    def get_last_n_frames(self, n: int):
        return self._frames[-n:]

    @property
    def version(self):
        return self._version.value

    # ── team info ───────────────────────────────────────────────

    @property
    def us_yellow(self):
        return self._us_yellow

    @property
    def us_positive(self):
        return self._us_positive

    def switch_team(self, us_yellow: bool, us_positive: bool):
        self._us_yellow = us_yellow
        self._us_positive = us_positive
        self._cache.clear()  # team changed, everything is stale
        log.info(f"team switched: yellow={us_yellow}, positive={us_positive}")

    @property
    def robots_active(self):
        return self._robots_active

    @robots_active.setter
    def robots_active(self, count):
        self._robots_active = count

    # ── game controller ─────────────────────────────────────────

    @property
    def game_state(self):
        return self._game_state

    @game_state.setter
    def game_state(self, state):
        self._game_state = state

    @property
    def ball_left_field(self):
        return self._ball_left_field

    @ball_left_field.setter
    def ball_left_field(self, location):
        self._ball_left_field = location

    # ── geometry ────────────────────────────────────────────────

    @property
    def geometry(self):
        return self._geometry

    def set_geometry(self, geo):
        self._geometry = geo
        self._field = geo.field if hasattr(geo, "field") else None

    @property
    def field(self):
        return self._field

    # ── cache access ────────────────────────────────────────────

    @property
    def cache(self) -> Cache:
        """Direct access to the model's cache for computed values."""
        return self._cache

    def __repr__(self):
        n_frames = len(self._frames)
        return f"WorldModel(v={self.version}, frames={n_frames}, yellow={self._us_yellow})"
