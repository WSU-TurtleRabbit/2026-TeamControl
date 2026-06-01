"""
Backend engine — bridges the multiprocessing system with the Qt UI.

Manages process lifecycle, polls the WorldModel, and emits Qt signals
that drive every widget in the dashboard.
"""

import time
import math
from multiprocessing import Process, Queue, Event

from PySide6.QtCore import QObject, QTimer, Signal

from TeamControl.process_workers.vision_runner import VisionProcess
from TeamControl.process_workers.gcfsm_runner import GCfsm
from TeamControl.process_workers.wm_runner import WMWorker
from TeamControl.process_workers.robot_recv_runner import RobotRecv
from TeamControl.world.model_manager import WorldModelManager
from TeamControl.dispatcher.dispatch import Dispatcher
from TeamControl.utils.yaml_config import Config

from TeamControl.robot.goalie import run_goalie
from TeamControl.robot.striker import run_striker
from TeamControl.robot.navigator import run_navigator, WAYPOINTS_A, WAYPOINTS_B
from TeamControl.robot.team import run_team
from TeamControl.robot.coop import run_coop

from TeamControl.network.ssl_sockets import grSimSender
from TeamControl.network.grSimPacketFactory import grSimPacketFactory
from TeamControl.onboard_vision import (
    OnboardObservationStore, build_ip_map,
)


# ── Snapshot dataclasses ─────────────────────────────────────────────

class RobotSnapshot:
    __slots__ = ("id", "team", "x", "y", "o", "confidence")

    def __init__(self, rid, team, x, y, o, conf):
        self.id = rid
        self.team = team
        self.x = x
        self.y = y
        self.o = o
        self.confidence = conf


class BallSnapshot:
    __slots__ = ("x", "y")

    def __init__(self, x, y):
        self.x = x
        self.y = y


class FrameSnapshot:
    __slots__ = ("yellow", "blue", "ball", "frame_number")

    def __init__(self):
        self.yellow: list[RobotSnapshot] = []
        self.blue: list[RobotSnapshot] = []
        self.ball: BallSnapshot | None = None
        self.frame_number: int = 0


# ── Engine ───────────────────────────────────────────────────────────

class SimEngine(QObject):
    """Manages the multiprocessing backend and emits signals for UI."""

    frame_ready = Signal(object)         # FrameSnapshot
    game_state_ready = Signal(object)    # str | None
    dispatch_info = Signal(object)       # dict snapshot from dispatcher
    engine_started = Signal(str)         # mode name
    engine_stopped = Signal()
    log_message = Signal(str)            # log line
    onboard_packet = Signal(object, object)  # (OnboardObservation, addr)

    MODES = ["vision_only", "goalie", "1v1", "obstacle", "coop", "6v6"]

    def __init__(self, parent=None):
        super().__init__(parent)
        self._config: Config | None = None
        self._wm_manager: WorldModelManager | None = None
        self._wm = None
        self._is_running: Event | None = None
        self._bg_procs: list[Process] = []
        self._fg_procs: list[Process] = []
        self._vision_q: Queue | None = None
        self._gc_q: Queue | None = None
        self._dispatch_q: Queue | None = None
        self._dispatch_info_q: Queue | None = None
        self._recv_q: Queue | None = None
        self._grsim_sender: grSimSender | None = None
        self._field_manual_q: Queue | None = None

        self._running = False
        self._mode = ""
        self._last_version = -1

        self._onboard_store = OnboardObservationStore()
        self._ip_to_robot: dict[str, tuple[bool, int]] = {}
        self._onboard_last_ts: dict[tuple[bool, int], float] = {}

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(16)  # ~60 fps
        self._poll_timer.timeout.connect(self._poll)

    # ── Properties ────────────────────────────────────────────────

    @property
    def is_running(self):
        return self._running

    @property
    def current_mode(self):
        return self._mode

    @property
    def config(self) -> Config | None:
        return self._config

    @property
    def onboard_store(self) -> OnboardObservationStore:
        return self._onboard_store

    @property
    def ip_to_robot(self) -> dict:
        return self._ip_to_robot

    def set_field_manual_control(self, shell_id: int, is_yellow: bool, enabled: bool):
        """
        While enabled, the dispatcher stops sending to this robot so Dashboard /
        Hardware Test field commands are not overwritten by AI (goalie, 6v6, …).
        """
        if not self._running or self._field_manual_q is None:
            return
        try:
            self._field_manual_q.put_nowait(
                ("on" if enabled else "off", int(shell_id), bool(is_yellow)))
        except Exception:
            pass

    # ── Lifecycle ─────────────────────────────────────────────────

    def reload_config(self):
        self._config = Config()
        return self._config

    def start(self, mode: str = "goalie", our_id: int = 0, opp_id: int = 0):
        if self._running:
            self.stop()

        self._config = Config()
        preset = self._config
        self._ip_to_robot = build_ip_map(preset)

        self._vision_q = Queue()
        self._gc_q = Queue()
        self._dispatch_q = Queue()
        self._dispatch_info_q = Queue()
        self._recv_q = Queue()
        self._field_manual_q = Queue()
        self._is_running = Event()

        self._wm_manager = WorldModelManager()
        self._wm_manager.start()
        self._wm = self._wm_manager.WorldModel()

        self._bg_procs = [
            Process(target=VisionProcess.run_worker,
                    args=(self._is_running, None, self._vision_q,
                          preset.use_grSim_vision, preset.vision[1]),
                    daemon=True),
            Process(target=GCfsm.run_worker,
                    args=(self._is_running, None, self._gc_q,
                          preset.us_yellow, preset.us_positive, preset.team_name),
                    daemon=True),
            Process(target=WMWorker.run_worker,
                    args=(self._is_running, None, self._wm,
                          self._vision_q, self._gc_q,
                          self._recv_q, dict(self._ip_to_robot)),
                    daemon=True),
            Process(target=Dispatcher.run_worker,
                    args=(self._is_running, None, self._dispatch_q, preset,
                          self._dispatch_info_q, self._field_manual_q),
                    daemon=True),
            Process(target=RobotRecv.run_worker,
                    args=(self._is_running, None, self._recv_q),
                    daemon=True),
        ]

        self._fg_procs = self._build_foreground(mode, preset, our_id, opp_id)

        self._is_running.set()
        for p in self._bg_procs:
            p.start()
        for p in self._fg_procs:
            p.start()

        try:
            self._grsim_sender = grSimSender(*preset.grSim_addr)
        except Exception:
            self._grsim_sender = None

        self._running = True
        self._mode = mode
        self._last_version = -1
        self._poll_timer.start()

        self.engine_started.emit(mode)
        self.log_message.emit(f"[engine] Started mode: {mode}")

    def stop(self):
        if not self._running:
            return
        self._poll_timer.stop()
        if self._is_running:
            self._is_running.clear()

        for p in self._fg_procs:
            p.join(timeout=3)
            if p.is_alive():
                p.terminate()
        for p in self._bg_procs:
            p.join(timeout=3)
            if p.is_alive():
                p.terminate()

        self._fg_procs.clear()
        self._bg_procs.clear()

        if self._wm_manager:
            try:
                self._wm_manager.shutdown()
            except Exception:
                pass

        self._wm = None
        self._wm_manager = None
        self._recv_q = None
        self._dispatch_info_q = None
        self._field_manual_q = None
        self._running = False
        self._mode = ""
        self._grsim_sender = None

        self.engine_stopped.emit()
        self.log_message.emit("[engine] Stopped")

    # ── Foreground builder ────────────────────────────────────────

    def _build_foreground(self, mode, preset, our_id=0, opp_id=0):
        procs = []
        wm = self._wm
        dq = self._dispatch_q
        ev = self._is_running

        self.log_message.emit(
            f"[engine] Building {mode}: our shell={our_id} "
            f"({'yellow' if preset.us_yellow else 'blue'}), "
            f"opp shell={opp_id} "
            f"({'blue' if preset.us_yellow else 'yellow'})")

        if mode == "vision_only":
            self.log_message.emit(
                "[engine] Vision-only mode — no robot models running")
            return procs

        if mode == "goalie":
            procs.append(Process(target=run_goalie,
                                 args=(ev, dq, wm, our_id, preset.us_yellow),
                                 daemon=True))
            procs.append(Process(target=run_striker,
                                 args=(ev, dq, wm, opp_id, not preset.us_yellow),
                                 daemon=True))
        elif mode == "1v1":
            procs.append(Process(target=run_striker,
                                 args=(ev, dq, wm, our_id, True),
                                 daemon=True))
            procs.append(Process(target=run_striker,
                                 args=(ev, dq, wm, opp_id, False),
                                 daemon=True))
        elif mode == "obstacle":
            procs.append(Process(target=run_navigator,
                                 args=(ev, dq, wm, our_id,
                                       preset.us_yellow, WAYPOINTS_A),
                                 daemon=True))
            procs.append(Process(target=run_navigator,
                                 args=(ev, dq, wm, opp_id,
                                       preset.us_yellow, WAYPOINTS_B),
                                 daemon=True))
        elif mode == "coop":
            # Cross-team coop: our bot + opp bot cooperate left → right
            us_y = preset.us_yellow
            opp_y = not us_y
            procs.append(Process(target=run_coop,
                                 args=(ev, dq, wm, our_id, opp_id, us_y),
                                 kwargs=dict(mate_is_yellow=opp_y,
                                             attack_positive=True),
                                 daemon=True))
            procs.append(Process(target=run_coop,
                                 args=(ev, dq, wm, opp_id, our_id, opp_y),
                                 kwargs=dict(mate_is_yellow=us_y,
                                             attack_positive=True),
                                 daemon=True))
        elif mode == "6v6":
            procs.append(Process(target=run_team,
                                 args=(ev, dq, wm, True, our_id),
                                 daemon=True))
            procs.append(Process(target=run_team,
                                 args=(ev, dq, wm, False, opp_id),
                                 daemon=True))
        return procs

    # ── Polling ───────────────────────────────────────────────────

    def _poll(self):
        if not self._wm:
            return
        try:
            frame = self._wm.get_latest_frame()
            if frame is None:
                return
            snap = self._extract_snapshot(frame)
            self.frame_ready.emit(snap)

            ver = self._wm.get_version()
            if ver != self._last_version:
                self._last_version = ver
                gs = self._wm.get_game_state()
                self.game_state_ready.emit(gs)
        except Exception as exc:
            self.log_message.emit(f"[engine] poll error: {exc}")

        self._sync_onboard_from_wm()
        self._drain_dispatch_info()

    def _drain_dispatch_info(self):
        if self._dispatch_info_q is None:
            return
        latest = None
        try:
            while True:
                latest = self._dispatch_info_q.get_nowait()
        except Exception:
            pass
        if latest is not None:
            self.dispatch_info.emit(latest)

    def _sync_onboard_from_wm(self):
        """Mirror new onboard observations from the shared WM into the
        UI-local store and emit signals. The recv_q is drained by WMWorker
        (the sole consumer); the WM is the source of truth."""
        if not self._wm:
            return
        try:
            snapshot = self._wm.onboard_snapshot()
        except Exception:
            return
        for key, obs in snapshot.items():
            ts = getattr(obs, "recv_ts", 0.0)
            if ts <= self._onboard_last_ts.get(key, 0.0):
                continue
            self._onboard_last_ts[key] = ts
            self._onboard_store.put(obs)
            self.onboard_packet.emit(obs, None)

    def _extract_snapshot(self, frame) -> FrameSnapshot:
        snap = FrameSnapshot()
        snap.frame_number = frame.frame_number

        if frame.robots_yellow:
            for r in frame.robots_yellow:
                snap.yellow.append(RobotSnapshot(
                    r.id, "yellow", r.x, r.y, r.o, r.confidence))

        if frame.robots_blue:
            for r in frame.robots_blue:
                snap.blue.append(RobotSnapshot(
                    r.id, "blue", r.x, r.y, r.o, r.confidence))

        if frame.ball:
            snap.ball = BallSnapshot(frame.ball.x, frame.ball.y)

        return snap

    # ── Simulation controls ───────────────────────────────────────

    def place_ball(self, x_mm, y_mm, vx=0.0, vy=0.0):
        if not self._grsim_sender:
            return
        try:
            pkt = grSimPacketFactory.ball_replacement_command(
                x=x_mm / 1000.0, y=y_mm / 1000.0,
                vx=vx / 1000.0, vy=vy / 1000.0)
            self._grsim_sender.send_packet(pkt)
            self.log_message.emit(f"[sim] Ball placed at ({x_mm:.0f}, {y_mm:.0f})")
        except Exception as e:
            self.log_message.emit(f"[sim] Ball placement failed: {e}")

    def place_robot(self, robot_id, is_yellow, x_mm, y_mm, orientation=0.0):
        if not self._grsim_sender:
            return
        try:
            pkt = grSimPacketFactory.robot_replacement_command(
                x=x_mm / 1000.0, y=y_mm / 1000.0,
                orientation=orientation,
                robot_id=robot_id, isYellow=is_yellow)
            self._grsim_sender.send_packet(pkt)
            team = "Yellow" if is_yellow else "Blue"
            self.log_message.emit(
                f"[sim] {team} #{robot_id} placed at ({x_mm:.0f}, {y_mm:.0f})")
        except Exception as e:
            self.log_message.emit(f"[sim] Robot placement failed: {e}")
