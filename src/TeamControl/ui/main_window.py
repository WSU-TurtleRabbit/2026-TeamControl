"""
Main Window — clean tabbed layout, no docks.

Tabs:
  Dashboard       Field + robots + game state + network sidebar
  Behavior Tree   Interactive BT visualizer / editor
  Hardware Test   Manual robot control & testing console
  Settings        Simulation controls + Config editor + Network
  Console         Scrolling log viewer
"""

from PySide6.QtWidgets import (
    QMainWindow, QToolBar, QLabel, QComboBox, QPushButton,
    QTabWidget, QStatusBar, QWidget, QHBoxLayout, QSizePolicy,
    QApplication, QSpinBox,
)
from PySide6.QtCore import Qt, QTimer, QPointF
from PySide6.QtGui import QAction, QFont, QIcon

from TeamControl.ui.theme import QSS, ACCENT, TEXT, TEXT_DIM, SUCCESS, DANGER, WARNING
from TeamControl.ui.engine import SimEngine
from TeamControl.ui.field_canvas import FieldCanvas
from TeamControl.ui.dashboard_page import DashboardPage
from TeamControl.ui.test_panel import TestPanel
from TeamControl.ui.settings_page import SettingsPage
from TeamControl.ui.dispatcher_panel import DispatcherPanel
from TeamControl.ui.log_panel import LogPanel
from TeamControl.ui.calibration_page import CalibrationPage
from TeamControl.ui.onboard_possession_panel import OnboardPossessionPanel


class MainWindow(QMainWindow):
    """TurtleRabbit SSL Command Center."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("TurtleRabbit — SSL Command Center")
        self.setMinimumSize(1280, 800)
        self.resize(1800, 1050)
        self.setStyleSheet(QSS)

        # ── Engine ────────────────────────────────────────────────
        self._engine = SimEngine(self)

        # ── Shared field canvas ──────────────────────────────────
        self._field = FieldCanvas()

        # ── Pages ─────────────────────────────────────────────────
        self._test_panel = TestPanel(engine=self._engine, field=self._field)
        self._dispatch_panel = DispatcherPanel(engine=self._engine)
        self._calibration = CalibrationPage(
            engine=self._engine, test_panel=self._test_panel)
        self._dashboard = DashboardPage(
            self._field, engine=self._engine, test_panel=self._test_panel)
        self._settings = SettingsPage()
        self._log_panel = LogPanel()
        self._onboard_panel = OnboardPossessionPanel(engine=self._engine)

        # ── Central tabs ──────────────────────────────────────────
        self._tabs = QTabWidget()
        self._tabs.setObjectName("mainTabs")
        self._tabs.setDocumentMode(True)
        self._tabs.addTab(self._dashboard, "  Dashboard  ")
        self._tabs.addTab(self._settings, "  Settings  ")
        self._tabs.addTab(self._log_panel, "  Console  ")
        self._tabs.addTab(self._test_panel, "  Hardware Test  ")
        self._tabs.addTab(self._dispatch_panel, "  Dispatcher  ")
        self._tabs.addTab(self._calibration, "  Calibration  ")
        self._tabs.addTab(self._onboard_panel, "  Onboard Possession  ")
        self.setCentralWidget(self._tabs)

        # ── Toolbar ──────────────────────────────────────────────
        self._build_toolbar()

        # ── Menu ─────────────────────────────────────────────────
        self._build_menu()

        # ── Status bar ───────────────────────────────────────────
        self._status_mode = QLabel("Mode: —")
        self._status_mode.setStyleSheet(f"color:{TEXT_DIM}; padding:0 12px;")
        self._status_coords = QLabel("(—, —)")
        self._status_coords.setStyleSheet(f"color:{TEXT}; padding:0 12px;")
        self._status_fps = QLabel("0 fps")
        self._status_fps.setStyleSheet(f"color:{TEXT_DIM}; padding:0 12px;")

        sb = QStatusBar()
        sb.addWidget(self._status_mode)
        sb.addWidget(self._status_coords)
        sb.addPermanentWidget(self._status_fps)
        self.setStatusBar(sb)

        # Give calibration and test panel access to the "Our Bot" spinner
        self._calibration.set_our_bot_spin(self._our_id_spin)
        self._test_panel.set_our_bot_spin(self._our_id_spin)

        # ── Wire signals ─────────────────────────────────────────
        self._wire_signals()

        # Show cal card for the default selected mode right away
        self._on_mode_combo_changed(self._mode_combo.currentText())

        # ── Boot log ─────────────────────────────────────────────
        self._log_panel.append("[engine] TurtleRabbit Command Center ready")
        self._log_panel.append("[engine] Select a mode and click Start")

    # ── Toolbar ──────────────────────────────────────────────────

    def _build_toolbar(self):
        tb = QToolBar("Control")
        tb.setMovable(False)
        self.addToolBar(tb)

        logo = QLabel("  TurtleRabbit  ")
        logo.setFont(QFont("Segoe UI", 14, QFont.Bold))
        logo.setStyleSheet(f"color:{ACCENT};")
        tb.addWidget(logo)
        tb.addSeparator()

        tb.addWidget(QLabel("  Mode: "))
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(SimEngine.MODES)
        self._mode_combo.setMinimumWidth(130)
        self._mode_combo.currentTextChanged.connect(self._on_mode_combo_changed)
        tb.addWidget(self._mode_combo)
        tb.addSeparator()

        tb.addWidget(QLabel("  Our Bot: "))
        self._our_id_spin = QSpinBox()
        self._our_id_spin.setRange(0, 15)
        self._our_id_spin.setValue(0)
        self._our_id_spin.setToolTip("Shell ID of our robot (from ipconfig.yaml)")
        self._our_id_spin.setFixedWidth(55)
        tb.addWidget(self._our_id_spin)

        tb.addWidget(QLabel("  Opp Bot: "))
        self._opp_id_spin = QSpinBox()
        self._opp_id_spin.setRange(0, 15)
        self._opp_id_spin.setValue(0)
        self._opp_id_spin.setToolTip("Shell ID of opponent robot")
        self._opp_id_spin.setFixedWidth(55)
        tb.addWidget(self._opp_id_spin)
        tb.addSeparator()

        self._start_btn = QPushButton("  Start  ")
        self._start_btn.setObjectName("startBtn")
        self._start_btn.clicked.connect(self._on_start)
        tb.addWidget(self._start_btn)

        self._stop_btn = QPushButton("  Stop  ")
        self._stop_btn.setObjectName("stopBtn")
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._on_stop)
        tb.addWidget(self._stop_btn)
        tb.addSeparator()

        self._state_label = QLabel("  IDLE  ")
        self._state_label.setFont(QFont("Segoe UI", 12, QFont.Bold))
        self._state_label.setStyleSheet(f"color:{TEXT_DIM};")
        tb.addWidget(self._state_label)

        # Spacer
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        tb.addWidget(spacer)

        # View controls
        zoom_in = QPushButton("+")
        zoom_in.setFixedSize(28, 28)
        zoom_in.setToolTip("Zoom In")
        zoom_in.clicked.connect(lambda: self._zoom(1.2))
        zoom_out = QPushButton("−")
        zoom_out.setFixedSize(28, 28)
        zoom_out.setToolTip("Zoom Out")
        zoom_out.clicked.connect(lambda: self._zoom(0.8))
        zoom_reset = QPushButton("Fit")
        zoom_reset.setFixedSize(40, 28)
        zoom_reset.setToolTip("Reset field view")
        zoom_reset.clicked.connect(self._reset_field_view)

        tb.addWidget(QLabel(" View: "))
        tb.addWidget(zoom_in)
        tb.addWidget(zoom_out)
        tb.addWidget(zoom_reset)

    def _zoom(self, factor):
        self._field._scale = max(0.2, min(5.0, self._field._scale * factor))
        self._field.update()

    def _reset_field_view(self):
        self._field._scale = 1.0
        self._field._offset = QPointF(0, 0)
        self._field.update()

    # ── Menu ─────────────────────────────────────────────────────

    def _build_menu(self):
        mb = self.menuBar()

        file_menu = mb.addMenu("File")
        file_menu.addAction("Reload Config", self._settings.config_panel.load)
        file_menu.addSeparator()
        file_menu.addAction("Exit", self.close)

        view_menu = mb.addMenu("View")
        view_menu.addAction("Reset Field View", self._reset_field_view)
        view_menu.addSeparator()
        for i, name in enumerate(["Dashboard", "Settings", "Console",
                                   "Hardware Test", "Dispatcher",
                                   "Calibration", "Onboard Possession"]):
            view_menu.addAction(
                name, lambda checked=False, idx=i: self._tabs.setCurrentIndex(idx))

        sim_menu = mb.addMenu("Simulation")
        sim_menu.addAction("Center Ball",
                           lambda: self._engine.place_ball(0, 0))
        sim_menu.addAction("Kickoff Formation",
                           self._settings.sim_panel._kickoff_formation)

        mode_menu = mb.addMenu("Mode")
        for m in SimEngine.MODES:
            mode_menu.addAction(
                m.capitalize(),
                lambda checked=False, mode=m: self._switch_mode(mode))

        help_menu = mb.addMenu("Help")
        help_menu.addAction("About", self._show_about)

    def _show_about(self):
        from PySide6.QtWidgets import QMessageBox
        QMessageBox.about(
            self, "TurtleRabbit",
            "WSU TurtleRabbit SSL Command Center\n\n"
            "RoboCup Small Size League\n"
            "Team Control Dashboard v2.0")

    # ── Signal wiring ────────────────────────────────────────────

    def _wire_signals(self):
        eng = self._engine

        eng.frame_ready.connect(self._on_frame)
        eng.game_state_ready.connect(self._dashboard.update_game_state)
        eng.dispatch_info.connect(self._dispatch_panel.update_info)
        eng.engine_started.connect(self._on_engine_started)
        eng.engine_stopped.connect(self._on_engine_stopped)
        eng.log_message.connect(self._log_panel.append)

        self._dashboard.coordinate_hover.connect(self._on_coord_hover)

        sp = self._settings.sim_panel
        sp.place_ball_requested.connect(
            lambda x, y, vx, vy: eng.place_ball(x, y, vx, vy))
        sp.place_robot_requested.connect(
            lambda rid, yl, x, y, o: eng.place_robot(rid, yl, x, y, o))
        sp.field_place_ball.connect(
            lambda: self._field.set_place_mode("ball"))
        sp.field_place_robot.connect(
            lambda rid, yl: self._field.set_place_mode(("robot", rid, yl)))

        self._field.ball_placed.connect(
            lambda x, y: eng.place_ball(x, y))
        self._field.robot_placed.connect(
            lambda rid, yl, x, y: eng.place_robot(rid, yl, x, y))
        self._field.point_picked.connect(self._test_panel.go_to_point)
        self._field.action_requested.connect(self._test_panel.field_action)

        self._settings.config_panel.config_changed.connect(
            lambda: self._log_panel.append("[config] Configuration saved"))

    # ── Handlers ─────────────────────────────────────────────────

    def _on_mode_combo_changed(self, mode):
        self._dashboard.set_mode(mode)

    def _on_start(self):
        mode = self._mode_combo.currentText()
        our_id = self._our_id_spin.value()
        opp_id = self._opp_id_spin.value()
        self._log_panel.append(
            f"[engine] Starting {mode} — our bot #{our_id}, opp bot #{opp_id}")
        try:
            self._engine.start(mode, our_id=our_id, opp_id=opp_id)
        except Exception as e:
            self._log_panel.append(f"[error] Failed to start: {e}")

    def _on_stop(self):
        self._log_panel.append("[engine] Stopping…")
        self._engine.stop()

    def _switch_mode(self, mode):
        idx = SimEngine.MODES.index(mode)
        self._mode_combo.setCurrentIndex(idx)
        if self._engine.is_running:
            self._engine.stop()
        self._engine.start(mode,
                           our_id=self._our_id_spin.value(),
                           opp_id=self._opp_id_spin.value())

    def _on_engine_started(self, mode):
        self._start_btn.setEnabled(False)
        self._stop_btn.setEnabled(True)
        self._mode_combo.setEnabled(False)
        self._our_id_spin.setEnabled(False)
        self._opp_id_spin.setEnabled(False)
        self._state_label.setText(f"  RUNNING — {mode.upper()}  ")
        self._state_label.setStyleSheet(f"color:{SUCCESS}; font-weight:bold;")
        self._status_mode.setText(f"Mode: {mode}")
        self._dashboard.set_mode(mode)
        self._dashboard.set_engine_running(True)
        self._dispatch_panel.set_running(True)

    def _on_engine_stopped(self):
        self._start_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        self._mode_combo.setEnabled(True)
        self._our_id_spin.setEnabled(True)
        self._opp_id_spin.setEnabled(True)
        self._state_label.setText("  IDLE  ")
        self._state_label.setStyleSheet(f"color:{TEXT_DIM};")
        self._status_mode.setText("Mode: —")
        self._dashboard.set_engine_running(False)
        self._dispatch_panel.set_running(False)

    def _on_frame(self, snap):
        self._dashboard.update_frame(snap)
        fps = self._dashboard.get_fps()
        self._status_fps.setText(f"{fps} fps")

    def _on_coord_hover(self, x, y):
        self._status_coords.setText(f"({x:.0f}, {y:.0f}) mm")

    # ── Cleanup ──────────────────────────────────────────────────

    def closeEvent(self, event):
        if self._engine.is_running:
            self._engine.stop()
        event.accept()
