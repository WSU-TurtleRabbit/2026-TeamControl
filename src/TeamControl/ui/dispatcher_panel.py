"""
Dispatcher Inspector — live view of the command dispatch pipeline.

Shows:
  - Running commands table (robot ID, team, vx/vy/w, kick, dribble, elapsed, IP, sends)
  - Shell ID → IP/port mapping from config
  - Dispatcher config (grSim target, queue depth)
  - Per-robot send counters
"""

import time
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSplitter,
    QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
    QGroupBox, QGridLayout, QFrame, QPlainTextEdit,
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QFont

from TeamControl.ui.theme import (
    ACCENT, TEXT_DIM, SUCCESS, DANGER, WARNING, BORDER,
    YELLOW_TEAM, BLUE_TEAM, BG_DARK, BG_CARD, BG_MID, TEXT,
)


def _heading(text):
    lbl = QLabel(text)
    lbl.setStyleSheet(f"font-size:14px; font-weight:bold; color:{ACCENT}; padding:2px 0;")
    return lbl


def _card(title_text):
    card = QFrame()
    card.setObjectName("card")
    lay = QVBoxLayout(card)
    lay.setContentsMargins(12, 10, 12, 10)
    lay.setSpacing(6)
    title = QLabel(title_text)
    title.setStyleSheet(f"font-size:13px; font-weight:bold; color:{ACCENT}; padding:0;")
    lay.addWidget(title)
    return card, lay


class DispatcherPanel(QWidget):
    """Live dispatcher state inspector."""

    CMD_COLS = [
        "ID", "Team", "VX", "VY", "W", "Kick", "Drib",
        "Elapsed", "Runtime", "Target IP", "Port", "Sends",
    ]

    SHELL_COLS = ["Shell ID", "grSim ID", "IP Address", "Port"]

    def __init__(self, parent=None, engine=None):
        super().__init__(parent)
        self._engine = engine
        self._last_info = None
        self._build_ui()
        self._load_shell_maps()

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        root.addWidget(_heading("Dispatcher Inspector"))

        splitter = QSplitter(Qt.Horizontal)

        # ── Left: running commands ────────────────────────────────
        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        ll.setSpacing(8)

        cmd_card, cmd_lay = _card("Running Commands")

        self._cmd_table = QTableWidget(0, len(self.CMD_COLS))
        self._cmd_table.setHorizontalHeaderLabels(self.CMD_COLS)
        self._cmd_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._cmd_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._cmd_table.setAlternatingRowColors(True)
        self._cmd_table.verticalHeader().setVisible(False)
        self._cmd_table.setShowGrid(True)
        hh = self._cmd_table.horizontalHeader()
        hh.setSectionResizeMode(QHeaderView.Stretch)
        hh.setMinimumSectionSize(45)
        cmd_lay.addWidget(self._cmd_table)
        ll.addWidget(cmd_card)

        # Raw dict view
        raw_card, raw_lay = _card("Raw Command Dict")
        self._raw_text = QPlainTextEdit()
        self._raw_text.setReadOnly(True)
        self._raw_text.setFont(QFont("Cascadia Code", 10))
        self._raw_text.setMaximumBlockCount(200)
        raw_lay.addWidget(self._raw_text)
        ll.addWidget(raw_card)

        splitter.addWidget(left)

        # ── Right: config & shell maps ────────────────────────────
        right = QWidget()
        rl = QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(8)

        # Status card
        stat_card, stat_lay = _card("Dispatcher Status")
        sg = QGridLayout()
        sg.setSpacing(6)

        sg.addWidget(QLabel("State:"), 0, 0)
        self._state_lbl = QLabel("IDLE")
        self._state_lbl.setFont(QFont("Segoe UI", 11, QFont.Bold))
        sg.addWidget(self._state_lbl, 0, 1)

        sg.addWidget(QLabel("Send to grSim:"), 1, 0)
        self._grsim_lbl = QLabel("—")
        self._grsim_lbl.setFont(QFont("Segoe UI", 11, QFont.Bold))
        sg.addWidget(self._grsim_lbl, 1, 1)

        sg.addWidget(QLabel("Queue depth:"), 2, 0)
        self._queue_lbl = QLabel("—")
        self._queue_lbl.setFont(QFont("Segoe UI", 11, QFont.Bold))
        sg.addWidget(self._queue_lbl, 2, 1)

        sg.addWidget(QLabel("Active robots:"), 3, 0)
        self._active_lbl = QLabel("0")
        self._active_lbl.setFont(QFont("Segoe UI", 11, QFont.Bold))
        sg.addWidget(self._active_lbl, 3, 1)

        sg.addWidget(QLabel("Last update:"), 4, 0)
        self._update_lbl = QLabel("—")
        self._update_lbl.setStyleSheet(f"color:{TEXT_DIM};")
        sg.addWidget(self._update_lbl, 4, 1)

        stat_lay.addLayout(sg)
        rl.addWidget(stat_card)

        # Yellow shell map
        y_card, y_lay = _card("Yellow Shell Map")
        self._yellow_table = QTableWidget(0, len(self.SHELL_COLS))
        self._yellow_table.setHorizontalHeaderLabels(self.SHELL_COLS)
        self._yellow_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._yellow_table.setAlternatingRowColors(True)
        self._yellow_table.verticalHeader().setVisible(False)
        self._yellow_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._yellow_table.setMaximumHeight(200)
        y_lay.addWidget(self._yellow_table)
        rl.addWidget(y_card)

        # Blue shell map
        b_card, b_lay = _card("Blue Shell Map")
        self._blue_table = QTableWidget(0, len(self.SHELL_COLS))
        self._blue_table.setHorizontalHeaderLabels(self.SHELL_COLS)
        self._blue_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._blue_table.setAlternatingRowColors(True)
        self._blue_table.verticalHeader().setVisible(False)
        self._blue_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._blue_table.setMaximumHeight(200)
        b_lay.addWidget(self._blue_table)
        rl.addWidget(b_card)

        rl.addStretch()
        splitter.addWidget(right)

        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        root.addWidget(splitter)

    # ── Public update API ─────────────────────────────────────────

    def update_info(self, info: dict):
        self._last_info = info
        self._update_status(info)
        self._update_cmd_table(info.get("commands", {}))
        self._update_raw(info.get("commands", {}))
        self._update_shell_table(self._yellow_table, info.get("yellow_shells", {}),
                                 YELLOW_TEAM)
        self._update_shell_table(self._blue_table, info.get("blue_shells", {}),
                                 BLUE_TEAM)

    def set_running(self, running):
        if running:
            self._state_lbl.setText("RUNNING")
            self._state_lbl.setStyleSheet(f"color:{SUCCESS}; font-weight:bold;")
            self._load_shell_maps()
        else:
            self._state_lbl.setText("IDLE")
            self._state_lbl.setStyleSheet(f"color:{TEXT_DIM};")
            self._cmd_table.setRowCount(0)
            self._raw_text.clear()
            self._queue_lbl.setText("—")
            self._active_lbl.setText("0")

    # ── Internal ──────────────────────────────────────────────────

    def _update_status(self, info):
        grsim = info.get("send_to_grSim", False)
        self._grsim_lbl.setText("YES" if grsim else "NO")
        self._grsim_lbl.setStyleSheet(
            f"color:{SUCCESS if grsim else TEXT_DIM}; font-weight:bold;")

        qs = info.get("queue_size", -1)
        self._queue_lbl.setText(str(qs) if qs >= 0 else "N/A")
        if qs > 10:
            self._queue_lbl.setStyleSheet(f"color:{DANGER}; font-weight:bold;")
        elif qs > 3:
            self._queue_lbl.setStyleSheet(f"color:{WARNING}; font-weight:bold;")
        else:
            self._queue_lbl.setStyleSheet(f"color:{SUCCESS}; font-weight:bold;")

        n_cmds = len(info.get("commands", {}))
        self._active_lbl.setText(str(n_cmds))
        self._active_lbl.setStyleSheet(
            f"color:{ACCENT}; font-weight:bold;" if n_cmds else f"color:{TEXT_DIM};")

        from PySide6.QtCore import QTime
        self._update_lbl.setText(QTime.currentTime().toString("HH:mm:ss.zzz"))

    def _update_cmd_table(self, commands: dict):
        self._cmd_table.setRowCount(len(commands))
        for row, (rid, c) in enumerate(commands.items()):
            team = "Yellow" if c["isYellow"] else "Blue"
            color = QColor(YELLOW_TEAM if c["isYellow"] else BLUE_TEAM)
            vals = [
                str(c["robot_id"]),
                team,
                f'{c["vx"]:.3f}',
                f'{c["vy"]:.3f}',
                f'{c["w"]:.3f}',
                str(c["kick"]),
                str(c["dribble"]),
                f'{c["elapsed"]:.1f}s',
                f'{c["runtime"]:.1f}s' if c["runtime"] < 999999 else "∞",
                c.get("ip", "?"),
                str(c.get("port", "?")),
                str(c.get("sends", 0)),
            ]
            for col, text in enumerate(vals):
                item = QTableWidgetItem(text)
                item.setTextAlignment(Qt.AlignCenter)
                if col == 1:
                    item.setForeground(color)
                    item.setFont(QFont("Segoe UI", 10, QFont.Bold))
                # Highlight non-zero velocities
                if col in (2, 3, 4):
                    try:
                        v = float(text)
                        if abs(v) > 0.01:
                            item.setForeground(QColor(SUCCESS))
                    except ValueError:
                        pass
                if col == 5 and text == "1":
                    item.setForeground(QColor(WARNING))
                if col == 6 and text == "1":
                    item.setForeground(QColor(ACCENT))
                self._cmd_table.setItem(row, col, item)

    def _update_raw(self, commands: dict):
        if not commands:
            self._raw_text.setPlainText("No active commands")
            return
        lines = []
        for rid, c in commands.items():
            team = "Y" if c["isYellow"] else "B"
            lines.append(
                f"[{team}{c['robot_id']}] "
                f"vx={c['vx']:+.3f} vy={c['vy']:+.3f} w={c['w']:+.3f} "
                f"k={c['kick']} d={c['dribble']} "
                f"→ {c.get('ip','?')}:{c.get('port','?')} "
                f"({c.get('sends',0)} pkts, {c['elapsed']:.1f}s)")
        self._raw_text.setPlainText("\n".join(lines))

    def _update_shell_table(self, table: QTableWidget, shells: dict,
                            color_hex: str):
        table.setRowCount(len(shells))
        color = QColor(color_hex)
        for row, (sid, info) in enumerate(shells.items()):
            vals = [
                str(sid),
                str(info.get("grSimID", "?")),
                info.get("ip", "?"),
                str(info.get("port", "?")),
            ]
            for col, text in enumerate(vals):
                item = QTableWidgetItem(text)
                item.setTextAlignment(Qt.AlignCenter)
                if col == 0:
                    item.setForeground(color)
                    item.setFont(QFont("Segoe UI", 10, QFont.Bold))
                table.setItem(row, col, item)

    def _load_shell_maps(self):
        """Load shell maps from ipconfig.yaml on startup so the panel
        always has something to show even before the dispatcher runs."""
        try:
            from TeamControl.utils.yaml_config import Config
            cfg = Config()
            y_shells = {}
            if cfg.yellow:
                for rkey, rdict in cfg.yellow.items():
                    sid = rdict.get("shellID")
                    if sid is not None:
                        y_shells[sid] = {
                            "ip": rdict.get("ip", "?"),
                            "port": rdict.get("port", "?"),
                            "grSimID": rdict.get("grSimID", "?"),
                        }
            b_shells = {}
            if cfg.blue:
                for rkey, rdict in cfg.blue.items():
                    sid = rdict.get("shellID")
                    if sid is not None:
                        b_shells[sid] = {
                            "ip": rdict.get("ip", "?"),
                            "port": rdict.get("port", "?"),
                            "grSimID": rdict.get("grSimID", "?"),
                        }
            self._update_shell_table(self._yellow_table, y_shells, YELLOW_TEAM)
            self._update_shell_table(self._blue_table, b_shells, BLUE_TEAM)
            self._grsim_lbl.setText("YES" if cfg.send_to_grSim else "NO")
            self._grsim_lbl.setStyleSheet(
                f"color:{SUCCESS if cfg.send_to_grSim else TEXT_DIM}; font-weight:bold;")
        except Exception:
            pass
