"""
2-D SSL field rendered with QPainter.

Features:
  - Anti-aliased green pitch with white markings
  - Yellow / blue robots (circles with orientation arrows + ID labels)
  - Orange ball
  - Click-to-place: left-click places ball, right-click queues robot placement
  - Hover tooltip with mm coordinates
  - Zoom / pan via mouse wheel + middle-drag
"""

import math
import time
from PySide6.QtWidgets import QWidget, QToolTip, QMenu, QPushButton, QHBoxLayout, QLabel
from PySide6.QtCore import Qt, QPointF, QRectF, Signal, QSize
from PySide6.QtGui import (QPainter, QPen, QBrush, QColor, QFont,
                           QPainterPath, QTransform, QWheelEvent,
                           QMouseEvent, QPaintEvent, QResizeEvent)

from TeamControl.ui.theme import (FIELD_GREEN, FIELD_LINE, YELLOW_TEAM,
                                   BLUE_TEAM, BALL_COLOR, BG_DARK, ACCENT,
                                   ROLE_GOALIE, ROLE_ATTACKER, ROLE_SUPPORT,
                                   ROLE_DEFENDER, TEXT)
from TeamControl.robot.constants import (
    FIELD_LENGTH, FIELD_WIDTH, HALF_LEN, HALF_WID,
    PENALTY_DEPTH, PENALTY_WIDTH, CENTER_RADIUS,
    GOAL_DEPTH, GOAL_WIDTH, ROBOT_RADIUS, FIELD_MARGIN as MARGIN,
)


class FieldCanvas(QWidget):
    """Interactive 2-D SSL field widget."""

    ball_placed = Signal(float, float)            # x_mm, y_mm
    robot_placed = Signal(int, bool, float, float)  # id, yellow, x, y
    point_picked = Signal(float, float)           # x_mm, y_mm — for go-to-point
    action_requested = Signal(str)                # action name
    coordinate_hover = Signal(float, float)       # x_mm, y_mm

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(600, 400)
        self.setMouseTracking(True)

        # Data
        self._yellow: list = []
        self._blue: list = []
        self._ball = None
        self._targets: list[tuple] = []
        self._paths: list[list[tuple]] = []
        self._frame_number = 0

        # View transform
        self._scale = 1.0
        self._offset = QPointF(0, 0)
        self._dragging = False
        self._drag_start = QPointF()

        # Placement state (set by sim panel)
        self._place_mode = None   # "ball", ("robot", id, yellow)

        # Velocity overlay
        self._show_velocity = False
        self._vel_prev: dict = {}   # (team, id) -> (x, y, timestamp)
        self._velocities: dict = {} # (team, id) -> (vx_mm_s, vy_mm_s)

        # Zoom overlay (built last so it sits on top)
        self._build_zoom_overlay()

    # ── Public API ────────────────────────────────────────────────

    def set_show_velocity(self, enabled: bool):
        self._show_velocity = enabled
        self.update()

    def set_frame(self, snap):
        if self._show_velocity:
            now = time.monotonic()
            new_vel = {}
            for r in (*snap.yellow, *snap.blue):
                key = (r.team, r.id)
                if key in self._vel_prev:
                    px, py, pt = self._vel_prev[key]
                    dt = now - pt
                    if 0 < dt < 0.5:
                        new_vel[key] = ((r.x - px) / dt, (r.y - py) / dt)
                self._vel_prev[key] = (r.x, r.y, now)
            self._velocities = new_vel

        self._yellow = snap.yellow
        self._blue = snap.blue
        self._ball = snap.ball
        self._frame_number = snap.frame_number
        self.update()

    def set_targets(self, targets):
        self._targets = list(targets)
        self.update()

    def set_paths(self, paths):
        self._paths = list(paths)
        self.update()

    def set_place_mode(self, mode):
        self._place_mode = mode
        if mode:
            self.setCursor(Qt.CrossCursor)
        else:
            self.setCursor(Qt.ArrowCursor)

    # ── Zoom overlay ──────────────────────────────────────────────

    def _build_zoom_overlay(self):
        self._zoom_overlay = QWidget(self)
        self._zoom_overlay.setStyleSheet(
            "QWidget { background: #1e1e2e; border: 1px solid #444; border-radius: 5px; }"
        )

        lay = QHBoxLayout(self._zoom_overlay)
        lay.setContentsMargins(6, 4, 6, 4)
        lay.setSpacing(4)

        icon = QLabel("🔍")
        icon.setStyleSheet("background: transparent; border: none; font-size: 14px; color: #cccccc;")
        lay.addWidget(icon)

        btn_style = (
            "QPushButton { background: #2a2a3e; color: #ffffff; border: none;"
            " border-radius: 3px; font-size: 16px; font-weight: bold; padding: 0px; }"
            "QPushButton:hover { background: #3a3a5e; }"
            "QPushButton:pressed { background: #e94560; }"
        )
        for text, tip, slot in [
            ("+", "Zoom In",     self._zoom_in),
            ("-", "Zoom Out",    self._zoom_out),
            ("↺", "Fit to View", self._zoom_fit),
        ]:
            btn = QPushButton(text)
            btn.setFixedSize(32, 28)
            btn.setToolTip(tip)
            btn.setStyleSheet(btn_style)
            btn.clicked.connect(slot)
            lay.addWidget(btn)

        self._zoom_overlay.adjustSize()

    def _zoom_in(self):
        self._scale = min(5.0, self._scale * 1.2)
        self.update()

    def _zoom_out(self):
        self._scale = max(0.2, self._scale * 0.8)
        self.update()

    def _zoom_fit(self):
        self._scale = 1.0
        self._offset = QPointF(0, 0)
        self.update()

    def resizeEvent(self, event: QResizeEvent):
        super().resizeEvent(event)
        self._zoom_overlay.adjustSize()
        margin = 10
        self._zoom_overlay.move(
            self.width() - self._zoom_overlay.width() - margin, margin)
        self._zoom_overlay.raise_()

    # ── Coordinate transforms ─────────────────────────────────────

    def _view_transform(self) -> QTransform:
        w, h = self.width(), self.height()
        total_w = FIELD_LENGTH + 2 * MARGIN + 2 * GOAL_DEPTH
        total_h = FIELD_WIDTH + 2 * MARGIN
        sx = w / total_w * self._scale
        sy = h / total_h * self._scale
        s = min(sx, sy)

        t = QTransform()
        t.translate(w / 2 + self._offset.x(), h / 2 + self._offset.y())
        t.scale(s, -s)  # flip Y so positive Y is up
        return t

    def _widget_to_field(self, pos: QPointF) -> QPointF:
        inv, ok = self._view_transform().inverted()
        if ok:
            return inv.map(pos)
        return QPointF(0, 0)

    # ── Painting ──────────────────────────────────────────────────

    def paintEvent(self, event: QPaintEvent):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.SmoothPixmapTransform)

        # Background
        p.fillRect(self.rect(), QColor(BG_DARK))

        p.setTransform(self._view_transform())
        self._draw_field(p)
        self._draw_targets(p)
        self._draw_paths(p)
        self._draw_robots(p, self._yellow, QColor(YELLOW_TEAM))
        self._draw_robots(p, self._blue, QColor(BLUE_TEAM))
        self._draw_ball(p)

        # Frame counter overlay
        p.resetTransform()
        p.setPen(QColor(TEXT))
        p.setFont(QFont("Segoe UI", 10))
        p.drawText(10, self.height() - 10, f"Frame: {self._frame_number}")
        p.end()

    def _draw_field(self, p: QPainter):
        outer = QRectF(-(HALF_LEN + MARGIN + GOAL_DEPTH),
                       -(HALF_WID + MARGIN),
                       FIELD_LENGTH + 2 * MARGIN + 2 * GOAL_DEPTH,
                       FIELD_WIDTH + 2 * MARGIN)
        p.fillRect(outer, QColor(FIELD_GREEN))

        pen = QPen(QColor(FIELD_LINE), 20)
        p.setPen(pen)
        p.setBrush(Qt.NoBrush)

        # Outer boundary
        p.drawRect(QRectF(-HALF_LEN, -HALF_WID, FIELD_LENGTH, FIELD_WIDTH))

        # Center line
        p.drawLine(QPointF(0, -HALF_WID), QPointF(0, HALF_WID))

        # Center circle
        p.drawEllipse(QPointF(0, 0), CENTER_RADIUS, CENTER_RADIUS)

        # Center dot
        p.setBrush(QColor(FIELD_LINE))
        p.drawEllipse(QPointF(0, 0), 20, 20)
        p.setBrush(Qt.NoBrush)

        # Left penalty area
        ph = PENALTY_WIDTH / 2
        p.drawRect(QRectF(-HALF_LEN, -ph, PENALTY_DEPTH, PENALTY_WIDTH))

        # Right penalty area
        p.drawRect(QRectF(HALF_LEN - PENALTY_DEPTH, -ph,
                          PENALTY_DEPTH, PENALTY_WIDTH))

        # Left goal
        gh = GOAL_WIDTH / 2
        goal_pen = QPen(QColor("#cccccc"), 16)
        p.setPen(goal_pen)
        p.drawRect(QRectF(-HALF_LEN - GOAL_DEPTH, -gh, GOAL_DEPTH, GOAL_WIDTH))

        # Right goal
        p.drawRect(QRectF(HALF_LEN, -gh, GOAL_DEPTH, GOAL_WIDTH))

    def _draw_robots(self, p: QPainter, robots, color: QColor):
        if not robots:
            return
        for r in robots:
            cx, cy = r.x, r.y
            # Body
            p.setPen(QPen(QColor("#111111"), 12))
            p.setBrush(QBrush(color))
            p.drawEllipse(QPointF(cx, cy), ROBOT_RADIUS, ROBOT_RADIUS)

            # Orientation arrow
            arr_len = ROBOT_RADIUS * 1.6
            ax = cx + math.cos(r.o) * arr_len
            ay = cy + math.sin(r.o) * arr_len
            arrow_pen = QPen(color.darker(140), 18)
            arrow_pen.setCapStyle(Qt.RoundCap)
            p.setPen(arrow_pen)
            p.drawLine(QPointF(cx, cy), QPointF(ax, ay))

            # Arrowhead
            head = 50
            a1 = r.o + math.pi * 0.82
            a2 = r.o - math.pi * 0.82
            p.drawLine(QPointF(ax, ay),
                       QPointF(ax + math.cos(a1) * head,
                               ay + math.sin(a1) * head))
            p.drawLine(QPointF(ax, ay),
                       QPointF(ax + math.cos(a2) * head,
                               ay + math.sin(a2) * head))

            # ID label
            p.setPen(QPen(QColor("#000000"), 1))
            font = QFont("Segoe UI", 1)
            font.setPixelSize(80)
            font.setBold(True)
            p.setFont(font)
            text_rect = QRectF(cx - 50, cy - 50, 100, 100)
            p.drawText(text_rect, Qt.AlignCenter, str(r.id))

            # Velocity vector
            if self._show_velocity:
                key = (r.team, r.id)
                if key in self._velocities:
                    vx, vy = self._velocities[key]
                    speed = math.hypot(vx, vy)
                    if speed > 100:  # suppress noise below 100 mm/s
                        scale = 0.3   # show 0.3 s of travel
                        ex = cx + vx * scale
                        ey = cy + vy * scale
                        vel_pen = QPen(QColor("#ffffff"), 14)
                        vel_pen.setCapStyle(Qt.RoundCap)
                        p.setPen(vel_pen)
                        p.drawLine(QPointF(cx, cy), QPointF(ex, ey))
                        angle = math.atan2(vy, vx)
                        head = 45
                        p.drawLine(QPointF(ex, ey),
                                   QPointF(ex + math.cos(angle + 2.6) * head,
                                           ey + math.sin(angle + 2.6) * head))
                        p.drawLine(QPointF(ex, ey),
                                   QPointF(ex + math.cos(angle - 2.6) * head,
                                           ey + math.sin(angle - 2.6) * head))

    def _draw_ball(self, p: QPainter):
        if not self._ball:
            return
        bx, by = self._ball.x, self._ball.y
        p.setPen(QPen(QColor("#000000"), 8))
        p.setBrush(QBrush(QColor(BALL_COLOR)))
        p.drawEllipse(QPointF(bx, by), 45, 45)

    def _draw_targets(self, p: QPainter):
        if not self._targets:
            return
        for item in self._targets:
            if len(item) == 3:
                tx, ty, color_hex = item
            else:
                tx, ty = item
                color_hex = ACCENT
            p.setPen(QPen(QColor(color_hex), 16))
            p.setBrush(Qt.NoBrush)
            size = 60
            p.drawLine(QPointF(tx - size, ty - size),
                       QPointF(tx + size, ty + size))
            p.drawLine(QPointF(tx - size, ty + size),
                       QPointF(tx + size, ty - size))

    def _draw_paths(self, p: QPainter):
        for path_entry in self._paths:
            if len(path_entry) < 2:
                continue
            # Support (points, color_hex) tuple or plain list of points
            if isinstance(path_entry, tuple) and len(path_entry) == 2 \
               and isinstance(path_entry[1], str):
                points, color_hex = path_entry
            else:
                points = path_entry
                color_hex = ACCENT
            pen = QPen(QColor(color_hex), 10, Qt.DashLine)
            p.setPen(pen)
            for i in range(len(points) - 1):
                p.drawLine(QPointF(*points[i]), QPointF(*points[i + 1]))

    # ── Input events ──────────────────────────────────────────────

    def mousePressEvent(self, ev: QMouseEvent):
        if ev.button() == Qt.MiddleButton:
            self._dragging = True
            self._drag_start = ev.position()
            ev.accept()
            return

        if ev.button() == Qt.LeftButton and self._place_mode:
            pt = self._widget_to_field(ev.position())
            if self._place_mode == "ball":
                self.ball_placed.emit(pt.x(), pt.y())
            elif self._place_mode == "go_to_point":
                self.point_picked.emit(pt.x(), pt.y())
            elif isinstance(self._place_mode, tuple):
                _, rid, yellow = self._place_mode
                self.robot_placed.emit(rid, yellow, pt.x(), pt.y())
            self._place_mode = None
            self.setCursor(Qt.ArrowCursor)
            ev.accept()
            return

        if ev.button() == Qt.RightButton:
            self._show_field_menu(ev)
            ev.accept()
            return

        super().mousePressEvent(ev)

    def _show_field_menu(self, ev: QMouseEvent):
        pt = self._widget_to_field(ev.position())
        x, y = pt.x(), pt.y()

        menu = QMenu(self)
        go_action = menu.addAction(f"Go to ({x:.0f}, {y:.0f})")
        go_ball_action = menu.addAction("Go to Ball")
        go_ball_kick_action = menu.addAction("Go to Ball && Kick")
        draw_square_action = menu.addAction("Draw Square")
        menu.addSeparator()
        ball_action = menu.addAction("Place ball here")
        menu.addSeparator()
        stop_action = menu.addAction("Stop")

        chosen = menu.exec(ev.globalPosition().toPoint())
        if chosen == go_action:
            self.point_picked.emit(x, y)
        elif chosen == go_ball_action:
            self.action_requested.emit("go_to_ball")
        elif chosen == go_ball_kick_action:
            self.action_requested.emit("go_to_ball_kick")
        elif chosen == draw_square_action:
            self.action_requested.emit("draw_square")
        elif chosen == ball_action:
            self.ball_placed.emit(x, y)
        elif chosen == stop_action:
            self.action_requested.emit("stop")

    def mouseReleaseEvent(self, ev: QMouseEvent):
        if ev.button() == Qt.MiddleButton:
            self._dragging = False
            ev.accept()
            return
        super().mouseReleaseEvent(ev)

    def mouseMoveEvent(self, ev: QMouseEvent):
        if self._dragging:
            delta = ev.position() - self._drag_start
            self._offset += delta
            self._drag_start = ev.position()
            self.update()
            ev.accept()
            return

        pt = self._widget_to_field(ev.position())
        self.coordinate_hover.emit(pt.x(), pt.y())
        QToolTip.showText(ev.globalPosition().toPoint(),
                          f"({pt.x():.0f}, {pt.y():.0f}) mm",
                          self)
        super().mouseMoveEvent(ev)

    def wheelEvent(self, ev: QWheelEvent):
        delta = ev.angleDelta().y()
        factor = 1.1 if delta > 0 else 0.9
        self._scale = max(0.2, min(5.0, self._scale * factor))
        self.update()
        ev.accept()

    def mouseDoubleClickEvent(self, ev: QMouseEvent):
        self._scale = 1.0
        self._offset = QPointF(0, 0)
        self.update()

    def sizeHint(self):
        return QSize(900, 600)
