#!/usr/bin/env python3 
#
# drones_panel.py
# Read-only "Drones" panel for the Click'n Fly operator window.
#
import os
import time
import numpy as np
import battery_limits

from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QPixmap, QPainter
from PySide6.QtSvg import QSvgRenderer
from PySide6.QtWidgets import (QGroupBox, QFrame, QLabel, QSizePolicy,
                               QPushButton, QVBoxLayout, QHBoxLayout, QWidget)


_ICON_SIZE = 20
# Paparazzi GCS strip icons (vendored SVGs under media/pprz_icons, from
# github.com/paparazzi/PprzGCS, GPL). Health indicator -> icon base name;
_ICON_DIR = os.path.join(os.path.dirname(__file__), 'media', 'pprz_icons')
_ICON_BASE = {"mocap": "gps", "RC": "rc", "link": "link", "battery": "battery"}
_STATE_SUFFIX = {"ok": "ok", "warn": "warning", "bad": "nok", "unknown": "nok"}
_icon_cache = {}

def _state_icon(kind, state, size=_ICON_SIZE):
    """Render the pprz strip icon for a health indicator, picked by state
    (ok green / warning amber / nok red; unknown = faint)."""
    key = (kind, state, size)
    pm = _icon_cache.get(key)
    if pm is not None:
        return pm
    base = _ICON_BASE.get(kind, "link")
    path = os.path.join(_ICON_DIR, f"{base}_{_STATE_SUFFIX.get(state, 'nok')}.svg")
    if not os.path.exists(path):          # rc/gps have no _warning variant
        path = os.path.join(_ICON_DIR, f"{base}_nok.svg")
    pm = QPixmap(size, size)
    pm.fill(Qt.GlobalColor.transparent)
    p = QPainter(pm)
    if state == "unknown":
        p.setOpacity(0.35)                # not heard from yet: dim it
    QSvgRenderer(path).render(p)
    p.end()
    _icon_cache[key] = pm
    return pm



DRONE_COLORS = ["#FFD400", "#00E676", "#D500F9"]      # yellow / green / violet
_DEFAULT_COLORS = DRONE_COLORS

_VALUE = "#C7D0CB"
_MUTED = "#8B938F"

_DEFAULT_LIMITS = battery_limits.BatteryLimits()
_BATT_LOW_COLOR  = "#F2A33C"
_BATT_CRIT_COLOR = "#F85149"       # same red as the HMI error state


def _limits_of(ac):
    """Battery limits of a drone, falling back to the defaults."""
    return getattr(ac, "batt_limits", None) or _DEFAULT_LIMITS
 
 
def battery_state(v, limits=None):
    """Classify a pack voltage into the health states used by both the
    panel icon and the flight logic (start gate / auto-stop). Single
    source of truth for the battery thresholds; `limits` are the drone's
    own (from its airframe), else the defaults."""
    return (limits or _DEFAULT_LIMITS).state(v)

# --- Pre-flight checklist -----------------------------------------------
# The GCS "green icons", brought into the operator's window (ConOps §4):
# per drone, is the mocap pose flowing, is the RC link up, is the
# telemetry downlink alive. Grey = never heard, green = good, red = was
# there and is gone
_CHECK_COLORS = {"ok": "#3FB950", "warn": _BATT_LOW_COLOR,
                 "bad": _BATT_CRIT_COLOR, "unknown": _MUTED}
_MOCAP_STALE_S  = 1.0   # EXTERNAL_POSE flows at mocap rate: >1s = chain down
_STATUS_STALE_S = 5.0   # ROTORCRAFT_STATUS is slow telemetry: be tolerant

_RC_STATUS_NAMES = {0: "OK", 1: "LOST", 2: "REALLY_LOST"}
_AP_MODE_NAMES = ("KILL", "FAILSAFE", "HOME", "RATE_DIRECT", "ATTITUDE_DIRECT",
                  "RATE_RC_CLIMB", "ATTITUDE_RC_CLIMB", "ATTITUDE_CLIMB",
                  "RATE_Z_HOLD", "ATTITUDE_Z_HOLD", "HOVER_DIRECT",
                  "HOVER_CLIMB", "HOVER_Z_HOLD", "NAV", "RC_DIRECT",
                  "CARE_FREE", "FORWARD", "MODULE", "FLIP", "GUIDED")
_ARMING_NAMES = ("NO_RC", "WAITING", "ARMING", "ARMED", "DISARMING",
                 "KILLED", "YAW_CENTERED", "THROTTLE_DOWN",
                 "NOT_MODE_MANUAL", "UNARMED_IN_AUTO", "THROTTLE_NOT_DOWN",
                 "STICKS_NOT_CENTERED", "PITCH_NOT_CENTERED",
                 "ROLL_NOT_CENTERED", "YAW_NOT_CENTERED",
                 "AHRS_NOT_ALLIGNED", "OUT_OF_GEOFENCE", "LOW_BATTERY")


PANEL_STYLE = """
QGroupBox#dronesPanel {
    background: transparent; border: none;
    margin-top: 0px; padding: 0px;
}
"""

_SPEED_ALPHA = 0.3   # lissage de la vitesse estimee (0..1, plus haut = plus reactif)


_KILL_IDLE  = ("background-color:#7A2B26; color:#FFEDEB; font-weight:700;"
               " font-size:12px; border:none; border-radius:4px; padding:3px 10px;")
_KILL_ARMED = ("background-color:#F85149; color:#FFFFFF; font-weight:700;"
               " font-size:12px; border:none; border-radius:4px; padding:3px 10px;")


class _DroneRow(QFrame):

    def __init__(self, drone_id, color, traj_name="", kill_cbk=None):
        super().__init__()
        self.drone_id = drone_id
        self._kill_cbk = kill_cbk
        self._kill_armed = False

        self.setStyleSheet(
            "QFrame { background:#202622; border:1px solid #2A312D;"
            f" border-left:3px solid {color}; border-radius:5px; }}")

        v = QVBoxLayout(self)
        v.setContentsMargins(10, 8, 10, 8)
        v.setSpacing(4)


        top = QHBoxLayout()
        top.setSpacing(8)

        name = QLabel(f"D{drone_id}")
        name.setStyleSheet("color:#E8ECEA; font-size:15px; font-weight:600; border:none;")

        self.lbl_traj = QLabel(traj_name)
        self.lbl_traj.setStyleSheet(f"color:{_MUTED}; font-size:13px; border:none;")
        self.lbl_traj.setToolTip(traj_name)

        # header: id + trajectory + kill only 
        top.addWidget(name)
        top.addWidget(self.lbl_traj)
        top.addStretch(1)

        self.button_kill = QPushButton("KILL")
        self.button_kill.setStyleSheet(_KILL_IDLE)
        self.button_kill.setToolTip(f"Cut motors of drone {drone_id} (it falls)")
        if kill_cbk is None:
            self.button_kill.setEnabled(False)
        else:
            self.button_kill.clicked.connect(self._on_kill_pressed)
        top.addWidget(self.button_kill)
        v.addLayout(top)


        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Minimum)


        self.lbl_metrics = QLabel()
        self.lbl_metrics.setWordWrap(False)   # keep it on a single line
        self.lbl_metrics.setStyleSheet(
            f"color:{_MUTED}; font-size:14px; border:none;"
            " font-family:'DejaVu Sans Mono','Menlo','Consolas',monospace;")
        v.addWidget(self.lbl_metrics)

        # one state line: health icons (mocap/RC/link, detail in the
        # tooltip) on the left, flight mode on the right
        stateline = QHBoxLayout()
        stateline.setSpacing(8)
        self._icons = {}
        for kind in ("mocap", "RC", "link", "battery"):
            lab = QLabel()
            lab.setFixedSize(_ICON_SIZE, _ICON_SIZE)
            # QLabel is a QFrame, so without this it inherits the row's
            # coloured left border (the stray blue bar next to each icon)
            lab.setStyleSheet("border:none;")
            self._icons[kind] = lab
            stateline.addWidget(lab)
        stateline.addStretch(1)
        self.lbl_nav = QLabel()
        self.lbl_nav.setStyleSheet(
            f"color:{_MUTED}; font-size:13px; border:none;"
            " font-family:'DejaVu Sans Mono','Menlo','Consolas',monospace;")
        stateline.addWidget(self.lbl_nav)
        v.addLayout(stateline)

        self.set_values(None, None, None)
        self.set_checklist([("mocap", "unknown", "no EXTERNAL_POSE seen yet"),
                            ("RC", "unknown", "no ROTORCRAFT_STATUS seen yet"),
                            ("link", "unknown", "no ROTORCRAFT_STATUS seen yet"),
                            ("battery", "unknown", "no ROTORCRAFT_STATUS seen yet")])
        self.set_nav("—", "")


    def _on_kill_pressed(self):
        if not self._kill_armed:
            self._kill_armed = True
            self.button_kill.setText("CONFIRM")
            self.button_kill.setStyleSheet(_KILL_ARMED)
            QTimer.singleShot(3000, self._disarm_kill)
            return
        self._disarm_kill()
        if self._kill_cbk is not None:
            self._kill_cbk(self.drone_id)

    def _disarm_kill(self):
        self._kill_armed = False
        self.button_kill.setText("KILL")
        self.button_kill.setStyleSheet(_KILL_IDLE)

    def set_traj_name(self, name):
        self.lbl_traj.setText(name)
        self.lbl_traj.setToolTip(name)


    def set_checklist(self, items):
        """items: list of (label, state, detail) with state in _CHECK_COLORS.
        Draws a state-coloured icon per item; the detail lives in the
        per-icon tooltip."""
        for label, state, detail in items:
            icon = self._icons.get(label)
            if icon is None:
                continue
            icon.setPixmap(_state_icon(label, state))
            icon.setToolTip(f"{label}: {detail}")


    def set_nav(self, html, tip=""):
        self.lbl_nav.setText(html)
        self.lbl_nav.setToolTip(tip)


    def set_values(self, alt, spd, batt=None, limits=None):
       def cell(label, value, unit, fmt, color=_VALUE, bold=False):
            v = (fmt % value + unit) if value is not None else "\u2014"
            weight = "font-weight:700;" if bold else ""
        return f'{label} <span style="color:{color};{weight}">{v}</span>'
        batt_color, batt_bold = _VALUE, False
        bstate = battery_state(batt, limits)
        if bstate == "bad":
            batt_color, batt_bold = _BATT_CRIT_COLOR, True
        elif bstate == "warn":
            batt_color = _BATT_LOW_COLOR
        # per cell, not pack: 4.2 V full / 3.5 V low reads the same whatever
        # the pack size (the tooltip keeps the pack voltage)
        lim = limits or _DEFAULT_LIMITS
      
        self.lbl_metrics.setText(
            cell("alt", alt, "m", "%.2f") + "   "
            + cell("spd", spd, "m/s", "%.1f") + "   "
            + cell("batt", lim.per_cell(batt), "V/cell", "%.2f",
                   batt_color, batt_bold))

class DronesPanel(QGroupBox):
    def __init__(self, ids, colors=None, trajs=None, kill_cbk=None):
        super().__init__()
        self.setObjectName("dronesPanel")
        self.setStyleSheet(PANEL_STYLE)
        self.ids = list(ids)
        self.colors = list(colors) if colors else list(_DEFAULT_COLORS)
        self._kill_cbk = kill_cbk
        trajs = list(trajs) if trajs else []

        v = QVBoxLayout(self)
        v.setContentsMargins(2, 2, 2, 2)
        v.setSpacing(6)

        header = QHBoxLayout()
        title = QLabel("DRONES")
        title.setStyleSheet("color:#6E7770; font-size:11px; letter-spacing:1.5px; border:none;")
        self.lbl_flight_time = QLabel()
        self.lbl_flight_time.setStyleSheet(
            "color:#8B938F; font-size:12px; border:none;"
            " font-family:'DejaVu Sans Mono','Menlo','Consolas',monospace;")
        header.addWidget(title)
        header.addStretch(1)
        header.addWidget(self.lbl_flight_time)
        v.addLayout(header)
        self._set_flight_time(None)

        self.rows = {}
        rows_box = QWidget()
        rows_box.setStyleSheet("background:transparent; border:none;")
        rows_lay = QVBoxLayout(rows_box)
        rows_lay.setContentsMargins(0, 0, 0, 0)
        rows_lay.setSpacing(8)
        for i, _id in enumerate(self.ids):
            color = self.colors[i % len(self.colors)]
            tname = trajs[i] if i < len(trajs) else ""
            row = _DroneRow(_id, color, tname, kill_cbk=kill_cbk)
            self.rows[_id] = row
            rows_lay.addWidget(row)
        v.addWidget(rows_box)

        self._prev = {}
        self._speed = {}

    def _set_flight_time(self, text):
        self.lbl_flight_time.setText(text if text is not None else "\u2014")

    @staticmethod
    def _checklist_items(ac, now):
        items = []

        # mocap: is the ground->drone EXTERNAL_POSE uplink flowing on the bus
        t_ext = getattr(ac, "t_last_ext_pose", None)
        if t_ext is None:
            items.append(("mocap", "unknown", "no EXTERNAL_POSE seen yet"))
        elif now - t_ext < _MOCAP_STALE_S:
            items.append(("mocap", "ok", "EXTERNAL_POSE flowing"))
        else:
            items.append(("mocap", "bad", f"EXTERNAL_POSE lost {now - t_ext:.0f}s ago"))

        # RC: last rc_status reported by the drone (arming needs RC OK)
        rc = getattr(ac, "rc_status", None)
        arming = getattr(ac, "arming_status", None)
        arming_name = (_ARMING_NAMES[arming]
                       if arming is not None and 0 <= arming < len(_ARMING_NAMES)
                       else "?")
        if rc is None:
            items.append(("RC", "unknown", "no ROTORCRAFT_STATUS seen yet"))
        else:
            state = {0: "ok", 1: "warn"}.get(rc, "bad")
            items.append(("RC", state,
                          f"rc {_RC_STATUS_NAMES.get(rc, rc)}, arming {arming_name}"))

        # link: is the telemetry downlink alive
        t_status = getattr(ac, "t_last_status", None)
        if t_status is None:
            items.append(("link", "unknown", "no ROTORCRAFT_STATUS seen yet"))
        elif now - t_status < _STATUS_STALE_S:
            items.append(("link", "ok", "telemetry alive"))
        else:
            items.append(("link", "bad", f"telemetry lost {now - t_status:.0f}s ago"))

        # battery: pack voltage against this drone's land-soon / land-now
        # thresholds (from its airframe, see battery_limits.py)
        v = getattr(ac, "battery_v", None)
        lim = _limits_of(ac)
        bstate = battery_state(v, lim)
        if bstate == "unknown":
            items.append(("battery", "unknown", "no ROTORCRAFT_STATUS seen yet"))
        else:
            pc, pack = lim.per_cell(v), f"{v:.1f}V pack, {lim.cells}S"
            detail = {
                "ok":   f"{pc:.2f}V/cell ok ({pack})",
                "warn": (f"{pc:.2f}V/cell low (< {lim.low_per_cell:.2f}): "
                         f"plan to land ({pack})"),
                "bad":  (f"{pc:.2f}V/cell CRITICAL (< {lim.crit_per_cell:.2f}): "
                         f"land now ({pack})"),
            }[bstate]
            items.append(("battery", bstate, detail))

        return items

    @staticmethod
    def _nav_html(ac, now):
        """Short flight state (mode · airborne) for the row, plus a full
        tooltip (block, motors) for when it's needed. Returns (html, tip)."""
        # a silent drone must not keep showing its last known state
        # (pool-reused drones would display a stale "motors ON" forever)
        t_status = getattr(ac, "t_last_status", None)
        if t_status is None or now - t_status >= _STATUS_STALE_S:
            return "—", ""

        parts = []
        mode = getattr(ac, "ap_mode", None)
        if mode is None:
            parts.append("—")
        else:
            name = (_AP_MODE_NAMES[mode] if 0 <= mode < len(_AP_MODE_NAMES)
                    else f"mode {mode}")
            if name in ("KILL", "FAILSAFE"):
                parts.append(f'<span style="color:{_BATT_CRIT_COLOR};'
                             f' font-weight:700;">{name}</span>')
            else:
                parts.append(f'<span style="color:{_VALUE};">{name}</span>')

        in_flight = getattr(ac, "ap_in_flight", None)
        if in_flight is not None:
            parts.append(f'<span style="color:{_VALUE};">airborne</span>'
                         if in_flight else "on ground")
        html = " · ".join(parts)

        tip = []
        cur_block = getattr(ac, "cur_block", None)
        if cur_block is not None:
            names = getattr(getattr(ac, "blocks", None), "names", [])
            block = (names[cur_block] if 0 <= cur_block < len(names)
                     else f"block {cur_block}")
            tip.append(f"block: {block}")
        motors = getattr(ac, "ap_motors_on", None)
        if motors is not None:
            tip.append("motors ON" if motors else "motors off")
        return html, "  ·  ".join(tip)

    def set_traj_name(self, idx, name):
        if 0 <= idx < len(self.ids):
            self.rows[self.ids[idx]].set_traj_name(name)

    def update_from_fd(self, fd):
        now = time.time()

        ft = None
        if getattr(fd.status, "name", "") == "GUIDING":
            m, s = divmod(int(now - fd.t0), 60)
            ft = f"{m:02d}:{s:02d}"
        self._set_flight_time(ft)

        for _id in self.ids:
            ac = fd.acs.get(_id)
            row = self.rows[_id]
            if ac is None:
                continue

            batt = getattr(ac, "battery_v", None)
            lim = _limits_of(ac)
            row.set_checklist(self._checklist_items(ac, now))
            row.set_nav(*self._nav_html(ac, now))

            if not ac.vehicle_traj:                 # aucune pose recue encore
                row.set_values(None, None, batt, lim)
                self._prev.pop(_id, None)
                continue

            pos = np.asarray(ac.T[:3, 3], dtype=float)
            alt = pos[2]

            # vitesse: derivee numerique lissee de la position (estimation)
            v_est = self._speed.get(_id, 0.0)
            prev = self._prev.get(_id)
            if prev is not None:
                dt = now - prev[1]
                if dt > 1e-3:
                    inst = np.linalg.norm(pos - prev[0]) / dt
                    v_est = _SPEED_ALPHA * inst + (1 - _SPEED_ALPHA) * v_est
                    self._speed[_id] = v_est
            self._prev[_id] = (pos, now)

            row.set_values(alt, v_est, batt, lim)
