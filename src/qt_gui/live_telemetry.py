#!/usr/bin/env python3
#
# Live telemetry recording and plotting for the Click'n Fly operator HMI.
#
# Unlike view_chronograms.py (planned trajectory profiles, matplotlib),
# this shows what the drones ACTUALLY do: measured altitude, speed and
# distance to the reference over time, fed from the Ivy poses at the
# control rate. pyqtgraph is used instead of matplotlib because a full
# matplotlib redraw is too slow for a live scrolling view.
#
import time
from collections import deque
from itertools import combinations
import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import QWidget, QVBoxLayout

_SPEED_ALPHA = 0.3    # EMA smoothing of the speed estimate (same as drones_panel)
_HISTORY_S = 120.     # seconds of history kept in the ring buffers
_WINDOW_S = 60.       # seconds shown in the scrolling view
_RECORD_HZ = 20       # Application.periodic() control rate, sizes the buffers

_COLORS = ["#1E78B3", "#FF800E", "#2BA02B"]   # same palette as 3D view / drones panel

class TelemetryRecorder:
    """Ring buffers of measured drone state.

    Fed from Application.periodic() at the control rate, whether or not
    the plot window is open, so the operator can open the window mid-show
    and still see the history."""

    def __init__(self, ids):
        self.t0 = time.time()
        self.reset(ids)

    def reset(self, ids):
        maxlen = int(_HISTORY_S * _RECORD_HZ)
        self.ids = list(ids)
        self.data = {_id: {k: deque(maxlen=maxlen) for k in ('t', 'alt', 'spd', 'dist')}
                     for _id in self.ids}
        # global series: min pairwise inter-drone distance (the avoidance metric)
        self.gdata = {k: deque(maxlen=maxlen) for k in ('t', 'mindist')}
        self._prev = {}
        self._speed = {}

    def record(self, fd):
        now = time.time()
        t = now - self.t0
        # dist_to_ref is only meaningful once a reference is being tracked;
        # before that Tref is the identity and the distance would be garbage
        tracking = getattr(fd.status, 'name', '') in ('GETTING_READY', 'GUIDING')
        pos_now = {}
        for _id in self.ids:
            ac = fd.acs.get(_id)
            if ac is None or not ac.vehicle_traj:   # no pose received yet
                continue
            pos = np.asarray(ac.T[:3, 3], dtype=float)
            pos_now[_id] = pos
            # speed: smoothed numerical derivative of the measured position
            v_est = self._speed.get(_id, 0.)
            prev = self._prev.get(_id)
            if prev is not None:
                dt = now - prev[1]
                if dt > 1e-3:
                    inst = np.linalg.norm(pos - prev[0]) / dt
                    v_est = _SPEED_ALPHA * inst + (1 - _SPEED_ALPHA) * v_est
                    self._speed[_id] = v_est
            self._prev[_id] = (pos, now)
            d = self.data[_id]
            d['t'].append(t)
            d['alt'].append(pos[2])
            d['spd'].append(v_est)
            d['dist'].append(ac.dist_to_ref() if tracking else float('nan'))

        # min pairwise separation (needs at least two measured drones)
        if len(pos_now) >= 2:
            dmin = min(np.linalg.norm(a - b) for a, b in combinations(pos_now.values(), 2))
        else:
            dmin = float('nan')
        self.gdata['t'].append(t)
        self.gdata['mindist'].append(dmin)


class LiveTelemetryWindow(QWidget):
    """Scrolling oscilloscope-style view of a TelemetryRecorder.

    Pulls from the recorder on its own timer (5 Hz), decoupled from the
    control loop. Closing the window just hides it; history keeps
    accumulating in the recorder."""

    def __init__(self, recorder):
        super().__init__()
        self.recorder = recorder
        self.setWindowTitle("Click'n Fly - Live telemetry")
        self.resize(900, 780)   # 4 stacked plots

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.glw = pg.GraphicsLayoutWidget()
        layout.addWidget(self.glw)

        self.plots = {}
        prev = None
        for row, (key, title, unit) in enumerate([('alt', 'altitude', 'm'),
                                                  ('spd', 'speed', 'm/s'),
                                                  ('dist', 'distance to reference', 'm'),
                                                  ('mindist', 'min inter-drone distance', 'm')]):
            p = self.glw.addPlot(row=row, col=0)
            p.setLabel('left', title, units=unit)
            p.showGrid(x=True, y=True, alpha=0.3)
            if prev is not None:
                p.setXLink(prev)
            prev = p
            self.plots[key] = p
        self.plots['mindist'].setLabel('bottom', 'time', units='s')
        self.legend = self.plots['alt'].addLegend()

        # global (not per-drone) curve: the avoidance metric, with the 1m
        # safety-distance line the conflict detector uses as reference
        self.plots['mindist'].addItem(pg.InfiniteLine(
            pos=1.0, angle=0, pen=pg.mkPen('#F2A33C', style=Qt.PenStyle.DashLine)))
        self.mindist_curve = self.plots['mindist'].plot(
            [], [], pen=pg.mkPen('#E8ECEA', width=2), connect='finite')

        self._per_drone_keys = ('alt', 'spd', 'dist')
        self.curves = {}   # (key, drone id) -> PlotDataItem
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(200)

    def _rebuild_curves(self):
        for (key, _id), curve in self.curves.items():
            self.plots[key].removeItem(curve)
        try:
            self.legend.clear()
        except AttributeError:   # older pyqtgraph without LegendItem.clear
            pass
        self.curves = {}
        for i, _id in enumerate(self.recorder.ids):
            pen = pg.mkPen(_COLORS[i % len(_COLORS)], width=2)
            for key in self._per_drone_keys:
                name = f'drone {_id}' if key == 'alt' else None
                self.curves[(key, _id)] = self.plots[key].plot(
                    [], [], pen=pen, connect='finite', name=name)

    def _refresh(self):
        if not self.isVisible():
            return
        if {i for (_k, i) in self.curves} != set(self.recorder.ids):
            self._rebuild_curves()
        tmax = 0.
        for _id in self.recorder.ids:
            d = self.recorder.data.get(_id)
            if not d or not d['t']:
                continue
            t = np.array(d['t'])
            tmax = max(tmax, t[-1])
            for key in self._per_drone_keys:
                self.curves[(key, _id)].setData(t, np.array(d[key], dtype=float))
        g = self.recorder.gdata
        if g['t']:
            self.mindist_curve.setData(np.array(g['t']),
                                       np.array(g['mindist'], dtype=float))
        if tmax > 0.:
            self.plots['alt'].setXRange(max(0., tmax - _WINDOW_S), max(1., tmax), padding=0)
