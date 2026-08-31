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
import csv
import logging
import time
from collections import deque
from itertools import combinations
import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QTimer, Qt
from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
								QComboBox, QCheckBox, QPushButton, QToolButton, 
								QMenu, QFileDialog, QMessageBox)

from drones_panel import DRONE_COLORS as _COLORS
 
logger = logging.getLogger(__name__)

_SPEED_ALPHA = 0.3    # EMA smoothing of the speed estimate (same as drones_panel)
_HISTORY_S = 1200.     # seconds of history kept in the ring buffers: a whole
                      # show stays in memory, so the CSV export covers it all
_WINDOW_S = 60.       # seconds shown in the scrolling view
_RECORD_HZ = 20       # Application.periodic() control rate, sizes the buffers

# The available plots, in display order: key -> (axis label, unit). The window
# shows all of them by default, or only the ones asked for, so the operator can
# open a single parameter full-height instead of squinting at the overview.
PLOTS = (('alt',     'altitude',                  'm'),
         ('spd',     'speed',                     'm/s'),
         ('yawrate', 'yaw rate',                  'rad/s'),
         ('dist',    'distance to reference',     'm'),
         ('mindist', 'min inter-drone distance',  'm'))
PLOT_TITLES = {k: t for k, t, _u in PLOTS}
# per-drone series (the rest is global) and their reference ("ghost") twin
_PER_DRONE = {'alt', 'spd', 'yawrate', 'dist'}
_REF_OF = {'alt': 'alt_ref', 'spd': 'spd_ref', 'yawrate': 'yawrate_ref'}


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
        self.data = {_id: {k: deque(maxlen=maxlen) for k in 
					 ('t', 'alt', 'spd', 'yawrate', 'dist',
                    	'alt_ref', 'spd_ref', 'yawrate_ref')}
                     for _id in self.ids}
        # global series: min pairwise inter-drone distance (the avoidance metric)
        self.gdata = {k: deque(maxlen=maxlen) for k in ('t', 'mindist')}
        self._prev = {}
        self._speed = {}
		self._yaw = {}      # (yaw, t) of the previous sample, for the yaw rate
        self._yawrate = {}
 
    # --- CSV export (on demand) ------------------------------------------
    # Nothing is written unless the operator asks for it from the window's
    # "..." menu, so no disk fills up in the background. What can be exported
    # is what is still in the ring buffers, hence their generous length.
    CSV_HEADER = ('t', 'drone', 'alt', 'alt_ref', 'spd', 'spd_ref',
                  'yawrate', 'yawrate_ref', 'dist', 'mindist')
 
    def export_csv(self, path):
        """Write the recorded history to `path`: one row per drone per sample,
        long format (a 'drone' column), with the global inter-drone distance
        repeated on each row of a sample. Returns the number of rows."""
        # global separation, looked up by timestamp so each row carries the
        # value measured at that instant
        gsep = dict(zip(self.gdata['t'], self.gdata['mindist']))
        rows = 0
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(self.CSV_HEADER)
            for _id in self.ids:
                d = self.data.get(_id)
                if not d:
                    continue
                for i, t in enumerate(d['t']):
                    w.writerow([round(t, 3), _id]
                               + [round(d[k][i], 4) for k in
                                  ('alt', 'alt_ref', 'spd', 'spd_ref',
                                   'yawrate', 'yawrate_ref', 'dist')]
                               + [round(gsep.get(t, float('nan')), 4)])
                    rows += 1
        logger.info(f'telemetry exported to {path} ({rows} rows)')
        return rows
 
    def summary(self):
        """Short report on the recorded history: how well each drone tracked,
        how fast it went, and how close the drones came to each other."""
        lines = []
        for _id in self.ids:
            d = self.data.get(_id)
            if not d or not d['t']:
                continue
            spd = np.asarray(d['spd'], dtype=float)
            dist = np.asarray(d['dist'], dtype=float)
            if np.all(np.isnan(dist)):
                lines.append(f'drone {_id}: top speed {np.nanmax(spd):.2f} m/s '
                             '(no tracking recorded)')
                continue
            lines.append(f'drone {_id}: tracking error mean '
                         f'{np.nanmean(dist):.2f} m / max {np.nanmax(dist):.2f} m, '
                         f'top speed {np.nanmax(spd):.2f} m/s')
        sep = np.asarray(self.gdata['mindist'], dtype=float)
        if sep.size and not np.all(np.isnan(sep)):
            lines.append(f'closest approach between drones: {np.nanmin(sep):.2f} m')
        if self.gdata['t']:
            span = self.gdata['t'][-1] - self.gdata['t'][0]
            lines.append(f'recorded window: {span:.0f} s')
        return lines

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

			# measured yaw rate: derivative of the heading read from the pose,
            # unwrapped so the +-pi crossing doesn't produce a huge spike
            yaw = np.arctan2(ac.T[1, 0], ac.T[0, 0])
            r_est = self._yawrate.get(_id, 0.)
            prev_yaw = self._yaw.get(_id)
            if prev_yaw is not None:
                dt = now - prev_yaw[1]
                if dt > 1e-3:
                    dyaw = (yaw - prev_yaw[0] + np.pi) % (2 * np.pi) - np.pi
                    r_est = _SPEED_ALPHA * (dyaw / dt) + (1 - _SPEED_ALPHA) * r_est
                    self._yawrate[_id] = r_est
            self._yaw[_id] = (yaw, now)
			
            d = self.data[_id]
            d['t'].append(t)
            d['alt'].append(pos[2])
            d['spd'].append(v_est)
			d['yawrate'].append(r_est)
            d['dist'].append(ac.dist_to_ref() if tracking else float('nan'))

			# the reference ("ghost" drone): flat output Yref, rows x/y/z/psi,
            # columns = derivative order. Only meaningful while tracking.
            if tracking:
                Yref = np.asarray(ac.Yref, dtype=float)
                d['alt_ref'].append(Yref[2, 0])
                d['spd_ref'].append(float(np.linalg.norm(Yref[:3, 1])))
                d['yawrate_ref'].append(Yref[3, 1])
            else:
                for k in ('alt_ref', 'spd_ref', 'yawrate_ref'):
                    d[k].append(float('nan'))

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

    def __init__(self, recorder, keys=None, title=None):
        super().__init__()
        self.recorder = recorder
		specs = [(k, t, u) for k, t, u in PLOTS if keys is None or k in keys]
        if not specs:                      # unknown selection: fall back to all
            specs = list(PLOTS)
        self._keys = [k for k, _t, _u in specs]
        self.setWindowTitle(title or "Click'n Fly - Live telemetry")
        self.resize(900, min(950, 220 + 180 * len(specs)))

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
		layout.addLayout(self._build_controls())
        self.glw = pg.GraphicsLayoutWidget()
        layout.addWidget(self.glw)

        self.plots = {}
        prev = None
        for row, (key, title_, unit) in enumerate(specs):
            p = self.glw.addPlot(row=row, col=0)
            p.setLabel('left', title_, units=unit)
            p.showGrid(x=True, y=True, alpha=0.3)
            if prev is not None:
                p.setXLink(prev)
            prev = p
            self.plots[key] = p
			
			# scale Y to what is actually on screen, not to the whole recorded
            # history: otherwise a past excursion flattens the live signal
            vb = p.getViewBox()
            vb.enableAutoRange(axis='y')
            vb.setAutoVisible(y=True)
            # a manual zoom/pan must not be fought by the scrolling: stop
            # following as soon as the operator grabs a plot
            vb.sigRangeChangedManually.connect(self._on_manual_range)
        self.plots[self._keys[-1]].setLabel('bottom', 'time', units='s')
        self.legend = self.plots[self._keys[0]].addLegend()
 
        # 'distance to reference' IS the tracking error, so it carries no ghost
        # curve: perfect tracking is the zero line, drawn as the guide instead
        if 'dist' in self.plots:
            self.plots['dist'].addItem(pg.InfiniteLine(
                pos=0.0, angle=0,
                pen=pg.mkPen('#8B938F', style=Qt.PenStyle.DashLine)))
 
        # global (not per-drone) curve: the avoidance metric, with the 1m
        # safety-distance line the conflict detector uses as reference
        self.mindist_curve = None
        if 'mindist' in self.plots:
            self.plots['mindist'].addItem(pg.InfiniteLine(
                pos=1.0, angle=0, pen=pg.mkPen('#F2A33C', style=Qt.PenStyle.DashLine)))
            self.mindist_curve = self.plots['mindist'].plot(
                [], [], pen=pg.mkPen('#E8ECEA', width=2), connect='finite')
 
        self._per_drone_keys = tuple(k for k in self._keys if k in _PER_DRONE)
        # measured series that also have a reference ("ghost") series, drawn
        # dashed in the same colour so the tracking error is visible directly
        self._ref_of = _REF_OF
        
		
        self.curves = {}   # (key, drone id) -> PlotDataItem
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(200)

	# --- view controls ---------------------------------------------------
    def _build_controls(self):
        """Time window / follow / reset strip above the plots."""
        row = QHBoxLayout()
        row.setContentsMargins(8, 6, 8, 0)
        row.setSpacing(8)
 
        self._window_s = _WINDOW_S
        self.combo_window = QComboBox()
        for label, span in (('30 s', 30.), ('1 min', 60.), ('2 min', 120.),
                            ('5 min', 300.), ('all', None)):
            self.combo_window.addItem(label, span)
        self.combo_window.setCurrentIndex(1)          # 1 min, the old fixed view
        self.combo_window.currentIndexChanged.connect(self._on_window_changed)
 
        self.check_follow = QCheckBox('follow')
        self.check_follow.setChecked(True)
        self.check_follow.setToolTip(
            'Keep the view on the latest data.\n'
            'Zooming or panning a plot turns this off so the view stays put.')
 
        btn_reset = QPushButton('reset zoom')
        btn_reset.setToolTip('Back to the scrolling view, auto-scaled')
        btn_reset.clicked.connect(self._reset_view)
 
        # "..." menu: the occasional actions (export, summary), kept out of the
        # way since a flight is analysed whenever, not necessarily right after
        self.btn_more = QToolButton()
        self.btn_more.setText('⋮')
        self.btn_more.setToolTip('More actions')
        self.btn_more.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)
        more = QMenu(self.btn_more)
        more.addAction('Export CSV...', self._export_csv)
        more.addAction('Flight summary...', self._show_summary)
        self.btn_more.setMenu(more)
 
        row.addWidget(QLabel('window'))
        row.addWidget(self.combo_window)
        row.addWidget(self.check_follow)
        row.addWidget(btn_reset)
        row.addStretch(1)
        row.addWidget(self.btn_more)
        return row
 
    def _export_csv(self):
        default = time.strftime('telemetry_%Y%m%d_%H%M%S.csv')
        path, _ = QFileDialog.getSaveFileName(
            self, 'Export telemetry', default, 'CSV files (*.csv);;All files (*)')
        if not path:
            return
        try:
            rows = self.recorder.export_csv(path)
        except Exception as e:
            QMessageBox.warning(self, 'Export failed', f'Could not write:\n{e}')
            return
        QMessageBox.information(
            self, 'Telemetry exported',
            f'{rows} rows written to\n{path}\n\n'
            'One row per drone per sample; the "drone" column tells them apart.')
 
    def _show_summary(self):
        lines = self.recorder.summary()
        QMessageBox.information(self, 'Flight summary',
                                '\n'.join(lines) if lines else
                                'Nothing recorded yet.')
 
    def _on_window_changed(self, _idx):
        self._window_s = self.combo_window.currentData()
        self.check_follow.setChecked(True)      # choosing a window means "follow"
 
    def _on_manual_range(self, *_args):
        # the operator took over: stop scrolling under their hands
        self.check_follow.setChecked(False)
 
    def _reset_view(self):
        for p in self.plots.values():
            vb = p.getViewBox()
            vb.enableAutoRange(axis='y')
            vb.setAutoVisible(y=True)
        self.check_follow.setChecked(True)
	

    def _rebuild_curves(self):
        for (key, _id), curve in self.curves.items():
            self.plots[key].removeItem(curve)
        try:
            self.legend.clear()
        except AttributeError:   # older pyqtgraph without LegendItem.clear
            pass
        self.curves = {}
        for i, _id in enumerate(self.recorder.ids):
			color = _COLORS[i % len(_COLORS)]
            pen = pg.mkPen(color, width=2)
            ref_pen = pg.mkPen(color, width=1, style=Qt.PenStyle.DashLine)
            legend_key = self._per_drone_keys[0] if self._per_drone_keys else None
            for key in self._per_drone_keys:
                name = f'drone {_id}' if key == legend_key else None
				self.curves[(key, _id)] = self.plots[key].plot(
                    [], [], pen=pen, connect='finite', name=name)
				ref_key = self._ref_of.get(key)
                if ref_key:
                    name = f'drone {_id} ref' if key == legend_key else None
                    self.curves[(ref_key, _id)] = self.plots[key].plot(
                        [], [], pen=ref_pen, connect='finite', name=name)

    def _refresh(self):
        if not self.isVisible():
            return
		if (self._per_drone_keys
                and {i for (_k, i) in self.curves} != set(self.recorder.ids)):
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
				ref_key = self._ref_of.get(key)
                if ref_key:
                    self.curves[(ref_key, _id)].setData(
                        t, np.array(d[ref_key], dtype=float))
        g = self.recorder.gdata
        if self.mindist_curve is not None and g['t']:
            self.mindist_curve.setData(np.array(g['t']),
                                       np.array(g['mindist'], dtype=float))

			tmax = max(tmax, g['t'][-1])
        # scroll only while following: a manual zoom/pan turns it off, so the
        # operator can study a moment without the view sliding away
        if tmax > 0. and self.check_follow.isChecked():
            span = self._window_s
            x0 = 0. if span is None else max(0., tmax - span)
            self.plots[self._keys[0]].setXRange(x0, max(1., tmax), padding=0)
