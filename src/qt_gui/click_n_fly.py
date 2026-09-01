#!/usr/bin/env python3
#
# Click'n Fly: operator interface for multi-drone shows in the aviary.
#
# Conflicts during the show are resolved by LAMBDA SCHEDULING: instead of
# delaying whole trajectories at start, the safety check pauses lower-priority
# drones ON their path just before each conflict zone (spatial_deconfliction.py).
# Geometry is never touched. Transits pick their own mode automatically
# (sequencing, staggered departures or height layering), see start_transit.
#

import sys, time, signal, logging, argparse
import numpy as np
from enum import Enum

from PySide6.QtWidgets import QApplication, QMainWindow, QDialog, QMessageBox
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QGuiApplication
QGuiApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts, True)
# https://www.pythonguis.com/tutorials/multithreading-pyside6-applications-qthreadpool/

import misc_utils as mu
import view_three_d as vtd, model
import scenarios as cnf_scen
import pat3.algebra as p_al

from pprz_connect import PprzConnect
from pprzlink.message import PprzMessage
from settings import PprzSettingsManager
from guided_mode import GuidedMode
from operator_window import OperatorWindow
from scenario_picker import ScenarioPickerDialog
from drones_panel import battery_state
import battery_limits
import flight_blocks as fb
import spatial_deconfliction as sd

logger = logging.getLogger(__name__)

DIST_TO_START_THRESHOLD = 0.5

STANDBY_POINTS = [
    (-1.0, -1.0, 1.2),
    ( 1.0, -1.0, 1.2),
    ( 0.0,  1.5, 1.2),
]
STANDBY_AIRBORNE_ALT = 0.4 
GUIDED_AP_MODE = 19 
TRANSIT_ARRIVE = 0.4     #m, target arival threshold

TRANSIT_LAYER_BASE = 1.2   # m, lowest transit layer
TRANSIT_LAYER_DZ   = 1.6   # m, vertical spacing between layers (> ~1.3 m margin)
TRANSIT_SEQ_MARGIN = 1.3   # m, min clearance to auto-pick the prettier 'sequence'


# A drone that cannot settle within TRANSIT_ARRIVE holds the whole show back
# with nothing on screen to say so. After this long, report how short each one
# is and offer the operator to start anyway (see Application.periodic).
TRANSIT_WAIT_WARN = 15.    # s, before offering to start the show as it stands

def _point_seg_dist(p, a, b):
    """Shortest distance from point p to the segment [a, b] (3D)."""
    p, a, b = (np.asarray(v, dtype=float) for v in (p, a, b))
    ab = b - a
    L2 = float(ab @ ab)
    if L2 < 1e-9:
        return float(np.linalg.norm(p - a))
    u = float(np.clip((p - a) @ ab / L2, 0.0, 1.0))
    return float(np.linalg.norm(p - (a + u * ab)))

# Speed cap: a trajectory whose peak speed exceeds TARGET_MAX_SPEED is slowed
# (SlowedTraj) just enough to bring its peak down to it, so fast trajectories
# track well on the real hardware WITHOUT making the already-slow ones drag.
# Adaptive per trajectory -- one knob. Airframe REF_MAX_SPEED is 2.5 m/s.
TARGET_MAX_SPEED = 1.5   # m/s

# Live show-speed factor (HMI slider): the show clock advances at this factor
# and the feedforward is rescaled accordingly, uniformly for every drone (keeps
# sync and deconfliction). The factor is eased toward the slider value over
# ~SPEED_SMOOTH_TAU so a slider move never jerks the drones.
SPEED_SMOOTH_TAU = 1.0   # s

SCHED_SAFETY_DIST   = 1.0   # m, pairwise distance defining a conflict
SCHED_STANDOFF      = 0.15  # m, extra buffer over safety for the parked drone: it waits as close to the conflict as safety+standoff allows (staged at the gate, not back at its corner)
SCHED_RESUME_MARGIN = 0.5   # s, extra wait after the other drone has cleared


class SlowedTraj:
    """Wrap a trajectory to traverse it k times slower: same geometry, but
    duration * k, and the n-th time-derivative divided by k**n (so velocity
    /k, accel /k^2 ...). Keeps the flat-output feedforward consistent, which
    is what lets the drone track it. Other attributes fall through."""
    def __init__(self, traj, factor):
        self.traj = traj
        self.k = float(factor)
        self.duration = traj.duration * self.k
        self.name = getattr(traj, 'name', 'slowed')
        self.desc = (getattr(traj, 'desc', '') + f' [x{self.k:g} slower]').strip()
 
    def get(self, t):
        Y = np.array(self.traj.get(t / self.k), dtype=float, copy=True)
        for d in range(1, Y.shape[1]):     # scale each time-derivative
            Y[:, d] /= self.k ** d
        return Y
 
    def __getattr__(self, name):
        return getattr(self.traj, name)
 
 
def apply_slowdown(model, target_v=TARGET_MAX_SPEED, npts=200):
    """Cap each trajectory's PEAK speed at target_v: one faster than that is
    slowed just enough (SlowedTraj) to bring its peak down to target_v;
    slower ones are left untouched. Adaptive, so fast trajectories track well
    without the already-slow ones dragging."""
    for i in range(model.trajectory_nb()):
        traj = model.get_trajectory(i)
        vmax = max(float(np.linalg.norm(traj.get(t)[:3, 1]))
                   for t in np.linspace(0., traj.duration, npts))
        if vmax > target_v + 1e-6:
            model.set_trajectory(SlowedTraj(traj, vmax / target_v), i)



class MainWindow(QMainWindow):
    def __init__(self, model, ids, controller):
        super().__init__()
        self.controller = controller
        self.resize(1280,900)
        self.tdw = vtd.ThreeDWidget()
        for i in range(len(ids)):
            # spectator view: the real drone plus its reference ("ghost")
            # trajectory path; the ghost quad stays hidden
            self.tdw.display_new_trajectory(model, i, show_details=False, show_quad=True,
                                            show_ref_quad=False, show_ref_traj=True)
        self.setCentralWidget(self.tdw)

    def set_quad_pose(self, T, i): self.tdw.set_quad_pose(T, i)
    def set_ref_pose(self, T, i): self.tdw.set_ref_pose(T, i)
    def update_vehicle_traj(self, vehicle_traj, i): self.tdw.update_vehicle_traj(vehicle_traj, i)

    def closeEvent(self, event):
        logger.debug('x button clicked')
        self.controller.on_quit()
        event.accept()

# class Worker(QRunnable):
#     def __init__(self, trajectory, traj_manager, dt=1./10):
#         super().__init__()
        
#     @Slot()
#     def run(self):
#         time.sleep(1)
#         print('worker exiting')


DroneStatus = Enum('DroneStatus', [('UNKNOWN', 1), ('CONNECTED', 2), ('READY', 3), ('CRUISING', 4), ('ARRIVED', 5)])
class Drone:
    def __init__(self):
        self.T, self.Tref = [np.eye(4)]*2
        self.Y, self.Yref = [np.zeros((4,5))]*2
        self.vehicle_traj = []
        self.vehicle_traj_max_len, self.vehicle_traj_increment = 1000, 100
        # maybe? https://github.com/eric-wieser/numpy_ringbuffer/blob/master/numpy_ringbuffer/__init__.py
        self.status = DroneStatus.UNKNOWN
        self.battery_v = None
        self.batt_limits = None    #set on connect, from the airframe
        self.link_down = False   # True once an Ivy send has failed (bus gone)
        self.standby_point = None   # fixed ENU staging point (None -> traj start)
        # pre-flight checklist inputs (see drones_panel)
        self.t_last_ext_pose = None  # last EXTERNAL_POSE seen (mocap uplink)
        self.t_last_status = None    # last ROTORCRAFT_STATUS (downlink alive)
        self.rc_status = None        # 0 OK, 1 LOST, 2 REALLY_LOST
        self.arming_status = None
        self.blocks = None           # flight plan block table, set on connect
        # flight plan / autopilot state shown in the drones panel
        self.ap_mode = None          # index into the ap_mode values (13 NAV, 19 GUIDED)
        self.ap_motors_on = None
        self.ap_in_flight = None
        self.cur_block = None        # current flight plan block (ground NAV_STATUS)

    def connect(self, conf, ivy):
        self.conf = conf
        self.settings = PprzSettingsManager(conf.settings, conf.id, ivy)
        self.guided = GuidedMode(ivy)
        self.ivy = ivy
        self.blocks = fb.FlightPlanBlocks(conf)
        # this drone's own battery thresholds, from its airframe (BAT section)
        self.batt_limits = battery_limits.from_airframe(conf)
        self.status = DroneStatus.CONNECTED
      
    def _send(self, action):
        """Run an Ivy-sending command; if the bus is gone, degrade
        gracefully (log once) instead of spamming tracebacks. Returns
        True on success, False if the Ivy link is down."""
        try:
            action()
        except RuntimeError as e:
            if not self.link_down:
                _id = getattr(self.conf, 'id', '?')
                logger.warning(f'aircraft {_id}: Ivy link down, command dropped ({e})')
            self.link_down = True
            return False
        except (AttributeError, KeyError) as e:
            # a setting name missing from THIS aircraft's settings (e.g.
            # 'auto2' absent on some airframes): don't let it crash the
            # periodic loop / handlers, degrade gracefully like a link drop
            _id = getattr(self.conf, 'id', '?')
            logger.warning(f'aircraft {_id}: command dropped, setting unavailable ({e})')
            return False
        self.link_down = False
        return True
        
    def take_control(self):
        if self.status == DroneStatus.UNKNOWN:
            return True
        def _do():
            self.settings['auto2'] = 'Guided'
            self.guided.move_at_ned_vel(self.conf.id) # set zero speed
        return self._send(_do)

    def release(self):
        #self.settings['auto2'] = 'Nav'
        if self.status == DroneStatus.UNKNOWN:
            return True
        return self._send(lambda: self.settings.__setitem__('auto2', 'Nav'))

    def set_pose(self, T):
        self.T=T
        self.vehicle_traj.append(mu.pos_of_T(T)) # FIXME: limit size
        if len(self.vehicle_traj) > self.vehicle_traj_max_len:
            self.vehicle_traj = self.vehicle_traj[self.vehicle_traj_increment:]

        
    def set_ref(self, Tref, Yref): self.Tref, self.Yref = Tref, Yref
    def goto_ref(self):
        return self._send(lambda: self.guided.goto_enu(self.conf.id, *self.Yref[:,0]))
    def goto_point(self, enu):
        if self.status == DroneStatus.UNKNOWN:
            return True
        return self._send(lambda: self.guided.goto_enu(self.conf.id, *enu))
      
    def go_standby(self):
        """Fly to this drone's fixed standby point, or its trajectory
        start if none is defined."""
        if self.standby_point is not None:
            return self.goto_point(self.standby_point)
        return self.goto_ref()
      
    def follow_ref(self):
        Y = mu.Yenu2ned(self.Yref)
        return self._send(lambda: self.guided.set_full_ned(self.conf.id,
                                 Y[0,0], Y[1,0], Y[2,0],
                                 Y[0,1], Y[1,1], Y[2,1],
                                 Y[0,2], Y[1,2], Y[2,2],
                                 Y[3,0]))
        
    def dist_to_ref(self):
        return np.linalg.norm(mu.pos_of_T(self.T)-mu.pos_of_T(self.Tref))

    def _jump_to_block(self, candidates, what):
        if self.status == DroneStatus.UNKNOWN or self.blocks is None:
            logger.warning(f'{what}: drone not connected, ignored')
            return False
        block_id = self.blocks.find(candidates)
        if block_id is None:
            logger.warning(f"aircraft {self.conf.id}: no '{what}' block in "
                           f'flight plan (has: {self.blocks.names})')
            return False
        return self._send(lambda: fb.jump_to_block(self.ivy, self.conf.id, block_id))

    def _set_kill(self, kill):
        """Set/clear kill_throttle. Tries the label values first (the
        settings manager maps them, cf. auto2='Guided'), then numeric,
        under the setting names seen in rotorcraft settings files."""
        last_err = None
        for name, value in (('kill_throttle', 'ON' if kill else 'OFF'),
                            ('kill_throttle', 1 if kill else 0),
                            ('autopilot.kill_throttle', 1 if kill else 0)):
            try:
                self.settings[name] = value
                return True
            except Exception as e:
                last_err = e
        logger.warning(f'aircraft {self.conf.id}: kill_throttle={kill} '
                       f'failed ({last_err})')
        return False

    def start_motors(self):
        # un-kill first: a landed (flight plan kills throttle at
        # touchdown) or killed drone would ignore the block jump. 
        if self.status != DroneStatus.UNKNOWN:
            self._set_kill(False)
        return self._jump_to_block(fb.MOTORS_CANDIDATES, 'start motors')

    def takeoff(self):      return self._jump_to_block(fb.TAKEOFF_CANDIDATES, 'takeoff')
    def land(self):         return self._jump_to_block(fb.LAND_CANDIDATES, 'land')

    def hold_position(self):
        """Freeze in a Guided hover where the drone currently is."""
        if self.status == DroneStatus.UNKNOWN:
            return True
        return self._send(lambda: self.guided.move_at_ned_vel(self.conf.id))

    def kill(self):
        #Cut the motors 
        if self.status == DroneStatus.UNKNOWN:
            return False
        return self._set_kill(True)

FDStatus = Enum('FDStatus', [('STAGING', 1), ('GETTING_READY', 2), ('GUIDING', 3), ('FINISHED', 4), ('RETURNING', 5)])
class FlightDirector:
    def __init__(self, trajectories, ids):
        self.trajectories = trajectories
        self.pprz_connect = PprzConnect(notify=self.on_pprz_connect)
        self.pprz_connect.ivy.subscribe(self.on_pprz_flight_param, PprzMessage("telemetry", "ROTORCRAFT_FP"))
        self.pprz_connect.ivy.subscribe(self.on_pprz_external_pose, PprzMessage("datalink", "EXTERNAL_POSE"))
        self.pprz_connect.ivy.subscribe(self.on_pprz_status, PprzMessage("telemetry", "ROTORCRAFT_STATUS"))
        self.pprz_connect.ivy.subscribe(self.on_pprz_nav_status, PprzMessage("ground", "NAV_STATUS"))
        self.status = FDStatus.STAGING
        self.ids, self.acs = ids, {}
        for _id in self.ids:
            self.acs[_id] = Drone()
        # Persistent pool: keep a strong reference to every Drone ever created,
        # even ids not in the current scenario. Dropping a Drone would let
        # Python garbage-collect its Ivy-bound settings/guided objects, which
        # tears down the shared Ivy bus ("Ivy server not running!") after a
        # scenario switch that removes a drone.
        self.drone_pool = dict(self.acs)
        self.known_confs = {}  # every conf ever seen, even for ids not currently in acs
        self.t0 = 0.
        self.duree_du_show = self.trajectories.trajectory_duration()  #POur avoir la durée du show
        self.show_t = 0.          # accumulated show-time (advances during GUIDING)
        self.speed_target = 1.0   # global speed factor requested from the HMI
        self.speed_s = 1.0        # eased factor actually applied (avoids jerk)
        self.t_ready0 = None      # when the transit to the starts began
        
    def on_pprz_external_pose(self, sender, msg):
        #print(sender, msg)
        #e="enu_x"     type="float" unit="m">ENU x position in vision frame</field>
        #<field name="enu_y"     type="float" unit="m">ENU y position in vision frame</field>
        #<field name="enu_z"
        #breakpoint() 
        pos_enu = [msg[_c] for _c in ['enu_x', 'enu_y', 'enu_z']]
        #x = msg['enu_x']
        #y = msg['enu_z']
        #z = msg['enu_y']
        #pos_enu = [x, y, -z]
        quat = np.array([msg[_c] for _c in ['body_qi', 'body_qx', 'body_qy', 'body_qz']])
        #breakpoint()
        rmat_enu2flu = p_al.rmat_of_quat(quat)
        T = np.eye(4); T[:3,3] = pos_enu;  T[:3,:3] = rmat_enu2flu
        try:
            ac = self.acs[int(sender)]
            #print("pose bine mis à jour")
        except KeyError: return # unknown
        ac.pose_source = 'external'
        ac.t_last_ext_pose = time.time()
        ac.set_pose(T)
      
        
    def run(self, dt=0.05): # for now called from GUI thread, maybe use our own thread?
        if self.status == FDStatus.STAGING or self.status == FDStatus.GETTING_READY:
            elapsed = 0.
        elif self.status == FDStatus.GUIDING:
            # advance the show clock at the (eased) global speed factor: a live
            # speed change never jumps the reference position, only the rate
            # going forward. Uniform for every drone -> sync & deconfliction
            # preserved. Feedforward is rescaled below to match the new rate.
            self.speed_s += (self.speed_target - self.speed_s) * min(dt / SPEED_SMOOTH_TAU, 1.0)
            self.show_t += self.speed_s * dt
            elapsed = self.show_t % self.duree_du_show
        else:
            elapsed = time.time() - self.t0

        for idx_traj, id_ac in enumerate(self.ids): # compute reference pose
            traj = self.trajectories.get_trajectory(idx_traj)
            # each trajectory loops on its OWN period: the show wraps on the
            # longest one, and shorter trajectories used to teleport mid-lap at the global wrap
            t_traj = elapsed % traj.duration if traj.duration > 0 else elapsed
            Yref = traj.get(t_traj)
            if self.status == FDStatus.GUIDING and self.speed_s != 1.0:
                # d-th time-derivative scales as speed_s**d (vel x s, accel x s^2...)
                Yref = np.array(Yref, dtype=float, copy=True)
                for d in range(1, Yref.shape[1]):
                    Yref[:, d] *= self.speed_s ** d
            Tref = np.eye(4); Tref[:3,3] = Yref[:3,0]
            self.acs[id_ac].set_ref(Tref, Yref)
        drone_status = [self.acs[_id].status for _id in self.ids]
        if self.status == FDStatus.STAGING:
            if np.all([s == DroneStatus.CONNECTED for s in drone_status]):
                self.status = FDStatus.GETTING_READY
                self.t_ready0 = time.time()
                # deconflicted transit (staggered departures) to the starts
                self.start_transit({i: tuple(float(v) for v in self.acs[i].Yref[:3, 0])
                                    for i in self.ids})
                logger.info('all connected -> deconflicted transit to start')
        elif self.status == FDStatus.GETTING_READY:
            if self.transit_step():
                self.begin_show()
                #self.duree_du_show = self.trajectories.trajectory_duration()
                #self.status, self.t0 = FDStatus.GUIDING, time.time()
                #self.show_t, self.speed_s = 0., self.speed_target  # start fresh
                logger.info('all drones arrived to start, starting the show')
                
        elif self.status == FDStatus.GUIDING:
            for i in self.ids:
                self.acs[i].follow_ref()
        elif self.status == FDStatus.RETURNING:
            if self.transit_step():
                self.status = FDStatus.FINISHED
                logger.info('all drones back at standby')
        elif self.status == FDStatus.FINISHED:
            pass


    def _seq_is_safe(self, start, targets, margin=TRANSIT_SEQ_MARGIN):
        """True if the one-by-one 'sequence' transit is collision-free: for each
        drone's straight move, no other (parked) drone is within `margin` of its
        path. Drones that already moved are parked at their target, the others at
        their start."""
        order = list(self.ids)
        for k, i in enumerate(order):
            a, b = start[i], targets[i]
            for kk, j in enumerate(order):
                if j == i:
                    continue
                parked = targets[j] if kk < k else start[j]
                if _point_seg_dist(parked, a, b) < margin:
                    return False
        return True
 
    def _schedule_delays(self, start, targets, margin=TRANSIT_SEQ_MARGIN * 1.15,
                         speed=1.0, dt=0.05, max_delay=12.0):
        """Lambda-scheduling by staggered departure (priority = drone order):
        each drone waits at its start until it can fly STRAIGHT to its target
        without coming within `margin` of a higher-priority drone's (already
        scheduled) transit. Returns ({ac_id: delay_s}, all_cleared).
 
        The check uses a 15% buffer over the safety margin, and a fine time step:
        planning exactly at the margin leaves no room for the real tracking error,
        and a coarse step can step over a brief close approach."""
        P = {i: np.asarray(start[i], dtype=float) for i in self.ids}
        Tg = {i: np.asarray(targets[i], dtype=float) for i in self.ids}
        dur = {i: max(float(np.linalg.norm(Tg[i] - P[i])) / speed, 1e-3) for i in self.ids}
 
        def pos(i, t, d):                       # drone i at time t, departing at delay d
            if t <= d:
                return P[i]
            return P[i] + (Tg[i] - P[i]) * min((t - d) / dur[i], 1.0)
 
        ids = list(self.ids)
        delays, cleared = {}, True
        for k, i in enumerate(ids):
            chosen = None
            for d in np.arange(0.0, max_delay + dt, dt):
                horizon = max([d + dur[i]] + [delays[j] + dur[j] for j in ids[:k]])
                clash = any(np.linalg.norm(pos(i, t, d) - pos(j, t, delays[j])) < margin
                            for t in np.arange(0.0, horizon + dt, dt) for j in ids[:k])
                if not clash:
                    chosen = float(d)
                    break
            if chosen is None:                  # could not clear within max_delay
                chosen, cleared = float(max_delay), False
            delays[i] = chosen
        return delays, cleared
 
    def start_transit(self, targets):
        """Begin the transit to `targets` {ac_id:(x,y,z)}, auto-picking the
        simplest safe mode: 'sequence', else 'lambda', else 'layered'."""
        targets = {i: tuple(float(v) for v in targets[i]) for i in self.ids}
        start = {i: tuple(float(v) for v in self.acs[i].T[:3, 3]) for i in self.ids}
        self._transit = {'targets': targets, 'start': start}
        if self._seq_is_safe(start, targets):
            self._transit['mode'] = 'sequence'
            self._transit['order'] = list(self.ids)   # priority = drone order
            self._transit['active'] = 0
            logger.debug("transit (sequence): " + " -> ".join(str(i) for i in self.ids))
        else:
            delays, cleared = self._schedule_delays(start, targets)
            if cleared:
                self._transit['mode'] = 'lambda'
                self._transit['delays'] = delays
                self._transit['start_t'] = time.time()
                logger.debug("transit (lambda): " + ", ".join(
                    f"{i}:{delays[i]:.1f}s" for i in self.ids))
            else:
                order = sorted(self.ids, key=lambda i: (round(targets[i][2], 3), start[i][2]))
                self._transit['mode'] = 'layered'
                self._transit['layers'] = {i: TRANSIT_LAYER_BASE + k * TRANSIT_LAYER_DZ
                                           for k, i in enumerate(order)}
                self._transit['phase'] = 'rise'
                logger.debug("transit (height-layered): " + ", ".join(
                    f"{i}@{self._transit['layers'][i]:.1f}m" for i in self.ids))
        for i in self.ids:
            self.acs[i].take_control()   # ensure Guided once
        self._transit_send()
 
    def _transit_waypoint(self, i):
        """Point drone i is currently commanded to, given the mode/phase."""
        t = self._transit
        if t['mode'] == 'sequence':
            # moved & currently-moving drones head to their target; the ones not
            # yet up stay parked at their start
            idx = t['order'].index(i)
            return t['targets'][i] if idx <= t['active'] else t['start'][i]
        if t['mode'] == 'lambda':
            # hold at start until my scheduled departure, then straight to target
            if (time.time() - t['start_t']) < t['delays'][i]:
                return t['start'][i]
            return t['targets'][i]
        # 'layered'
        sx, sy, _ = t['start'][i]
        tx, ty, tz = t['targets'][i]
        lz = t['layers'][i]
        phase = t['phase']
        if phase == 'rise':      return (sx, sy, lz)   # climb to layer, xy fixed
        if phase == 'translate': return (tx, ty, lz)   # move above target at layer
        return (tx, ty, tz)                            # descend to target

    def begin_show(self):
        """Leave the transit and start guiding, from wherever the drones are."""
        self._transit = None
        self.t_ready0 = None
        self.duree_du_show = self.trajectories.trajectory_duration()
        self.status, self.t0 = FDStatus.GUIDING, time.time()
        self.show_t, self.speed_s = 0., self.speed_target  # start fresh
 
    def transit_shortfalls(self):
        """How far each drone still is from its scenario start, in metres.
        once everyone is within TRANSIT_ARRIVE, and None while any drone is
        still on an intermediate leg -- climbing to its layer, or parked
        waiting its turn. Starting the show then would not mean "a drone is a
        little short": it would mean a drone metres above or away from where
        the choreography expects it, so the caller must not offer to force."""
        t = getattr(self, '_transit', None)
        if t is None:
            return {}
        out = {}
        for i in self.ids:
            tgt = np.asarray(t['targets'][i], dtype=float)
            here = np.asarray(self._transit_waypoint(i), dtype=float)
            if np.linalg.norm(here - tgt) > 1e-6:
                return None            # still heading somewhere else
            d = float(np.linalg.norm(np.asarray(self.acs[i].T[:3, 3], dtype=float) - tgt))
            if d >= TRANSIT_ARRIVE:
                out[i] = d
        return out
    
    def _transit_send(self):
        for i in self.ids:
            self.acs[i].goto_point(self._transit_waypoint(i))
 
    def _transit_reached(self, i):
        return (np.linalg.norm(np.asarray(self.acs[i].T[:3, 3], dtype=float)
                - np.asarray(self._transit_waypoint(i), dtype=float)) < TRANSIT_ARRIVE)
 
    _TRANSIT_PHASES = ('rise', 'translate', 'descend', 'done')
 
    def transit_step(self):
        """Drive the active transit; True once every drone reached its target."""
        t = getattr(self, '_transit', None)
        if t is None:
            return True
        self._transit_send()
        if t['mode'] == 'sequence':
            # let the next drone depart once the active one has arrived
            if self._transit_reached(t['order'][t['active']]):
                t['active'] += 1
                if t['active'] >= len(t['order']):
                    self._transit = None
                    return True
                logger.debug(f"transit: drone {t['order'][t['active']]} departs")
            return False
        if t['mode'] == 'lambda':
            # done once every drone sits on its target (delays handled in the
            # waypoint), i.e. the last staggered departure has arrived
            if all(np.linalg.norm(np.asarray(self.acs[i].T[:3, 3], dtype=float)
                   - np.asarray(t['targets'][i], dtype=float)) < TRANSIT_ARRIVE
                   for i in self.ids):
                self._transit = None
                return True
            return False
        # 'layered': advance phase only when ALL drones reached the waypoint
        if all(self._transit_reached(i) for i in self.ids):
            nxt = self._TRANSIT_PHASES[self._TRANSIT_PHASES.index(t['phase']) + 1]
            if nxt == 'done':
                self._transit = None
                return True
            t['phase'] = nxt
            logger.debug(f"transit: phase -> {t['phase']}")
        return False
 
    

    def on_pprz_connect(self, conf):
        logger.debug(f'{conf.id} ({conf.name}) connected')
        self.known_confs[int(conf.id)] = conf
        if int(conf.id) in self.acs:
            self.acs[int(conf.id)].connect(conf, self.pprz_connect.ivy)
            # do NOT take control (Guided) here: in Guided the autopilot
            # ignores the flight plan, so the Start motors / Take off
            # block jumps would be dead. Drones stay in NAV until LAUNCH
            # SHOW arms Guided (on_guide_clicked).
  
    def on_pprz_flight_param(self, sender, msg):
        pos_enu = [float(msg[_c])/2**8 for _c in ['east', 'north', 'up']]
        euler_ned2frd = [float(msg[_c])/2**12 for _c in ['phi', 'theta', 'psi']]
        rmat_enu2flu = mu.rmat_enu2flu_of_euler_ned2frd(euler_ned2frd)
        T = np.eye(4); T[:3,3] = pos_enu; T[:3,:3] = rmat_enu2flu
        try:
            ac = self.acs[sender]
        except KeyError: return # unknown aircraft
        if getattr(ac, 'pose_source', None) == 'external':
            return
        ac.set_pose(T)


    def on_pprz_status(self, sender, msg):
        try:
            ac = self.acs[sender]
        except KeyError: return # unknown aircraft
        if ac.t_last_status is None:
            # first status of the run, raw: the reference to compare with
            # the GCS when the panel and the strip disagree
            logger.debug(f'first ROTORCRAFT_STATUS from {sender}: {msg}')
        ac.t_last_status = time.time()
        ac.battery_v = float(msg['vsupply'])
        try:
            ac.rc_status = int(msg['rc_status'])
            ac.arming_status = int(msg['arming_status'])
            ac.ap_mode = int(msg['ap_mode'])
            motors = int(msg['ap_motors_on'])
            if motors != ac.ap_motors_on:
                logger.info(f'aircraft {sender}: motors '
                            f'{"ON" if motors else "off"} (raw ap_motors_on='
                            f'{msg["ap_motors_on"]}, ap_mode={msg["ap_mode"]})')
            ac.ap_motors_on = motors
            ac.ap_in_flight = int(msg['ap_in_flight'])
        except (KeyError, TypeError, ValueError):
            pass  # fields absent from this telemetry file: dots stay grey

    def on_pprz_nav_status(self, sender, msg):
        # ground-class message from the pprz server: the aircraft is a
        # field, not the Ivy sender
        try:
            ac = self.acs[int(msg['ac_id'])]
            ac.cur_block = int(msg['cur_block'])
        except (KeyError, TypeError, ValueError):
            return  # unknown aircraft or malformed message
          
    def get_acs(self): return self.acs
    def quit(self):
        # release every drone ever pooled, not just the active scenario's,
        # so none is left armed in Guided from an earlier scenario
        for drone in self.drone_pool.values():
            drone.release()
        time.sleep(0.2) # wait for message to be transmitted before closing middleware, yeah.. fuck, we need synchro with ivy
        self.pprz_connect.shutdown()


class Application(QApplication):
    def __init__(self, args):
        super().__init__(sys.argv)
        #super().__init__(args)
        self.setApplicationDisplayName("ClicknFly")
        self.setApplicationName("ClicknFly")

        picker = ScenarioPickerDialog(cnf_scen.scenarios, preselect=int(args.scen))
        if picker.exec() != QDialog.DialogCode.Accepted:
            sys.exit(0)
        self.scenario = picker.get_scenario()
        trajs, ids = self.scenario.trajs, self.scenario.ids

        self.model = model.Model()
        for traj_name in trajs:
            self.model.load_from_factory(traj_name)
        apply_slowdown(self.model)       #cap peak speed of fast trajs

        self.fd = FlightDirector(self.model, ids)
        self._assign_standby_points()
        self.window = MainWindow(self.model, ids, self)
        self.window.setWindowTitle("Click'n Fly 3 - Spectator view")
        self.window.show()

        self.operator_view = OperatorWindow(self, self.model, self.fd)
        self.operator_view.show()
        self.operator_view.log_text(
            'Deconfliction: on-path lambda scheduling (run the safety check)')

        #self.threadpool = QThreadPool()
        #self.worker = None

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.periodic)
        self.timer.start(50)
        self.t0, self.dt_control = time.time(), 0.05

        self.is_guiding = False

    def _assign_standby_points(self):
        """Give each active drone its fixed standby point, by scenario
        order. Drones beyond the configured list keep None (they fall
        back to their trajectory start)."""
        for i, ac_id in enumerate(self.fd.ids):
            pt = STANDBY_POINTS[i] if i < len(STANDBY_POINTS) else None
            self.fd.acs[ac_id].standby_point = pt

    def on_quit(self):
        if getattr(self, '_quitting', False):
            return
        self._quitting = True
        logger.debug('app on quit')
        self.fd.quit()
        self.quit()

    def _flight_plan_step(self, label, action):
        """Send a flight plan jump to every drone of the scenario."""
        failed = [str(_id) for _id in self.fd.ids
                  if not action(self.fd.acs[_id])]
        if failed:
            self.operator_view.log_text(
                f'{label}: FAILED for drone(s) {", ".join(failed)} (see terminal log)')
        else:
            self.operator_view.log_text(f'{label} sent to {len(self.fd.ids)} drone(s)')

    def on_prepare_clicked(self):
        """Single staging button (operator request): start motors, then
        take off, then move to the standby points once airborne. The
        steps are sequenced from periodic() so each waits for the
        previous one to take effect."""
        self.operator_view.log_text('PREPARE: starting motors')
        self._flight_plan_step('Start motors', lambda d: d.start_motors())
        self._prepare_state = 'motors'
        self._prepare_t = time.time()

    def _flight_plan_step(self, label, action):
        """Send a flight plan jump to every drone of the scenario."""
        failed = [str(_id) for _id in self.fd.ids
                  if not action(self.fd.acs[_id])]
        if failed:
            self.operator_view.log_text(
                f'{label}: FAILED for drone(s) {", ".join(failed)} (see terminal log)')
        else:
            self.operator_view.log_text(f'{label} sent to {len(self.fd.ids)} drone(s)')
          

    @staticmethod
    def _drone_airborne(ac):
        """Airborne if the autopilot says so (ap_in_flight, the reliable
        signal) or, as a fallback, the measured altitude is past the
        threshold."""
        if getattr(ac, 'ap_in_flight', None) == 1:
            return True
        return ac.T[2, 3] > STANDBY_AIRBORNE_ALT

    def _advance_prepare(self):
        """Drive the PREPARE sequence from periodic(): motors -> takeoff,
        then hand over to the takeoff->standby step."""
        if getattr(self, '_prepare_state', None) != 'motors':
            return
        motors = [self.fd.acs[i].ap_motors_on for i in self.fd.ids]
        all_on = bool(motors) and all(m == 1 for m in motors)
        # proceed on confirmation, or on timeout if the field is absent
        if all_on or (time.time() - self._prepare_t) > 4.0:
            self._prepare_state = None
            self.operator_view.log_text('PREPARE: taking off')
            self._flight_plan_step('Takeoff', lambda d: d.takeoff())
            self._standby_state = 'airborne'  # -> arm Guided -> goto standby

    def on_land_all_clicked(self):
        """Emergency (or end-of-show) landing: stop guiding, hand every
        drone back to its flight plan on the land block."""
        self.operator_view.log_text('LAND ALL')
        self._standby_state = None
        self._prepare_state = None
        self.is_guiding = False
        self.fd.status = FDStatus.FINISHED
        for ac_id in self.fd.ids:
            self.fd.acs[ac_id].release()  # back to NAV: the flight plan executes
        self._flight_plan_step('Land', lambda d: d.land())
        self.operator_view.button_guide.setEnabled(True)
        self.operator_view.button_stop.setEnabled(False)
        self.operator_view.set_preflight_enabled(True)

    def _offer_start_anyway(self, short):
        """Offer to start the show with drones still short of their starts.
        Only ever reached while every drone is on its final approach, so the
        error is the distance shown and nothing more. Declining waits another
        TRANSIT_WAIT_WARN and asks again, rather than going quiet."""
        detail = '\n'.join(f'    drone {i} : {d:.2f} m'
                           for i, d in sorted(short.items()))
        box = QMessageBox(self.operator_view)
        box.setIcon(QMessageBox.Warning)
        box.setWindowTitle("Lancer le show quand même ?")
        box.setText(f"{len(short)} drone(s) ne sont pas arrivés à leur point de "
                    f"départ après {int(TRANSIT_WAIT_WARN)} s.")
        box.setInformativeText(
            f"Distance restante :\n{detail}\n\n"
            "Lancer maintenant fait démarrer chacun avec cette erreur de "
            "position, qu'il rattrape sur les premiers mouvements. Voulez-vous continuer? ")
        box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        box.setDefaultButton(QMessageBox.No)
        if box.exec() == QMessageBox.Yes:
            self.operator_view.log_text(
                'show started with drones short of their starts: '
                + ', '.join(f'{i} ({d:.2f} m)' for i, d in sorted(short.items())))
            self.fd.begin_show()
        else:
            self.operator_view.log_text('start declined, still waiting')
            self.fd.t_ready0 = time.time()   # ask again in another window
            self._ready_asked = False

    def on_kill_clicked(self, ac_id):
        drone = self.fd.acs.get(ac_id) or self.fd.drone_pool.get(ac_id)
        if drone is None:
            self.operator_view.log_text(f'KILL {ac_id}: unknown drone')
            return
        if drone.kill():
            self.operator_view.log_text(f'KILL sent to drone {ac_id}')
        else:
            self.operator_view.log_text(f'KILL {ac_id}: FAILED (see terminal log)')

    def on_guide_clicked(self):
        
        def _packs(state):
            # voltages reported per cell, like the drones panel
            out = []
            for _id in self.fd.ids:
                ac = self.fd.acs[_id]
                v, lim = getattr(ac, 'battery_v', None), getattr(ac, 'batt_limits', None)
                if battery_state(v, lim) == state:
                    out.append((str(_id), lim.per_cell(v) if lim else v))
            return out
        crit, low = _packs('bad'), _packs('warn')
        if crit:
            detail = ', '.join(f'{_id} ({v:.2f}V/cell)' for _id, v in crit)
            self.operator_view.log_text(
                f'START BLOCKED: battery CRITICAL on drone(s) {detail} '
                f'- swap the pack(s) before launching')
            return
        if low:
            detail = ', '.join(f'{_id} ({v:.2f}V/cell)' for _id, v in low)
            answer = QMessageBox.question(
                self.operator_view, 'Batterie basse',
                f'Batterie basse sur le(s) drone(s) {detail}.\n\n'
                'Le vol reste possible, mais gardez-le court : le pack peut '
                'atteindre le seuil critique en vol (atterrissage automatique).\n\n'
                'Lancer le show quand même ?',
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No)      # default = do not launch
            if answer != QMessageBox.StandardButton.Yes:
                self.operator_view.log_text(
                    f'Launch cancelled (low battery on drone(s) {detail})')
                return
            self.operator_view.log_text(
                f'Launching on LOW battery ({detail}) - operator confirmed, keep it short')

        self._standby_state = None  # launching now supersedes standby staging
        self._prepare_state = None
        self.operator_view.log_text('Take off and trajectory following started')
        # arm Guided mode NOW, not at connect: before launch the drones
        # stay in NAV so the flight plan (start motors, takeoff blocks)
        # still executes. This is the single Guided entry point.
        results = [self.fd.acs[ac_id].take_control() for ac_id in self.fd.ids]
        if not all(results):
            self.operator_view.log_text(
                'WARNING: Ivy bus unavailable - is Paparazzi (server/simulator) running?')
        self.fd.status = FDStatus.STAGING
        self.is_guiding = True
        self.operator_view.button_guide.setEnabled(False)
        self.operator_view.button_stop.setEnabled(True)
        # a block jump would yank a drone out of the show: lock them out
        self.operator_view.set_preflight_enabled(False)
        
    def on_stop_clicked(self):
        # stay in Guided and return to the fixed standby points: a known,
        # repeatable formation the show can be relaunched from, or LAND
        # ALL used for a normal landing. NAV is only given back by LAND
        # ALL (so the flight plan lands) or at app exit.
        self.operator_view.log_text('Show stopped: returning to standby (deconflicted)')
        self.is_guiding = False
        targets = {ac_id: (self.fd.acs[ac_id].standby_point
                           or tuple(float(v) for v in self.fd.acs[ac_id].Yref[:3, 0]))
                   for ac_id in self.fd.ids}
        self.fd.start_transit(targets)
        self.fd.status = FDStatus.RETURNING
        self.operator_view.button_guide.setEnabled(True)
        self.operator_view.button_stop.setEnabled(False)
        self.operator_view.set_preflight_enabled(True)

    def resolve_conflicts_hook(self, safety_distance=1.0):
        """Called by the operator window's safety check: schedule away
        the conflicts by pausing drones on their paths (lambda holds),
        then propagate the new durations to the flight director."""
        ok, report = sd.resolve_conflicts_spatial(self.model,
                                                  safety_distance=SCHED_SAFETY_DIST,
                                                  standoff=SCHED_STANDOFF,
                                                  resume_margin=SCHED_RESUME_MARGIN)
        # holds stretch the trajectories: the show length must follow
        self.fd.duree_du_show = self.model.trajectory_duration()
        return ok, report

    def on_change_scenario_clicked(self):
        preselect = 0
        current_name = getattr(self.scenario, "name", None)
        for i, cls in enumerate(cnf_scen.scenarios):
            if cls.__name__ == current_name:
                preselect = i
                break
        picker = ScenarioPickerDialog(cnf_scen.scenarios, preselect=preselect,
                                      parent=self.operator_view)
        if picker.exec() == QDialog.DialogCode.Accepted:
            self._load_scenario(picker.get_scenario())

    def _load_scenario(self, scenario):
        # bring the current show to a safe stop before tearing it down
        if self.is_guiding:
            self.on_stop_clicked()

        self.scenario = scenario
        trajs, ids = scenario.trajs, scenario.ids

        new_model = model.Model()
        for traj_name in trajs:
            new_model.load_from_factory(traj_name)
        apply_slowdown(new_model)           # cap peak speed of fast trajs

        # Reuse Drone objects from the persistent pool (never dropped, so their
        # Ivy connection stays alive). A drone new to this run but already seen
        # on the Ivy bus is adopted immediately (its original on_pprz_connect
        # notification was lost since it wasn't tracked yet); a truly new id
        # starts out unconnected.
        new_acs = {}
        for _id in ids:
            drone = self.fd.drone_pool.get(_id)
            if drone is None:
                drone = Drone()
                self.fd.drone_pool[_id] = drone
                conf = self.fd.known_confs.get(_id)
                if conf is not None:
                    drone.connect(conf, self.fd.pprz_connect.ivy)
                    # no take_control here either: stay in NAV until launch
            new_acs[_id] = drone
        # drones survive scenario switches (persistent pool), so their flown
        # trace must be cleared explicitly or the old show's trail lingers
        # in the freshly rebuilt 3D views
        for drone in new_acs.values():
            drone.vehicle_traj = []
        self.fd.acs = new_acs
        self.fd.ids = ids
        self._assign_standby_points()
        self.fd.trajectories = new_model
        self.fd.duree_du_show = new_model.trajectory_duration()
        self.fd.status = FDStatus.STAGING
        self.fd.t0 = 0.

        self.model = new_model
        self.window.tdw.set_trajectories(new_model, show_details=False,
                                         show_quad=True, show_ref_quad=False,
                                         show_ref_traj=True)  # spectator: real drone + ghost trajectory
        self.operator_view.load_show(new_model, self.fd, scenario)

        name = getattr(scenario, "name", None) or scenario.__class__.__name__
        self.operator_view.log_text(f"Scenario changed: {name}")
  
    def periodic(self):
        now = time.time()
        elapsed = now - self.t0
        if elapsed >= self.dt_control:
            if self.is_guiding or self.fd.status == FDStatus.RETURNING:
                self.fd.run(self.dt_control)
            # the drones panel doubles as the pre-flight checklist, so it
            # must live before takeoff, not only while guiding
            self.operator_view.drones_panel.update_from_fd(self.fd)
            # always record (not only while guiding): staging and manual
            # moves are interesting to see in the live telemetry too
            self.operator_view.record_live_telemetry(self.fd)
            self.t0 += self.dt_control

        # drive the PREPARE staging sequence (motors -> takeoff)
        self._advance_prepare()

        # A drone that will not settle within TRANSIT_ARRIVE holds the show
        # back with nothing on screen to say so. Name it and say how short it
        # is, then let the operator start from where the drones are -- but
        # only while every one of them is already on its final approach.
        if self.fd.status == FDStatus.GETTING_READY:
            short = self.fd.transit_shortfalls()
            # say it once, then only when it actually changes: a drone stuck
            # at the same distance produces one line, not one every tick
            if short:
                prev = getattr(self, '_ready_last', None)
                moved = (prev is None or set(prev) != set(short)
                         or any(abs(short[i] - prev[i]) > 0.2 for i in short))
                if moved and (now - getattr(self, '_ready_log_t', 0.)) > 2.:
                    self._ready_log_t, self._ready_last = now, dict(short)
                    self.operator_view.log_text(
                        'waiting on ' + ', '.join(f'drone {i} ({d:.2f} m short)'
                                                  for i, d in sorted(short.items())))
            else:
                self._ready_last = None
            t_ready = self.fd.t_ready0
            if (short and t_ready is not None
                    and (now - t_ready) > TRANSIT_WAIT_WARN
                    and not getattr(self, '_ready_asked', False)):
                self._ready_asked = True
                self._offer_start_anyway(short)
        else:
            self._ready_asked = False
            self._ready_last = None

        # takeoff -> standby, in two steps so the guided goto isn't dropped:
        #   airborne: wait until every drone is actually airborne, then
        #      arm Guided (the auto2='Guided' mode switch).
        #   guided: wait until the autopilot has really switched to
        #      GUIDED before sending the goto. A goto sent while still in NAV
        #      is silently ignored by the autopilot 
        state = getattr(self, '_standby_state', None)
        if state == 'airborne':
            airborne = [self._drone_airborne(self.fd.acs[i]) for i in self.fd.ids]
            if airborne and all(airborne):
                for ac_id in self.fd.ids:
                    self.fd.acs[ac_id].take_control()  # switch to Guided
                self._standby_state = 'guided'
                self._standby_t = now
                self.operator_view.log_text('Airborne: arming Guided for standby')
        elif state == 'guided':
            # send the standby goto ONLY once the autopilot confirms GUIDED:
            # a goto sent while still in NAV is dropped, which would leave
            # the drone on the flight-plan standby instead of ours. A
            # re-launch after LAND (which handed the drone back to NAV)
            # sometimes needs a second auto2=Guided to take, so keep nudging
            # the mode switch while we wait instead of firing a blind goto.
            in_guided = all(getattr(self.fd.acs[i], 'ap_mode', None) == GUIDED_AP_MODE
                            for i in self.fd.ids)
            if in_guided:
                for ac_id in self.fd.ids:
                    self.fd.acs[ac_id].go_standby()
                self._standby_state = None
                self.operator_view.log_text('Guided confirmed: moving to standby points')
            elif (now - self._standby_t) > 6.0:
                # last resort: re-arm and send anyway rather than hang
                for ac_id in self.fd.ids:
                    self.fd.acs[ac_id].take_control()
                    self.fd.acs[ac_id].go_standby()
                self._standby_state = None
                self.operator_view.log_text(
                    'WARNING: GUIDED not confirmed after 6s; sent standby anyway')
            elif now - getattr(self, '_standby_rearm_t', 0.) > 0.5:
                self._standby_rearm_t = now
                for ac_id in self.fd.ids:
                    self.fd.acs[ac_id].take_control()  # nudge the mode switch

        # critical-battery auto-land (operator safety): a pack below the
        # land-now threshold cannot be trusted to keep flying. Fire once
        # per critical event for any airborne drone; LAND ALL hands every
        # drone back to its flight plan to land. Re-arms once the criticals
        # clear (packs swapped / drones on the ground).
        crit = [str(_id) for _id in self.fd.ids
                if self._drone_airborne(self.fd.acs[_id])
                and battery_state(getattr(self.fd.acs[_id], 'battery_v', None),
                                   getattr(self.fd.acs[_id], 'batt_limits', None)) == 'bad']
        if crit and not getattr(self, '_batt_landing', False):
            self._batt_landing = True
            self.operator_view.log_text(
                f'CRITICAL BATTERY on drone(s) {", ".join(crit)} - AUTO LANDING')
            self.on_land_all_clicked()
        elif not crit:
            self._batt_landing = False

        if self.is_guiding and self.fd.status == FDStatus.GUIDING:
            loop_elapsed = self.fd.show_t % self.fd.duree_du_show
            progress_percent= int((loop_elapsed / self.fd.duree_du_show) * 100)
            self.operator_view.show_progress(progress_percent)

        acs = self.fd.get_acs()
        for i, ac_id in enumerate(self.fd.ids):
            ac = acs[ac_id]
            self.window.set_ref_pose(ac.Tref, i)
            try:
                self.window.set_quad_pose(ac.T, i)
            except KeyError: pass # we don't know the drone pose yet
            self.window.update_vehicle_traj(np.array(ac.vehicle_traj), i)

            self.operator_view.tdw.set_ref_pose(ac.Tref, i)
            try:
                self.operator_view.tdw.set_quad_pose(ac.T, i)
            except KeyError: pass # we don't know the drone pose yet
            self.operator_view.tdw.update_vehicle_traj(np.array(ac.vehicle_traj), i)



def parse_cli():
    parser = argparse.ArgumentParser(description='ClicknFly, flight director.')
    parser.add_argument('--scen', help='the name of the scenario', default=0)
    parser.add_argument('--qt-name', help="Set the window name.", default='blaaaa', metavar="inkcut")
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='debug logging (transit plans, staging details...)')
    args = parser.parse_args()
    return args

            
def main():
    args = parse_cli()
    logging.basicConfig(level=logging.INFO)
    logger.setLevel(logging.DEBUG if args.verbose else logging.INFO)
    cnf = Application(args)
    def _quit(sig, frame):
        #print(chr(8)+chr(8),end="") # remove ^C from console... nope...
        logger.debug('Keyboard interrupt')
        cnf.on_quit()
        sys.exit()
    signal.signal(signal.SIGINT, _quit)
    cnf.exec()


if __name__ == '__main__':
    main()
