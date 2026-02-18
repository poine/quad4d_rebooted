#!/usr/bin/env python3
import sys, time, signal, logging, yaml, argparse
import numpy as np
from enum import Enum

from PySide6.QtWidgets import (QApplication, QMainWindow, QDialog,
                               QPushButton, QProgressBar, QPlainTextEdit,
                               QVBoxLayout, QHBoxLayout)
from PySide6.QtCore import QRunnable, QThreadPool, QTimer, Slot
# https://www.pythonguis.com/tutorials/multithreading-pyside6-applications-qthreadpool/

import traj_factory, misc_utils as mu
import view_three_d as vtd, model

from pprz_connect import PprzConnect
from pprzlink.message import PprzMessage
from settings import PprzSettingsManager
from guided_mode import GuidedMode

logger = logging.getLogger(__name__)


class GuidanceDialog(QDialog):
    def __init__(self, parent, cbk):
        super().__init__(parent)
        self.setWindowTitle("Guide Quadrotor")
        self.resize(800,600)
        self.textedit_wid = QPlainTextEdit()
        self.textedit_wid.setReadOnly(True)
        self.button_guide = QPushButton("Guide")
        self.progress = QProgressBar()
        #self.progress.setTextVisible(True)
        layout = QVBoxLayout()
        layout.addWidget(self.textedit_wid)
        layout.addWidget(self.button_guide)
        layout.addWidget(self.progress)
        self.setLayout(layout)
        #self.textedit_wid.setPlainText(str('12345'))
        self.button_guide.clicked.connect(cbk)

    def show_progress(self, value): self.progress.setValue(value)
    def log_text(self, txt): self.textedit_wid.appendPlainText(txt)


class MainWindow(QMainWindow):
    def __init__(self, model, ids, controller):
        super().__init__()
        self.controller = controller
        self.resize(1280,900)
        self.tdw = vtd.ThreeDWidget()
        for i in range(len(ids)):
            self.tdw.display_new_trajectory(model, i, False); self.tdw.show_quad(True, idx=i)
        self.setCentralWidget(self.tdw)

        self.dialog = GuidanceDialog(self, controller.on_guide_clicked)
        self.dialog.show()

    def set_quad_pose(self, T, i): self.tdw.set_quad_pose(T, i)
    def set_ref_pose(self, T, i): self.tdw.set_ref_pose(T, i)
    def update_vehicle_traj(self, vehicle_traj, i): self.tdw.update_vehicle_traj(vehicle_traj, i)
    
    def show_progress(self, p): self.dialog.show_progress(p)
    def log_text(self, t): self.dialog.log_text(t)

    def closeEvent(self, event):
        logger.debug('x button clicked')
        self.controller.on_quit()
        event.accept()


class Worker(QRunnable):
    def __init__(self, trajectory, traj_manager, dt=1./10):
        super().__init__()
        
    @Slot()
    def run(self):
        time.sleep(1)
        print('worker exiting')


DroneStatus = Enum('DroneStatus', [('UNKNOWN', 1), ('CONNECTED', 2), ('READY', 3), ('CRUISING', 4), ('ARRIVED', 5)])
class Drone:
    def __init__(self, conf, ivy):
        self.conf = conf
        self.settings = PprzSettingsManager(conf.settings, conf.id, ivy)
        self.guided = GuidedMode(ivy)
        self.status = DroneStatus.CONNECTED
        self.T, self.Tref = [np.eye(4)]*2
        self.Y, self.Yref = [np.zeros((4,5))]*2
        self.vehicle_traj = []
        self.vehicle_traj_max_len, self.vehicle_traj_increment = 1000, 100
        # maybe? https://github.com/eric-wieser/numpy_ringbuffer/blob/master/numpy_ringbuffer/__init__.py

    def take_control(self):
        self.settings['auto2'] = 'Guided'
        self.guided.move_at_ned_vel(self.conf.id) # set zero speed

    def release(self):
        self.settings['auto2'] = 'Nav'

    def set_pose(self, T):
        self.T=T
        self.vehicle_traj.append(mu.pos_of_T(T)) # FIXME: limit size
        if len(self.vehicle_traj) > self.vehicle_traj_max_len:
            self.vehicle_traj = self.vehicle_traj[self.vehicle_traj_increment:]

        
    def set_ref(self, Tref, Yref): self.Tref, self.Yref = Tref, Yref
    def goto_ref(self):
        self.guided.goto_enu(self.conf.id, *self.Yref[:,0])
    def follow_ref(self):
        Y = mu.Yenu2ned(self.Yref)
        #self.guided.set_full_ned(self.conf.id, *Y[:3,0], *Y[:3,1], *Y[:3,2], Y[3,0])
        self.guided.set_full_ned(self.conf.id,
                                 Y[0,0], Y[1,0], Y[2,0],
                                 Y[0,1], Y[1,1], Y[2,1],
                                 Y[0,2], Y[1,2], Y[2,2],
                                 Y[3,0])
        
    def dist_to_ref(self):
        return np.linalg.norm(mu.pos_of_T(self.T)-mu.pos_of_T(self.Tref))

FDStatus = Enum('FDStatus', [('STAGING', 1), ('GETTING_READY', 2), ('GUIDING', 3), ('FINISHED', 4)])      
class FlightDirector:
    def __init__(self, trajectories, ids):
        self.trajectories = trajectories
        self.pprz_connect = PprzConnect(notify=self.on_pprz_connect)
        self.pprz_connect.ivy.subscribe(self.on_pprz_flight_param, PprzMessage("telemetry", "ROTORCRAFT_FP"))
        self.status = FDStatus.STAGING
        self.ids, self.acs = ids, {}
        self.t0 = 0.
        
    def run(self): # for now called from GUI thread, maybe use our own thread?
        if self.status == FDStatus.STAGING or self.status == FDStatus.GETTING_READY:
            elapsed = 0.
        else:
            elapsed = time.time() - self.t0
        for idx_traj, id_ac in enumerate(self.ids): # compute reference pose
            Yref = self.trajectories.get_trajectory(idx_traj).get(elapsed)
            Tref = np.eye(4); Tref[:3,3] = Yref[:3,0]
            self.acs[id_ac].set_ref(Tref, Yref)
        drone_status = [self.acs[_id].status for _id in self.ids]
        if self.status == FDStatus.STAGING:
            if np.all([s == DroneStatus.CONNECTED for s in drone_status]):
                self.status = FDStatus.GETTING_READY
                for i in self.ids:
                  self.acs[i].goto_ref()  
                logger.debug('all drones connected, moving them to start pos')
        elif self.status == FDStatus.GETTING_READY:
            dist_to_start = [self.acs[i].dist_to_ref() for i in self.ids]
            if np.max(dist_to_start) < 0.1:
                self.status, self.t0 = FDStatus.GUIDING, time.time()
                logger.debug('all drones arrived to start, starting the show')
        elif self.status == FDStatus.GUIDING:
            for i in self.ids:
                self.acs[i].follow_ref() 

    def on_pprz_connect(self, conf):
        logger.debug(f'{conf.id} ({conf.name}) connected')
        self.acs[int(conf.id)] = Drone(conf, self.pprz_connect.ivy)
        self.acs[int(conf.id)].take_control()
        
    def on_pprz_flight_param(self, sender, msg):
        pos_enu = [float(msg[_c])/2**8 for _c in ['east', 'north', 'up']]
        euler_ned2frd = [float(msg[_c])/2**12 for _c in ['phi', 'theta', 'psi']]
        rmat_enu2flu = mu.rmat_enu2flu_of_euler_ned2frd(euler_ned2frd)
        T = np.eye(4); T[:3,3] = pos_enu; T[:3,:3] = rmat_enu2flu
        try:
            self.acs[sender].set_pose(T)
        except KeyError: pass # unknown aircraft

    def get_acs(self): return self.acs
    def quit(self):
        for _id in self.acs:
            self.acs[_id].release()
        time.sleep(0.1) # wait for message to be transmitted before closing middleware, yeah.. fuck, we need synchro with ivy
        self.pprz_connect.shutdown()


scen1 = '''
ids: [4,5]
trajs: ["circle_with_intro1", "circle_with_intro3"]
'''
scen2 = '''
ids: [4,5]
trajs: ["space indexed oval", "space indexed oval2"]
'''
scen3 = '''
ids: [4, 5, 6]
trajs: ["circle_with_intro1", "circle_with_intro2", "circle_with_intro3"]
'''
        
class Application(QApplication):
    def __init__(self):
        super().__init__(sys.argv)
        self.setApplicationDisplayName("ClicknFly")
        #breakpoint()
        scen = yaml.safe_load(scen3)
        trajs, ids = scen['trajs'], scen['ids']

        self.model = model.Model()
        for traj_name in trajs:
            self.model.load_from_factory(traj_name)

       
        self.fd = FlightDirector(self.model, ids)
        self.window = MainWindow(self.model, ids, self)
        self.window.show()


        #self.threadpool = QThreadPool()
        #self.worker = None

        self.connected_aircraft = {}
    
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.periodic)
        self.timer.start(50)
        self.t0, self.dt_control = time.time(), 0.1


    def on_quit(self):
        logger.debug('app on quit')
        self.fd.quit()
         
    def on_guide_clicked(self):
        #self.worker = Worker(self.model.get_trajectory(), self.traj_manager)
        #self.threadpool.start(self.worker)
        self.window.log_text('started')

    def periodic(self):
        now = time.time()
        elapsed = now - self.t0
        if elapsed >= self.dt_control:
            self.fd.run()
            self.t0 += self.dt_control

        acs = self.fd.get_acs()  # FIXME... maybe encapsulate that
        for i, ac_id in enumerate(self.fd.ids):
            ac = acs[ac_id]
            self.window.set_ref_pose(ac.Tref, i)
            try:
                self.window.set_quad_pose(ac.T, i)
            except KeyError: pass # we don't know the drone pose yet
            self.window.update_vehicle_traj(np.array(ac.vehicle_traj), i)

def main():
    logging.basicConfig(level=logging.INFO)
    logger.setLevel(logging.DEBUG)
    app = Application()
    def _quit(sig, frame):
        #print(chr(8)+chr(8),end="") # remove ^C from console... nope...
        logger.debug('Keyboard interrupt')
        app.on_quit()
        sys.exit()
    signal.signal(signal.SIGINT, _quit)
    app.exec()


if __name__ == '__main__':
    main()
