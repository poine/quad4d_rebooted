#!/bin/env python3
import sys, time, signal, logging
import numpy as np

from PySide6.QtGui import QAction, QIcon, QCursor, Qt, QColor, QPalette
from PySide6.QtWidgets import QMainWindow, QMenu, QApplication, QWidget, QVBoxLayout, QLabel
import PySide6.QtCore


import model as cnf_mod, main_window as cnf_mw, view_geometry as cng_geom

#import traj_guided
#from pprzlink.message import PprzMessage

logger = logging.getLogger(__name__)

class Application(QApplication):
    def __init__(self):
        super().__init__(sys.argv)
        self.model = cnf_mod.Model(load_fact_id='space indexed race track 1')
        self.window = cnf_mw.MainWindow(self.model, self)
        self.timer = PySide6.QtCore.QTimer(self)
        self.timer.timeout.connect(self.periodic)
        self.window.show()

    def load_from_factory(self, which, idx):
        idx = self.model.load_from_factory(which, idx=idx)
        self.timer_t0 = time.time()
        self.window.display_new_trajectory(self.model, idx, show_quad=self.timer.isActive())
        
    def add_from_factory(self, which):
        print('add from factory', which)
        self.model.load_from_factory(which, replace=None)
        print(self.model.trajectories)

    def remove_drone(self, idx):
        logger.debug(f'CNF::remove_drone {idx}')
        
    def on_dyn_ctl_point_moved(self, dyn_ctl_pts):
        logger.debug('on dyn points moved')
        self.model.set_dynamics(dyn_ctl_pts)
        self.window.update_plot(self.model)

    def on_geom_waypoint_moved(self, wps):
        logger.debug('CNF::on_geom_waypoint_moved')
        self.model.set_waypoints(wps)
        self.window.update_plot(self.model)
        
    def toggle_animate(self, s=None):
        if self.timer.isActive():
            self.timer.stop()
        else:
            self.window.show_animation(True)
            self.timer_t0 = time.time()
            self.timer.start(50)#100)

    def periodic(self):
        elapsed = time.time() - self.timer_t0 
        _d = self.model.trajectory_duration()
        if elapsed >= _d:
            self.timer_t0 += _d
            elapsed -= _d
        for idx in range(self.model.trajectory_nb()):
            traj = self.model.get_trajectory(idx)
            if elapsed < traj.duration:
                Tenu2flu = self.model.get_traj_pose_at(elapsed, idx)
                self.window.draw_current_pose(elapsed, Tenu2flu, idx)
        
    def export_to_csv(self, filename, sample_time=0.05):
         logger.debug(f'exporting to {filename}')
         time, Y = self.model.sample_traj_output()
         np.savetxt(filename, np.hstack((time[:,np.newaxis], Y[:,:,0], Y[:,:,1], Y[:,:,2])), delimiter=',',
                    header='time,x(N),y(E),z(D),psi,xd(N),yd(E),zd(D),psid,xdd(N),ydd(E),zdd(D),psidd')

        
def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = Application()
    sys.exit(app.exec())

if __name__=="__main__":
    logging.basicConfig(level=logging.INFO)
    logger.setLevel(logging.DEBUG)
    cnf_mw.logger.setLevel(logging.DEBUG)
    cng_geom.logger.setLevel(logging.DEBUG)
    main()


