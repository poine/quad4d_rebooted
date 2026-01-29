#!/bin/env python3
import sys, time, signal
import numpy as np

from PySide6.QtGui import QAction, QIcon, QCursor, Qt, QColor, QPalette
from PySide6.QtWidgets import QMainWindow, QMenu, QApplication, QWidget, QVBoxLayout, QLabel
import PySide6.QtCore


import model as cnf_mod
import main_window as cnf_mw

import traj_guided
#from pprzlink.message import PprzMessage

class Application(QApplication):
    def __init__(self):
        super().__init__(sys.argv)
        self.model = cnf_mod.Model()
        self.window = cnf_mw.MainWindow(self.model, self.on_dyn_point_moved, self)
        self.timer = PySide6.QtCore.QTimer(self)
        self.timer.timeout.connect(self.periodic)
        self.window.show()

    def load_from_factory(self, which):
        self.model.load_from_factory(which)
        self.window.update_plot(self.model)
        
    def on_dyn_point_moved(self):
        marker_pos = np.array([p.c.center for p in self.window.space_idx_chronogram_window.si_chrono_view.dyn_points])
        self.model.set_dynamics(marker_pos)
        self.window.update_plot(self.model)
        print('on dyn points moved')

    def on_geom_waypoint_moved(self, wps):
        self.model.set_waypoints(wps)
        self.window.update_plot(self.model)
        print('on geom waypoint moved')
        
    def toggle_play(self, s=None):
        print("toggle play", s)
        if self.timer.isActive():
            self.timer.stop()
        else:
            self.timer.start(100)
            self.timer_t0 = time.time()

    def periodic(self):
        elapsed = time.time() - self.timer_t0 
        if elapsed >= self.model.trajectory.duration:
            self.timer_t0 += self.model.trajectory.duration
            elapsed -= self.model.trajectory.duration
        Y = self.model.get_output_at(elapsed)
        pos, vel, rmat = self.model.get_state_at(elapsed)
        self.window.draw_current_pose(elapsed, Y, rmat)

    def export_to_csv(self, filename, sample_time=0.05):
         print(f'exporting to {filename}')
         time, Y = self.model.sample_output()
         np.savetxt(filename, np.hstack((time[:,np.newaxis], Y[:,:,0], Y[:,:,1], Y[:,:,2])), delimiter=',',
                    header='time,x(N),y(E),z(D),psi,xd(N),yd(E),zd(D),psid,xdd(N),ydd(E),zdd(D),psidd')

        
def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = Application()
    sys.exit(app.exec())

if __name__=="__main__":
    main()


