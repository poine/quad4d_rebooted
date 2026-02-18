#!/bin/env python3

import sys, time, signal

from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QTimer

import model, view_three_d as vtd
import traj_factory

def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication()

    t1,t2 = 'space indexed figure of height', 'space indexed figure of height3'
    #t1,t2 = 'space indexed oval', 'space indexed oval2'

    mo = model.Model(load_fact_id=t1)
    mo.load_from_factory(t2, idx=1)

    tdw = vtd.ThreeDWidget()
    tdw.display_new_trajectory(mo, 0, False); tdw.show_quad(True, idx=0)
    tdw.display_new_trajectory(mo, 1, False); tdw.show_quad(True, idx=1)

    win = QMainWindow()
    win.resize(1280, 1024)
    win.setCentralWidget(tdw)
    win.show()

    timer = QTimer(app)

    traj_nb = mo.trajectory_nb()
    duration = max([mo.get_trajectory(idx).duration for idx in range(traj_nb)])
    t0 = time.time()

    def periodic():
        nonlocal t0
        elapsed = time.time()-t0
        for i in range(traj_nb):
            Tenu2flu = mo.get_traj_pose_at(elapsed, i)
            tdw.set_quad_pose(Tenu2flu, idx=i)
            #print(f'{i}', dt, Yi,'\n')
        if elapsed >= duration: t0 += duration

    timer.timeout.connect(periodic)
    timer.start(50)
                   
    
    sys.exit(app.exec())

if __name__=="__main__":
    main()
