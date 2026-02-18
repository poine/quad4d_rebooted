#!/bin/env python3

import sys, time, signal

from PySide6.QtWidgets import QApplication, QMainWindow

import model, view_chronograms
import traj_factory

def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication()

    t1,t2 = 'space indexed figure of height', 'space indexed figure of height3'
    t1,t2 = 'space indexed oval', 'space indexed oval2'
    mo = model.Model(load_fact_id=t1)
    mo.load_from_factory(name=t2, idx=1)
    
    sc = view_chronograms.FullStateChronogram()
    #sc = view_chronograms.StateChronogram()
    sc.display_new_trajectory(mo, 0)
    sc.display_new_trajectory(mo, 1)

    win = QMainWindow()
    win.resize(1280, 1024)
    win.setCentralWidget(sc)
    win.show()
    
    sys.exit(app.exec())

if __name__=="__main__":
    main()
