#!/bin/env python3

import sys, time, signal

from PySide6.QtWidgets import QApplication, QMainWindow

import view_three_d as vtd, model

def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication()

    mo = model.Model(load_fact_id='space indexed figure of height')
    #mo.load_from_factory('space indexed figure of height2', replace=None)

    tdw = vtd.ThreeDWidget()
    tdw.display_new_trajectory(mo, 0)

    win = QMainWindow()
    win.resize(1280, 1024)
    win.setCentralWidget(tdw)
    win.show()

    sys.exit(app.exec())

if __name__=="__main__":
    main()
