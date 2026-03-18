#!/bin/env python3

import sys, time, signal
import numpy as np

from PySide6.QtWidgets import QApplication, QMainWindow

import view_three_d as vtd, model

import misc_utils as mu

def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication()

    mo = model.Model(traj_fact_id='space indexed figure of height',
                     arena_cfg='data/arena_3.yaml')

    tdw = vtd.ThreeDWidget(mo)
    tdw.set_item_visible('frames', True)
    #tdw.set_item_visible('arena', True)
    tdw.display_new_trajectory(mo, 0)

    win = QMainWindow()
    win.resize(1280, 1024)
    win.setCentralWidget(tdw)
    win.show()

    sys.exit(app.exec())

if __name__=="__main__":
    main()
