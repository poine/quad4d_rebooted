#!/bin/env python3

#
# Second attempt at optimizing dynamic of a space index trajectory
#
# a splines as space index
#
import sys, time, signal
import numpy as np, matplotlib.pyplot as plt
from scipy.optimize import minimize

from PySide6.QtWidgets import QApplication, QMainWindow

import pat3.algebra as p_al, pat3.trajectory_1D as p_t1d
import pat3.vehicles.rotorcraft.multirotor_trajectory as p_tm
import pat3.vehicles.rotorcraft.multirotor_trajectory_dev as p_tmdev

import view_chronograms as vc, view_three_d as vtd, model

import test_optim_dyn


def optimize_dyn(max_vel=3., max_acc=3.):
    wps = np.array([[0.2,0, 1],[2.,3., 2], [2.,-3., 3], [-2.,-3., 4], [-2.,3., 3], [-0.2, 0., 2]])
    geom = p_tmdev.SpaceWaypoints(wps)
    npts = 8
    def traj_from_params(x):
        dur, ls = x[0], x[1:]
        pts_t = np.linspace(0, dur, npts)
        pts_l = np.insert(np.insert(ls, 0, 0), npts-1, 1)
        dyn = p_t1d.SplineOne(pts_t, pts_l) 
        comp = p_tmdev.SpaceIndexedTraj(geom, dyn)
        return dyn, comp

    def cost(x): return x[0]
    def vel_cons(x):
        dyn, comp = traj_from_params(x)
        Ys = np.array([comp.get(t) for t in np.linspace(0, comp.duration, 500)])
        vel = np.linalg.norm(Ys[:,:3,1], axis=1)
        return 100*(max_vel-np.max(vel))
    def acc_cons(x):
        dyn, comp = traj_from_params(x)
        Ys = np.array([comp.get(t) for t in np.linspace(0, comp.duration, 500)])
        acc = np.linalg.norm(Ys[:,:3,2], axis=1)
        return max_acc-np.max(acc)
    x0 = np.insert(np.linspace(0, 1, npts)[1:-1], 0,10.)
    constraint1 = {'type': 'ineq', 'fun': vel_cons, 'desc':'vel'}
    constraint2 = {'type': 'ineq', 'fun': acc_cons, 'desc':'acc'}
    constraints = [constraint1, constraint2]
    bounds = [(0.1, 100)]+[(0.001, 0.999)]*(npts-2)
    x0 = np.insert(np.linspace(0, 1, npts)[1:-1], 0, 20.)
    print("Starting optimization"); t1=time.time()
    result = minimize(cost, x0, constraints=constraints, bounds=bounds)
    t2=time.time(); print(f"ran in: {t2-t1:.2f}s")
    print("Optimal solution:", result.x)
    print(f"Optimal cost: {result.fun:.2f}s")
    for cons in [constraint1, constraint2]:
        print(f"{cons['desc']} constraint: {cons['fun'](result.x):.2e}")
    dyn, comp = traj_from_params(result.x)
    test_optim_dyn.plot_dyn(dyn)
    test_optim_dyn.plot_comp(comp)
    test_optim_dyn.plot_state(comp)
    plt.show()
    
  
def main():

    optimize_dyn()

    return

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QApplication()
    win = QMainWindow()
    win.resize(1280, 1024)
    win.show()
    mo = model.Model(load_fact_id='space indexed figure of height')
    #mo.load_from_factory('space indexed figure of height2', replace=None)
    mo.load_from_factory('circle north', replace=None)

    if 1:
        fsw = vc.ChronogramWindow(vc.FullStateChronogram)
        fsw.display_new_trajectory(mo, 0)
        fsw.display_new_trajectory(mo, 1)
        win.setCentralWidget(fsw)
    if 0:
        tdw = vtd.ThreeDWidget()
        tdw.display_new_trajectory(mo, 0)
        #breakpoint()
        #tdw.display_new_trajectory(mo, 1)
        #breakpoint()
        win.setCentralWidget(tdw)
    
    sys.exit(app.exec())

if __name__=="__main__":
    main()
