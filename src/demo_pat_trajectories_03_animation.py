#! /usr/bin/env python3

import math, numpy as np, matplotlib.pyplot as plt
import scipy.interpolate as interpolate
import matplotlib.animation as animation

import pat3.vehicles.rotorcraft.multirotor_trajectory as trj
import pat3.vehicles.rotorcraft.multirotor_trajectory_dev as trj_dev
import pat3.vehicles.rotorcraft.multirotor_trajectory_factory as trj_fact
import pat3.vehicles.rotorcraft.multirotor_fdm as p3_fdm
import pat3.vehicles.rotorcraft.multirotor_control as p3_ctl
import pat3.plot_utils as ppu
import pat3.utils as p3_u


import demo_pat_trajectories_utils as dptu


def demo_poly():
    return trj.SmoothBackAndForth(x0=[0, 0, 0.5, 0], x1=[1, 0, -0.5, np.pi/2]), '', ''

def demo_composite():
    l, r, v, z = 2., 1., 2, -1.
    c1, c2 = np.array([-l, 0, z]), np.array([l, 0, z])
    p1, p2 = np.array([-l, -r, z]), np.array([l, -r, z])
    p3, p4 = np.array([l, r, z]), np.array([-l, r, z])
    traj = trj.CompositeTraj([trj.Line(p1, p2, v),
                              trj.Circle(c2, r, v, -np.pi/2, np.pi),
                              trj.Line(p3, p4, v),
                              trj.Circle(c1, r, v, np.pi/2, np.pi)])
    return traj, 'Composite', 'oval made of lines and circles'

def demo_sphere():
    traj = trj_fact.Sphere0()
    return traj, traj.name, traj.desc

def demo_ref():
    #traj_sp = demo_composite()[0]
    traj_sp = demo_waypoints()[0]
    traj = trj.RefModelTraj(traj_sp)
    traj.name, traj.desc = '', ''
    return traj, traj.name, traj.desc

def demo_waypoints(v=2., dz=0.2):
    Ps = np.array([[0, 0, 0, 0], [2, 0, dz, np.pi/2], [2, 2, 0, np.pi], [0, 2, dz, 3*np.pi/2]])
    npts = len(Ps)
    segments = [trj.Line(Ps[i,:3], Ps[(i+1)%npts,:3], v) for i in range(npts)]
    stays = [trj.Cst(P, duration=1.5) for P in Ps[1:]]
    stays.append(trj.Cst(Ps[0], duration=1.5))
    steps=[]
    for stay, segment in zip(stays, segments):
        steps.append(segment); steps.append(stay)
    traj = trj.CompositeTraj(steps)
    traj.name, traj.desc = "bar", 'foo'
    return traj, traj.name, traj.desc

def main(dt=0.025):
    #traj, _, _ = demo_poly()
    #traj, _, _ = demo_composite()
    traj, _, _ = demo_sphere()
    #traj, _, _ = demo_ref()
    #traj, _, _ = demo_waypoints()
    time = np.arange(0, traj.duration, dt)
    Ys = np.array([traj.get(_t) for _t in time])
    threed_window = dptu.ThreeDWindow()
    if 1:
        Y_window = dptu.OutputWindow()
        Y_window.update_display(time, Ys)
    fdm = p3_fdm.MR_FDM()
    Xs = np.array([p3_ctl.DiffFlatness.state_and_cmd_of_flat_output(None, Y, fdm.P)[0] for Y in Ys])
    if 1:
        X_window = dptu.StateWindow()
        X_window.update_display(time, Xs)
    threed_window.add_trajectory(time, Ys, Xs)
    #threed_window.update_state_trajectory(time, Xs)
    ani = threed_window.animate()
    plt.show()
    
if __name__ == "__main__":
    main()
