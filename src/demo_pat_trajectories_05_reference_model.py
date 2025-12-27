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
    fdm = p3_fdm.MR_FDM()
    traj_sp = demo_waypoints()[0]
    time = np.arange(0, traj_sp.duration, dt)
    Ys = np.array([traj_sp.get(_t) for _t in time])
    Xs = np.array([p3_ctl.DiffFlatness.state_and_cmd_of_flat_output(None, Y, fdm.P)[0] for Y in Ys])

    traj_refmod = traj = trj.RefModelTraj(traj_sp)
    Y2s = np.array([traj_refmod.get(_t) for _t in time])
    X2s = np.array([p3_ctl.DiffFlatness.state_and_cmd_of_flat_output(None, Y, fdm.P)[0] for Y in Y2s])

    threed_window = dptu.ThreeDWindow()
    threed_window.add_trajectory(time, Ys, Xs)
    threed_window.add_trajectory(time, Y2s, X2s)
    ani = threed_window.animate()
    plt.show()

    
if __name__ == "__main__":
    main()
