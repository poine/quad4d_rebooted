#! /usr/bin/env python3

import math, numpy as np, matplotlib.pyplot as plt

import pat3.vehicles.rotorcraft.multirotor_trajectory as trj
import pat3.vehicles.rotorcraft.multirotor_fdm as p3_fdm
import pat3.vehicles.rotorcraft.multirotor_control as p3_ctl


def main(dt=0.01):
    p1, p2, v, psi = np.array([0, 0, 0]), np.array([1, 0, 0]), 2., np.deg2rad(45.)
    traj = trj.Line(p1, p2, v, psi)

    c, r, v = np.array([0, 0, 1]), 1., 2.  # center, radius, velocity
    alpha0, dalpha = 0., 2*np.pi # start angle, angle span
    zt, psit = None, None # height and heading
    traj = trj.Circle(c, r, v, alpha0, dalpha, zt, psit)
    
    time = np.linspace(0., traj.duration, int(traj.duration/dt))
    Yr = np.array([traj.get(t) for t in time])

    _fdm = p3_fdm.MR_FDM()
    Xr, Ur = np.zeros((len(time), p3_fdm.sv_size)), np.zeros((len(time), p3_fdm.iv_size))
    for i in range(0, len(time)):
        Xr[i], Ur[i], Xd = p3_ctl.DiffFlatness.state_and_cmd_of_flat_output(None, Yr[i], _fdm.P)

    figure, axes = trj.plot(time, Yr, window_title="Output trajectory")

    _f3, _a3 = p3_fdm.plot(time, Xr, window_title="State Trajectory", U=Ur)

    plt.show()

    
    
if __name__ == "__main__":
    np.set_printoptions(linewidth=500)
    main()
