#! /usr/bin/env python3

import math, numpy as np, matplotlib.pyplot as plt

import pat3.vehicles.rotorcraft.multirotor_trajectory as trj
import pat3.vehicles.rotorcraft.multirotor_trajectory_dev as trj_dev
import pat3.vehicles.rotorcraft.multirotor_fdm as p3_fdm
import pat3.vehicles.rotorcraft.multirotor_control as p3_ctl


def main(dt=0.01):
    p1, p2, v, psi = np.array([0, 0, 0]), np.array([1, 0, 0]), 2., np.deg2rad(45.)
    traj = trj.Line(p1, p2, v, psi)

    c, r, v = np.array([0, 0, 1]), 1., 2.  # center, radius, velocity
    alpha0, dalpha = 0., 2*np.pi # start angle, angle span
    zt, psit = None, None # height and heading
    traj = trj.Circle(c, r, v, alpha0, dalpha, zt, psit)

    straj = trj_dev.SpaceCircle(r=1.5, c=[0,1.], alpha0=0, dalpha=2*np.pi)
    times, Ys, Xs, Us = [], [], [], []
    for duration in [10, 5, 2, 1]:
        dtraj = trj.AffineOne(1./duration,0., duration)
        traj = trj_dev.SpaceIndexedTraj(straj, dtraj)
    
        times.append(np.linspace(0., traj.duration, int(traj.duration/dt)))
        Ys.append(np.array([traj.get(t) for t in times[-1]]))

        _fdm = p3_fdm.MR_FDM()
        Xs.append(np.zeros((len(times[-1]), p3_fdm.sv_size))); Us.append(np.zeros((len(times[-1]), p3_fdm.iv_size)))
        for i in range(0, len(times[-1])):
            Xs[-1][i], Us[-1][i], Xd = p3_ctl.DiffFlatness.state_and_cmd_of_flat_output(None, Ys[-1][i], _fdm.P)
    
    f1, a1, f2, a2 = None, None, None, None
    for time, Y, X, U in zip(times, Ys, Xs, Us):
        f1, a1 = trj.plot(time, Y, f1, a1, window_title="Output trajectory")
        f2, a2 = p3_fdm.plot(time, X, figure=f2, axes=a2, window_title="State Trajectory", U=None)

    plt.show()

    
    
if __name__ == "__main__":
    np.set_printoptions(linewidth=500)
    main()
