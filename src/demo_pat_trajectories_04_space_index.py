#! /usr/bin/env python3

import math, numpy as np, matplotlib.pyplot as plt

import pat3.vehicles.rotorcraft.multirotor_trajectory as trj
import pat3.vehicles.rotorcraft.multirotor_trajectory_dev as trj_dev
import pat3.vehicles.rotorcraft.multirotor_fdm as p3_fdm
import pat3.vehicles.rotorcraft.multirotor_control as p3_ctl

import demo_pat_trajectories_utils as dptu

def demo_varying_dynamics(geom, durations, dt=0.01):
    _fdm = p3_fdm.MR_FDM() # needed for inertias...
    f1, a1, f2, a2 = None, None, None, None # we use two windows for output and state
    threed_window = dptu.ThreeDWindow()
    for duration in durations:
        dyn = trj.AffineOne(1./duration,0., duration) # dynamic with given duration
        traj = trj_dev.SpaceIndexedTraj(geom, dyn)    # 
        time = np.linspace(0., traj.duration, int(traj.duration/dt))
        Y = np.array([traj.get(t) for t in time])
        f1, a1 = trj.plot(time, Y, f1, a1, window_title="Output trajectory")
        X = np.array([p3_ctl.DiffFlatness.state_and_cmd_of_flat_output(None, _y, _fdm.P)[0] for _y in Y])
        f2, a2 = p3_fdm.plot(time, X, figure=f2, axes=a2, window_title="State Trajectory", U=None)
        threed_window.add_trajectory(time, Y, X)
    return threed_window.animate()
        
def main(dt=0.01):
    straj = trj_dev.SpaceCircle(r=1.5, c=[0,1.], alpha0=0, dalpha=2*np.pi)
    anim = demo_varying_dynamics(straj, [6, 3, 2, 1])
    plt.show()
    
if __name__ == "__main__":
    np.set_printoptions(linewidth=500)
    main()
