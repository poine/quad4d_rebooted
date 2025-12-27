#! /usr/bin/env python3

import math, numpy as np, matplotlib.pyplot as plt

import pat3.vehicles.rotorcraft.multirotor_trajectory as trj
import pat3.vehicles.rotorcraft.multirotor_trajectory_dev as trj_dev
import pat3.plot_utils as pu

import demo_pat_trajectories_utils as dptu

import scipy.interpolate as interpolate
class SmoothWaypoints:
    def __init__(self, times, waypoints):
        self._ylen, self._nder = 4, 5
        self.times, self.waypoints = times, np.array(waypoints)
        #l = np.linspace(0, 1, len(self.waypoints))
        self.splines = [interpolate.InterpolatedUnivariateSpline(self.times, self.waypoints[:,i], k=4) for i in range(self._ylen)]
        self.duration = times[-1]
        
    def get(self, l):
        Yl = np.zeros((self._ylen, self._nder))
        Yl = [self.splines[i].derivatives(l) for i in range(self._ylen)]
        return Yl
 


def main():
    waypoints = np.array([[0, 0, 0, 0], [2, 0, 0, np.pi/2], [2, 2, 0, np.pi], [0, 2, 0, 3*np.pi/2], [0, 0, 1, 0], [2, 0, 1, 0], [2, 2, 1, 0], [0, 2, 1, 0], [0, 0, 0, 0]])
    npts, vel = len(waypoints), 2.5
    segments = [trj.Line(waypoints[i,:3], waypoints[(i+1),:3], vel) for i in range(npts-1)]
    traj = trj.CompositeTraj(segments)
    dt=0.01; time = np.arange(0, traj.duration, dt)
    Ys = np.array([traj.get(_t) for _t in time])
    Y_window = dptu.OutputWindow()
    Y_window.update_display(time, Ys)

    threed_window = dptu.ThreeDWindow()
    threed_window.add_trajectory(time, Ys)
    
    segments = waypoints[1:,] - waypoints[:-1]
    lengths = np.linalg.norm(segments[:,:3], axis=1)
    durations = lengths/vel
    times = np.insert(np.cumsum(durations), 0, 0)
    traj2 = SmoothWaypoints(times, waypoints)
    time = np.arange(0, traj2.duration, dt)
    Ys = np.array([traj2.get(_t) for _t in time])
    Y_window.update_display(time, Ys)
    #breakpoint()
    #trj.SplinesOne(points_t, points_l)
    threed_window.add_trajectory(time, Ys)
     
    ani = threed_window.animate()
    
    plt.show()

if __name__ == "__main__":
    main()
