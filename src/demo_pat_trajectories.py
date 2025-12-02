#! /usr/bin/env python3

import math, numpy as np, matplotlib.pyplot as plt

import pat3.vehicles.rotorcraft.multirotor_trajectory as trj
import pat3.vehicles.rotorcraft.multirotor_trajectory_dev as trj_dev
import pat3.plot_utils as pu


def demo(traj, title, savefigpath=None):
    time = np.linspace(0., traj.duration, int(traj.duration/0.05))
    Y = np.array([traj.get(t) for t in time])
    figure, axes = trj.plot(time, Y, window_title=title)
    if savefigpath is not None: plt.savefig(savefigpath)
    
def demo_line():
    p1, p2, v, psi = np.array([0, 0, 0]), np.array([1, 0, 0]), 2., np.deg2rad(45.)
    demo(trj.Line(p1, p2, v, psi), 'Line', '/tmp/demo_line.png')

def demo_circle():
    c, r, v = np.array([0, 0, 1]), 1., 2.  # center, radius, velocity
    alpha0, dalpha = 0., 2*np.pi # start angle, angle span
    zt, psit = None, None # height and heading
    demo(trj.Circle(c, r, v, alpha0, dalpha, zt, psit), 'Circle', '/tmp/demo_circle.png')

def demo_poly():
    Y0, Y1 = [0, 0, 0, 0], [1, 0, 0, 0]
    traj = trj.SmoothLine(Y00=Y0, Y10=Y1, duration=2.)
    demo(traj, 'Poly', '/tmp/demo_poly.png')

def demo_composite():
    l, r, v, z = 2., 1., 2, -1.
    c1, c2 = np.array([-l, 0, z]), np.array([l, 0, z])
    p1, p2 = np.array([-l, -r, z]), np.array([l, -r, z])
    p3, p4 = np.array([l, r, z]), np.array([-l, r, z])
    traj = trj.CompositeTraj([trj.Line(p1, p2, v),
                              trj.Circle(c2, r, v, -np.pi/2, np.pi),
                              trj.Line(p3, p4, v),
                              trj.Circle(c1, r, v, np.pi/2, np.pi)])
    demo(traj, 'Composite', '/tmp/demo_composite.png')

def demo_space_indexed(duration=5., dt=1./200):
    straj = trj_dev.SpaceCircle(r=1.5, c=[0,1.], alpha0=0, dalpha=2*np.pi)
    dtraj = trj.AffineOne(1./duration,0., duration)
    traj = trj_dev.SpaceIndexedTraj(straj, dtraj)
    demo(traj, 'Space Indexed', '/tmp/demo_space_indexed.png')
     
def main(exp=0):
    #demo_line()
    #demo_circle()
    #demo_poly()
    #demo_composite()
    #demo_space_indexed()
    plt.show()

    
    
if __name__ == "__main__":
    np.set_printoptions(linewidth=500)
    main()
