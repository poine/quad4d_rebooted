#!/bin/env python3

#
# Fisrt attempt at optimizing dynamic of a space index trajectory
#

import sys, time, signal
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

import pat3.plot_utils as p_pu
import pat3.algebra as p_al, pat3.trajectory_1D as p_t1d
import pat3.vehicles.rotorcraft.multirotor_trajectory as p_tm
import pat3.vehicles.rotorcraft.multirotor_trajectory_dev as p_tmdev
import pat3.vehicles.rotorcraft.multirotor_fdm as p_mfdm
import pat3.vehicles.rotorcraft.multirotor_control as p_mctl

#import traj_factory
import misc_utils as mu

class PiecewiseAffine:
    def __init__(self, dts, cs):
        self.dts, self.cs = np.asarray(dts), np.asarray(cs)
        self.ts = np.insert(np.cumsum(self.dts), 0, 0)
        self.duration = self.ts[-1]
        self.ps=[0]
        for i in range(0, len(self.ts)-1):
            self.ps.append(self.ps[i]+self.dts[i]*self.cs[i])
        
    def get(self, t):
        idx = (t<self.ts).argmax()-1
        return [self.ps[idx]+self.cs[idx]*(t-self.ts[idx]), self.cs[idx], 0, 0, 0]

class HomogenPieceWiseAffine(PiecewiseAffine):
    def __init__(self, duration, n_pieces=10, endval=1):
        dts, cs = duration/n_pieces*np.ones(n_pieces), endval/duration*np.ones(n_pieces)
        super().__init__(dts, cs)

    
def plot_dyn(p, npts=1000):
    ts = np.linspace(0, p.duration, npts)
    ls = np.array([p.get(t) for t in ts])
    figure = plt.figure(figsize=(12, 10), layout='tight')
    figure.suptitle('Dynamics', fontsize=16)
    nplots = p.nder
    axes = figure.subplots(nplots,1)
    for i in range(nplots):
        axes[i].plot(ts, ls[:,i])
        mu.decorate(axes[i], title=f'$\\lambda^{{({i})}}$', xlab=None, ylab=None, legend=None, grid=True)
    if p.has_ctl_pts():
        ctl_pts_t, ctl_pts_l = p.get_ctl_pts()
        axes[0].plot(ctl_pts_t, ctl_pts_l, '.')
    
def plot_geom(p):
    ls = np.linspace(0, 1, 100)
    Ys = np.array([p.get(l) for l in ls])
    figure = plt.figure(figsize=(12, 10), layout='tight')
    figure.suptitle('Geometry', fontsize=16)
    axes = ax1, ax2, ax3 = figure.subplots(3,1)
    ax1.plot(ls, Ys[:,:3, 0])
    ax2.plot(ls, Ys[:,:3, 1])
    ax3.plot(ls, Ys[:,:3, 2])
    
def plot_comp(comp, npts=1000):
    ts = np.linspace(0, comp.duration, npts)
    Ys = np.array([comp.get(t) for t in ts])
    figure = plt.figure(figsize=(12, 10), layout='tight')
    figure.suptitle('Composed', fontsize=16)
    axes = ax1, ax2, ax3 = figure.subplots(3,1)
    for d in range(3):
        for c in range(3):
            axes[d].plot(ts, Ys[:,c, d]) 
        axes[d].set_title(f'Y^{d}'); axes[d].grid(True)

def plot_state(comp, npts=1000):
    ts = np.linspace(0, comp.duration, npts)
    Ys = np.array([comp.get(t) for t in ts])
    figure = plt.figure(figsize=(12, 10), layout='tight')
    figure.suptitle('State', fontsize=16)
    axes = ax1, ax2, ax3, ax4 = figure.subplots(4,1)
    ax1.set_title('Position'); ax1.grid(True)
    for c in range(3):
        ax1.plot(ts, Ys[:,c, 0])
    vel = np.linalg.norm(Ys[:,:3,1], axis=1)
    ax2.set_title('Velocity'); ax2.grid(True)
    ax2.plot(ts, vel)
    accel = np.linalg.norm(Ys[:,:3,2], axis=1)
    ax3.set_title('Acceleration'); ax3.grid(True)
    ax3.plot(ts, accel)
    _fdm = p_mfdm.MR_FDM()
    Xs = np.zeros((len(ts), p_mfdm.sv_size))
    for i in range(0, len(ts)):
        Xs[i], _, _ = p_mctl.DiffFlatness.state_and_cmd_of_flat_output(Ys[i], _fdm.P)
    euler = np.array([p_al.euler_of_quat(q) for q in Xs[:, p_mfdm.sv_slice_quat]])
    ax4.plot(ts, np.rad2deg(euler[:,0]))
    ax4.plot(ts, np.rad2deg(euler[:,1]))
    ax4.set_title('Euler'); ax4.grid(True)

def test1():
    #p = HomogenPieceWiseAffine(duration=10, n_pieces=5)
    #p = PiecewiseAffine(dts=[0.5, 1, 0.5], cs=[0.1, 0.2, 0.3])
    np.set_printoptions(linewidth=500)
    #p = p_t1d.SmoothStopStopCstVel(dt_acc=0.4, dl_acc=0.04, dt_cruise=5.)
    p = p_t1d.SmoothStopStopCstVel(dt_acc=0.4, dl_acc=0.06, dt_cruise=5.)
    plot_dyn(p)
    plt.show()

def test2():
    geom = p_tmdev.SpaceCircle(ztraj=p_t1d.CstOne(0))
    #geom = p_tmdev.SpaceIndexedLine(np.array([0, 0, 1]), np.array([10., 0, 1]), 0)
    plot_geom(geom)
    plt.show()
    dyn = HomogenPieceWiseAffine(duration=10, n_pieces=5)
    comp = p_tmdev.SpaceIndexedTraj(geom, dyn)
    #comp = p_tm.Line(np.array([0, 0, 1]), np.array([10., 0, 1]), v=2.) 
    #plot_dyn(dyn)
    plot_comp(comp)
    plt.show()

def test3(): # example from https://codesignal.com/learn/courses/optimization-with-scipy/lessons/optimization-with-constraints-using-scipy
    def f(x): return 2*x[0]**2 - 5*x[0] + 3*x[1]**2
    constraint = {
        'type': 'ineq',
        'fun': lambda x: 1 - (x[0] + x[1])
    }
    initial_guess = [1, 1]
    result_constraints = minimize(f, initial_guess, constraints=[constraint])
    print("Optimal solution:", result_constraints.x)
    print("Objective function value at optimal solution:", result_constraints.fun) 

def test4(max_vel=1.5): # optimize piecewise linear dynamics for duration constrained by max vel
    #geom = p_tmdev.SpaceCircle(ztraj=p_t1d.CstOne(0))
    geom = p_tmdev.SpaceIndexedLine(np.array([0, 0, 1]), np.array([10., 0, 1]), 0)
    def cost(x): return x
    def vel_cons(x, max_vel=max_vel):
        dur = x[0]
        dyn = PiecewiseAffine(dts=[dur], cs=[1/dur])
        comp = p_tmdev.SpaceIndexedTraj(geom, dyn)
        Ys = np.array([comp.get(t) for t in np.linspace(0, comp.duration, 100)])
        vel = np.linalg.norm(Ys[:,:3,1], axis=1)
        return max_vel-np.max(vel)
    constraint1 = {'type': 'ineq', 'fun': vel_cons}
    constraint2 = {'type': 'ineq', 'fun': lambda x: x[0]} # duration is positive
    initial_guess = [20.]
    result = minimize(cost, initial_guess, constraints=[constraint1, constraint2])
    print("Optimal solution:", result.x)
    print("Objective function value at optimal solution:", result.fun) 
    dur = result.x[0]
    dyn = PiecewiseAffine(dts=[dur], cs=[1/dur])
    plot_dyn(dyn)
    comp = p_tmdev.SpaceIndexedTraj(geom, dyn)
    plot_comp(comp, npts=100)
    plot_state(comp)
    plt.show()

       
    
def test5(max_vel=4., max_acc=8.): # optimize poly-line-poly dynamics for duration constrained by max vel and max accel
    #geom = p_tmdev.SpaceIndexedLine(np.array([0, 0, 1]), np.array([10., 0, 1]), 0)
    #geom = p_tmdev.SpaceCircle(ztraj=p_t1d.CstOne(0))
    wps = np.array([[0.2,0, 1],[2.,3., 2], [2.,-3., 3], [-2.,-3., 4], [-2.,3., 3], [-0.2, 0., 2]])
    geom = p_tmdev.SpaceWaypoints(wps)
    def traj_from_params(x):
        if 0:
            (dt1, dt2, dt3), (c1, c3) = x, (0, 0)
            c2 = (1. - (dt1*c1 + dt3*c3))/dt2  # last slope computed to reach 1.
            dts, cs = [dt1, dt2, dt3], [c1, c2, c3]
            #dyn = PiecewiseAffine(dts=[dt1, dt2, dt3], cs=[c1, c2, c3])
            dyns, tf, lf = [], 0, 0
            for dt, c in zip(dts, cs):
                ti=tf; li=lf; tf+=dt; lf+=c*dt
                dyns.append(p_t1d.AffOne([ti,li],[tf, lf]))
            dyn = p_t1d.CompositeOne(dyns) 
            #dyn = p_t1d.SmoothedCompositeOne(dyns)
        else:
            dt_acc, dl_acc, dt_cruise = x
            dyn=p_t1d.SmoothStopStopCstVel(dt_acc, dl_acc, dt_cruise)
        comp = p_tmdev.SpaceIndexedTraj(geom, dyn)
        return dyn, comp
    def cost(x): return 2*x[0]+x[2] #x[0]+x[1]+x[2]
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
    constraint1 = {'type': 'ineq', 'fun': vel_cons, 'desc':'vel'}
    constraint2 = {'type': 'ineq', 'fun': acc_cons, 'desc':'acc'}
    #constraint3 = {'type': 'ineq', 'fun': lambda x: x[0]-0.2} # first_step > 0.2s
    #constraint4 = {'type': 'ineq', 'fun': lambda x: x[1]-1} # first_step > 1s
    #constraint5 = {'type': 'ineq', 'fun': lambda x: x[2]-0.2} # first_step > 0.2s
    #constraint6 = {'type': 'eq', 'fun': lambda x: x[3]} # c1=0
    #constraint7 = {'type': 'eq', 'fun': lambda x: x[4]} # c3=0
    constraints = [constraint1, constraint2]
    #bounds = [(0.2, 100), (0.2, 1000), (0.2, 100)]
    #initial_guess = [1, 1, 1]
    bounds = [(0.1, 100), (0.001, 0.5), (0.2, 1000)]
    initial_guess = [1, 0.05, 1]
    print("Starting optimization"); t1=time.time()
    result = minimize(cost, initial_guess, constraints=constraints, bounds=bounds)
    t2=time.time(); print(f"ran in: {t2-t1:.2f}s")
    print("Optimal solution:", result.x)
    print(f"Optimal cost: {result.fun:.2f}s")
    for cons in [constraint1, constraint2]:
        print(f"{cons['desc']} constraint: {cons['fun'](result.x):.2e}")
    dyn, comp = traj_from_params(result.x)
    plot_dyn(dyn)
    plot_comp(comp)
    plot_state(comp)
    plt.show()




    
def main():
    #trajectory = traj_factory.TrajFactory.trajectories['ex_si0']()
    #test1()
    #test2()
    #test3()
    #test4(max_vel=0.5)
    test5()

    
if __name__=="__main__":
    main()
