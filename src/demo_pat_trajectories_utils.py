import math, numpy as np, matplotlib.pyplot as plt
import scipy.interpolate as interpolate
import matplotlib.animation as animation
from matplotlib.colors import TABLEAU_COLORS

import pat3.vehicles.rotorcraft.multirotor_trajectory as trj
import pat3.vehicles.rotorcraft.multirotor_trajectory_dev as trj_dev
import pat3.vehicles.rotorcraft.multirotor_fdm as p3_fdm
import pat3.vehicles.rotorcraft.multirotor_control as p3_ctl
import pat3.plot_utils as ppu
import pat3.algebra as pal

class ThreeDWindow:
    def __init__(self):
        self.fig = plt.figure(figsize=(16.5, 10.5))
        self.fig.canvas.manager.set_window_title('3D')
        self.ax = self.fig.add_subplot(projection="3d")
        self.ax.set(xlim3d=(-1, 2), xlabel='X')
        self.ax.set(ylim3d=(-2, 2), ylabel='Y')
        self.ax.set(zlim3d=(-1, 2), zlabel='Z')
        self.ts, self.Ys, self.Xs, self.X = [], [], [], None
        self.lines_traj, self.points_xyz, self.lines_psi = [], [], []
        self.lines_quad_triedra, self.lines_quad_body  = [], []
        self.colors = [TABLEAU_COLORS[_k] for _k in TABLEAU_COLORS]
        
    def add_trajectory(self, t, Y, X=None):
        _c = self.colors[len(self.Ys)]
        self.ts.append(t); self.Ys.append(Y); self.Xs.append(X)
        self.lines_traj.append(self.ax.plot([], [], [], '-.', color=_c, alpha=0.7)[0])
        self.lines_traj[-1].set_data_3d(Y[:,:3,0].T)
        self.points_xyz.append(self.ax.scatter([0], [0], [0], color=_c))                    # a point for representing xyz
        self.lines_psi.append(self.ax.plot([0, 0], [0, 0], [0, 1], color=_c, alpha=0.7)[0]) # a line representing psi
        self.lines_quad_triedra.append(self.ax.plot([], [], [], color=_c, alpha=0.7)[0])
        self.lines_quad_body.append(self.ax.plot([], [], [], color=_c, alpha=0.7)[0])
        self.t = self.ts[np.argmax([ _t[-1] for _t in self.ts])] # time for longuest trajectory

    def animate(self):
        _decim = int(4) # sim at 100hz, we want anim at 25 fps... It sucks, I know
        time_template = 'time = {:0.1f}s'
        time_text = self.ax.text2D(0.05, 0.98, 'Hello', transform=self.ax.transAxes)
        def _draw_Y(p, l, Y, _s=0.2):
            x,y,z,psi = Y
            p._offsets3d = ([x], [y], [z])
            p1 = np.array([x,y,z])
            p2 = p1 + [_s*np.cos(psi), _s*np.sin(psi), 0]
            ps = np.array([p1, p2])
            l.set_data_3d(ps[:,0], ps[:,1], ps[:,2])

        def _draw_quad(idx_traj, X, d=0.2):
            rmat = pal.rmat_of_quat(X[p3_fdm.sv_slice_quat]).T # b2w_ned <-> enu
            def _b2w(ps): return np.array([rmat@p for p in ps]) + X[p3_fdm.sv_slice_pos]
            ps =_b2w([[0,0,0], [d,0,0], [0,0,0], [0,d,0], [0,0,0], [0,0,d]])
            self.lines_quad_triedra[idx_traj].set_data_3d(ps[:,0], ps[:,1], ps[:,2])
            d1, d2 = 0.1, 0.1
            ps = _b2w([[d1,0,0], [d1+d2,d2,0], [-d1-d2,d2,0], [-d1,0,0], [-d1-d2,-d2,0], [d1+d2,-d2,0], [d1,0,0]])
            self.lines_quad_body[idx_traj].set_data_3d(ps[:,0], ps[:,1], ps[:,2])
            
        def _init():
            return [time_text] + self.points_xyz + self.lines_psi
        def _animate(i):
            i1 = i*_decim
            time_text.set_text(time_template.format(self.t[i1]))
            for idx_traj ,(p, l) in enumerate(zip(self.points_xyz, self.lines_psi)):
                idx_t = min(i1, len(self.ts[idx_traj])-1)
                _draw_Y(p, l, self.Ys[idx_traj][idx_t,:,0])
                if self.Xs[idx_traj] is not None:
                    _draw_quad(idx_traj,  self.Xs[idx_traj][idx_t])
            return [time_text] + self.points_xyz + self.lines_psi + self.lines_quad_body + self.lines_quad_triedra
        dt = self.t[1]-self.t[0]
        ani = animation.FuncAnimation(self.fig, _animate, int(len(self.t)/_decim), interval=100, init_func=_init, repeat_delay=0)
        return ani


class OutputWindow:
    def __init__(self):
        self.fig = plt.figure(layout="constrained")
        self.axes = self.fig.subplots(5, 4)
    def update_display(self, t, Y):
        trj.plot(t, Y, self.fig, self.axes, window_title='Trajectory (Y)')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


class StateWindow:
    def __init__(self):
        self.fig = plt.figure(layout="constrained")
        self.axes = self.fig.subplots(4, 3)
    def update_display(self, t, X):
        p3_fdm.plot(t, X, None, self.fig, self.axes, 'Trajectory (X)')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
