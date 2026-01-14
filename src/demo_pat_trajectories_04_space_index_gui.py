#!/bin/env python3

import numpy as np, matplotlib.pyplot as plt
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

#import test_profile as tprf
import pat3.trajectory_1D as ptrj_1d
import pat3.vehicles.rotorcraft.multirotor_trajectory_dev as ptrj_dev
import pat3.vehicles.rotorcraft.multirotor_control as pctl
import pat3.vehicles.rotorcraft.multirotor_fdm as pfdm
import pat3.algebra as pal

def autoscale_axis(ax, px, py, mg=0.05): # I can't seem to get axis.autoscale to work :(
    xm, xM, ym, yM = np.min(px), np.max(px), np.min(py), np.max(py)
    dx, dy = xM-xm, yM-ym
    xm-=mg*dx; xM+=mg*dx; ym-=mg*dy; yM+=mg*dy 
    ax.set(xlim=(xm, xM), ylim=(ym, yM))

class MovableMarkers:
    def __init__(self, ax, pos, radius, moved_cbk):
        self.c = Circle(pos, radius); ax.add_artist(self.c)
        self.moved_cbk, self.pressed = moved_cbk, False
        self.c.figure.canvas.mpl_connect('button_press_event', self.on_press)
        self.c.figure.canvas.mpl_connect('button_release_event', self.on_release)
        self.c.figure.canvas.mpl_connect('motion_notify_event', self.on_motion)

    def on_press(self, event):
        self.pressed = (event.inaxes == self.c.axes and self.c.contains(event)[0])
        
    def on_motion(self, event):
        if event.inaxes == self.c.axes and self.pressed:
            ydata = np.clip(event.ydata, 0, 1)
            self.c.set(center=(event.xdata, ydata))
            self.c.figure.canvas.draw()
          
    def on_release(self, event):
        if event.inaxes == self.c.axes and self.pressed:
            self.pressed = False
            self.moved_cbk()
            
class OutputView:
    def __init__(self, model, cbk):
        self.cbk = cbk
        self.fig = plt.figure(figsize=(12, 10), layout='tight')
        self.fig.canvas.manager.set_window_title('Output')
        self.axes = ax1, ax2, ax3 = self.fig.subplots(3,1)

        ax1.set_title('Geometry'); ax1.grid(True)
        ax1.xaxis.set_label_text('relative (%)'); ax1.yaxis.set_label_text('m')
        self.lines_geom = [ax1.plot([],[], label=l)[0] for l in ['x', 'y', 'z']]
        ax1.legend()

        ax2.set_aspect('equal', 'box'); ax2.set_title('Dynamics'); ax2.grid(True)
        ax2.xaxis.set_label_text('time in s'); ax2.yaxis.set_label_text('relative (%)')
        ax2.set(xlim=(-0.1, 10.1), ylim=(-0.2, 1.2))
        self.dyn_points = [MovableMarkers(ax2, p, 0.05, cbk) for p in model.dyn_ctl_pts]
        self.line_pw_dyn, self.line_smthd_dyn = ax2.plot([],[])[0], ax2.plot([],[])[0]

        ax3.set_title('Composed'); ax3.grid(True)
        ax3.xaxis.set_label_text('time in s'); ax3.yaxis.set_label_text('m')
        self.lines_comp = [ax3.plot([],[], label=l)[0] for l in ['x', 'y', 'z']]
        ax3.legend()

    def draw_geometry(self, model):
        ls, pts_geom = model.sample_geometry()
        for i in range(3): self.lines_geom[i].set_data(ls, pts_geom[:,i,0])
        autoscale_axis(self.axes[0], ls, pts_geom[:,:,0]); self.fig.canvas.draw()
        
    def draw(self, model):
        self.line_pw_dyn.set_data(model.dyn_ctl_pts[:,0], model.dyn_ctl_pts[:,1])
        time, smtd_dyn_pts = model.sample_dynamics()
        self.line_smthd_dyn.set_data(time, smtd_dyn_pts[:,0])
        autoscale_axis(self.axes[1], time, smtd_dyn_pts[:,0])
        time, comp_pts =  model.sample_composed()
        for i in range(3): self.lines_comp[i].set_data(time, comp_pts[:,i,0])
        autoscale_axis(self.axes[2], time, comp_pts[:,:,0]);
        self.fig.canvas.draw()

class StateView:
    def __init__(self):
        self.fig = plt.figure(figsize=(12, 10), layout='tight')
        self.fig.canvas.manager.set_window_title('State')
        self.axes = ax1, ax2, ax3 = self.fig.subplots(3,1)
        ax1.set_title('Position'); ax1.grid(True)
        ax1.xaxis.set_label_text('time in s'); ax1.yaxis.set_label_text('m')
        self.lines_pos = [ax1.plot([],[], label=l)[0] for l in ['x', 'y', 'z']]
        ax1.legend()
        ax2.set_title('Velocity'); ax2.grid(True)
        ax2.xaxis.set_label_text('time in s'); ax2.yaxis.set_label_text('m/s')
        self.line_vel = ax2.plot([],[])[0]
        ax3.set_title('Attitude'); ax3.grid(True)
        ax3.xaxis.set_label_text('time in s'); ax3.yaxis.set_label_text('degres')
        self.lines_att = [ax3.plot([],[], label=l)[0] for l in ['$\\phi$', '$\\theta$']]
        ax3.legend()
         
    def draw(self, model):
        time, pos, vel, euler = model.sample_state()
        for i in range(3): self.lines_pos[i].set_data(time, pos[:,i])
        autoscale_axis(self.axes[0], time, pos)
        self.line_vel.set_data(time, vel)
        autoscale_axis(self.axes[1], time, vel)
        for i in range(2): self.lines_att[i].set_data(time, np.rad2deg(euler[:,i]))
        autoscale_axis(self.axes[2], time, np.rad2deg(euler[:,:2]))
        self.fig.canvas.draw()


class Model:
    def __init__(self):
        self.geom = ptrj_dev.SpaceCircle()
        self.set_dynamics(np.array([[0,0],[3.,0.1], [5.,0.7], [7.,0.9], [10,1]]))
        self.fdm = pfdm.MR_FDM()
        
    def set_dynamics(self, dyn_ctl_pts):
        self.dyn_ctl_pts = dyn_ctl_pts 
        dyn_segments = [ptrj_1d.AffOne(dyn_ctl_pts[i], dyn_ctl_pts[i+1]) for i in range(len(dyn_ctl_pts)-1)]
        self.smoothed_dyn = ptrj_1d.SmoothedCompositeOne(dyn_segments, eps=1.)
        self.comp_traj = ptrj_dev.SpaceIndexedTraj(self.geom, self.smoothed_dyn)

    def sample_geometry(self, npts=1000):
        ls = np.linspace(0, 1, npts)
        return ls, np.array([self.geom.get(l) for l in ls])

    def sample_dynamics(self, npts=1000):
        time = np.linspace(0, self.smoothed_dyn.duration, npts)
        return time, np.array([self.smoothed_dyn.get(t) for t in time])

    def sample_composed(self, npts=1000):
        time = np.linspace(0, self.smoothed_dyn.duration, npts)
        return time, np.array([self.comp_traj.get(t) for t in time])

    def sample_state(self, npts=1000):
        time, Ys = self.sample_composed(npts)
        Xs = np.array([pctl.DiffFlatness.state_and_cmd_of_flat_output(None, Y, self.fdm.P)[0] for Y in Ys])
        pos, vel = Xs[:, pfdm.sv_slice_pos], np.linalg.norm(Xs[:, pfdm.sv_slice_vel], axis=1)
        euler = np.array([pal.euler_of_quat(q) for q in Xs[:, pfdm.sv_slice_quat]])
        return time, pos, vel, euler
        
    
class Controller:
    def __init__(self):
        self.model = Model()
        self.output_view = OutputView(self.model, self.on_dyn_point_moved)
        self.state_view = StateView()
        self.output_view.draw_geometry(self.model)
        self.output_view.draw(self.model)
        self.state_view.draw(self.model)
        
    def on_dyn_point_moved(self):
        marker_pos = np.array([p.c.center for p in self.output_view.dyn_points])
        self.model.set_dynamics(marker_pos)
        self.output_view.draw(self.model)
        self.state_view.draw(self.model)
        
def main():
    app = Controller()
    plt.show()

if __name__=="__main__":
    main()
