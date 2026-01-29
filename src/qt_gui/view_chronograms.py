import numpy as np

from matplotlib.figure import Figure
from matplotlib.patches import Circle
from matplotlib.backends.backend_qtagg import FigureCanvas

from PySide6.QtWidgets import (QWidget, QVBoxLayout)


import pat3.vehicles.rotorcraft.multirotor_trajectory as p_mt


# I can't seem to get axis.autoscale to work :(
def autoscale_axis(ax, px, py, mg=0.05, min_span=0.1):
    xm, xM, ym, yM = np.min(px), np.max(px), np.min(py), np.max(py)
    #dx, dy = np.max(min_span, xM-xm), np.max(min_span, yM-ym) # wtf!!!!
    dx, dy = max(min_span, xM-xm), max(min_span, yM-ym)
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
            ydata = np.clip(event.ydata, 0., 1.)
            self.c.set(center=(event.xdata, ydata))
            self.c.figure.canvas.draw()
          
    def on_release(self, event):
        if event.inaxes == self.c.axes and self.pressed:
            self.pressed = False
            self.moved_cbk()

class OutputChronogram(FigureCanvas):
    def __init__(self):
        super().__init__(Figure(figsize=(12, 10)))
        self.axes = self.figure.subplots(p_mt._nder, p_mt._ylen)
        self.figure.tight_layout()
        self.lines = np.empty_like(self.axes)
        ylabel = ['$x^{{({})}}$', '$y^{{({})}}$', '$z^{{({})}}$', '$\\psi^{{({})}}$']
        for i in range(p_mt._nder):
            for ax, l in zip(self.axes[i], ylabel):
                ax.set_title(l.format(i))
                ax.grid(True)
        for d in range(p_mt._nder):
            for c in range(p_mt._ylen):
                self.lines[d, c] = self.axes[d,c].plot([],[])[0]
                
    def update_plot(self, model):
        time, Y = model.sample_output()
        #breakpoint()
        for d in range(p_mt._nder):
            for c in range(p_mt._ylen):
                #self.axes[d,c].plot(time, Y[:,c,d])
                self.lines[d, c].set_data(time, Y[:,c,d])
                autoscale_axis(self.axes[d,c], time, Y[:,c,d])
        self.draw()
        
            
class StateChronogram(FigureCanvas):
    def __init__(self):
        super().__init__(Figure(figsize=(12, 10)))
        self.figure.tight_layout()#FIXME
        self.axes = ax1, ax2, ax3 = self.figure.subplots(3,1)
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

    def update_plot(self, model):
        time, pos, vel, euler = model.sample_state()
        for i in range(3): self.lines_pos[i].set_data(time, pos[:,i])
        autoscale_axis(self.axes[0], time, pos)
        self.line_vel.set_data( time, vel)
        autoscale_axis(self.axes[1], time, vel)
        for i in range(2): self.lines_att[i].set_data(time, np.rad2deg(euler[:,i]))
        autoscale_axis(self.axes[2], time, np.rad2deg(euler[:,:2]))
        self.draw()
            
class SpaceIndexedChronogram(FigureCanvas):
    def __init__(self, model, cbk):
        super().__init__(Figure(figsize=(12, 10)))
        self.figure.tight_layout() # FIXME
        self.axes = ax1, ax2, ax3 = self.figure.subplots(3,1)

        ax1.set_title('Geometry'); ax1.grid(True)
        ax1.xaxis.set_label_text('relative (%)'); ax1.yaxis.set_label_text('m')
        self.lines_geom = [ax1.plot([],[], label=l)[0] for l in ['x', 'y', 'z']]
        ax1.legend()
        
        ax2.set_aspect('equal', 'box'); ax2.set_title('Dynamics'); ax2.grid(True)
        ax2.xaxis.set_label_text('time in s'); ax2.yaxis.set_label_text('relative (%)')
        self.dyn_points = [MovableMarkers(ax2, p, 0.05, cbk) for p in model.trajectory.get_dyn_ctl_pts()]
        self.line_pw_dyn, self.line_smthd_dyn = ax2.plot([],[])[0], ax2.plot([],[])[0]

        ax3.set_title('Composed'); ax3.grid(True)
        ax3.xaxis.set_label_text('time in s'); ax3.yaxis.set_label_text('m')
        self.lines_comp = [ax3.plot([],[], label=l)[0] for l in ['x', 'y', 'z']]
        ax3.legend()

    def _draw(self, model):
        dyn_ctl_pts = model.trajectory.get_dyn_ctl_pts()
        self.line_pw_dyn.set_data(dyn_ctl_pts[:,0], dyn_ctl_pts[:,1])
        time, smtd_dyn_pts = model.sample_dynamics()
        self.line_smthd_dyn.set_data(time, smtd_dyn_pts[:,0])
        autoscale_axis(self.axes[1], time, smtd_dyn_pts[:,0])
        time, comp_pts =  model.sample_output()
        for i in range(3): self.lines_comp[i].set_data(time, comp_pts[:,i,0])
        autoscale_axis(self.axes[2], time, comp_pts[:,:,0]);
        self.draw()
        
    def draw_geometry(self, model):
        ls, pts_geom = model.sample_geometry()
        for i in range(3): self.lines_geom[i].set_data(ls, pts_geom[:,i,0])
        autoscale_axis(self.axes[0], ls, pts_geom[:,:,0]); self.draw()

# pack some plot into a QWidget
class ChronogramWindow(QWidget):
    def __init__(self, chrono_class, title=''):
        super().__init__()
        self.setWindowTitle(title)
        layout = QVBoxLayout()
        self._chronogram = chrono_class()
        layout.addWidget(self._chronogram)
        self.setLayout(layout)
    
    def update_plot(self, model): self._chronogram.update_plot(model)
