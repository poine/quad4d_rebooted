import logging
import numpy as np

from matplotlib.figure import Figure
from matplotlib.patches import Circle
from matplotlib.backends.backend_qtagg import FigureCanvas

from PySide6.QtCore import Qt, Signal, Slot
from PySide6.QtWidgets import (QWidget, QVBoxLayout)


import pat3.vehicles.rotorcraft.multirotor_trajectory as p_mt
import misc_utils as mu

logger = logging.getLogger(__name__)

class SpaceIndexedChronogram(FigureCanvas):
    def __init__(self, model, cbk):
        super().__init__(Figure(figsize=(12, 10)))
        self.figure.tight_layout() # FIXME: doesn't seem to work
        self.axes = ax1, ax2, ax3 = self.figure.subplots(3,1)

        self.lines_geom = [ax1.plot([],[], label=l)[0] for l in ['x', 'y', 'z']]
        mu.decorate(ax1, title='Geometry', xlab='relative (%)', ylab='m', legend=True, grid=True)
        
        self.markers_dyn = []
        self.line_marker_dyn, self.line_dyn = ax2.plot([],[])[0], ax2.plot([],[])[0]
        self.markers_cbk = cbk
        mu.decorate(ax2, title='Dynamics', xlab='time in s', ylab='relative (%)', grid=True)
        
        self.lines_comp = [ax3.plot([],[], label=l)[0] for l in ['x', 'y', 'z']]
        mu.decorate(ax3, title='Composed', xlab='time in s', ylab='m', legend=True, grid=True)

    def _draw(self, model):
        traj = model.get_trajectory()
        if traj.has_dyn_ctl_pts():
            dyn_ctl_pts = traj.get_dyn_ctl_pts()
            self.line_marker_dyn.set_data(dyn_ctl_pts[:,0], dyn_ctl_pts[:,1])
        time, smtd_dyn_pts = model.sample_dynamics()
        self.line_dyn.set_data(time, smtd_dyn_pts[:,0])
        mu.autoscale_axis(self.axes[1], time, smtd_dyn_pts[:,0])
        time, comp_pts =  model.sample_output()
        for i in range(3): self.lines_comp[i].set_data(time, comp_pts[:,i,0])
        mu.autoscale_axis(self.axes[2], time, comp_pts[:,:,0]);
        #self.axes[1].relim(); self.axes[2].relim()
        self.draw()
        
    def draw_geometry(self, model):
        ls, pts_geom = model.sample_geometry()
        for i in range(3): self.lines_geom[i].set_data(ls, pts_geom[:,i,0])
        mu.autoscale_axis(self.axes[0], ls, pts_geom[:,:,0]); self.draw()
        #self.axes[0].relim(); self.draw() # !!!! WTF!!!!

    def update_plot(self, model):
        self.draw_geometry(model)
        self._draw(model)
        
    def display_new_trajectory(self, model):
        print(' #SpaceIndexedChronogram::display_new_trajectory')
        for m in self.markers_dyn: m.remove()
        self.markers_dyn = []
        traj = model.get_trajectory()
        if traj.has_dyn_ctl_pts():
            dyn_ctl_pts = model.trajectory.get_dyn_ctl_pts()
            self.markers_dyn = [mu.DraggableMarker(self.axes[1], p, f'{i+1}', self.markers_cbk, None, (0,1)) for i, p in enumerate(dyn_ctl_pts)]
        self.update_plot(model)
                     
class SiChronoWindow(QWidget):
    closed = Signal()
    def __init__(self, model, controller):
        super().__init__()
        self.controller = controller
        self.setWindowTitle('Space Index Chronogram')
        layout = QVBoxLayout()
        layout.setContentsMargins(0,0,0,0)
        layout.setSpacing(0)
        self.si_chrono_view = SpaceIndexedChronogram(model, self.on_marker_moved)
        layout.addWidget(self.si_chrono_view)
        self.setLayout(layout)

    @Slot()
    def closeEvent(self, event):
        self.closed.emit()
        super().closeEvent(event)
            
    def on_marker_moved(self, arg):
        marker_pos = np.array([p.get_center() for p in self.si_chrono_view.markers_dyn])
        self.controller.on_dyn_ctl_point_moved(marker_pos)

    def display_new_trajectory(self, model):
        self.si_chrono_view.display_new_trajectory(model)

    def update_plot(self, model): self.si_chrono_view.update_plot(model)
    
        
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
                
    # no difference between new trajectory and update
    def display_new_trajectory(self, model):
        print(' #OutputChronogram::display_new_trajectory')
        self.update_plot(model)

    def update_plot(self, model):
        time, Y = model.sample_output()
        for d in range(p_mt._nder):
            for c in range(p_mt._ylen):
                #self.axes[d,c].plot(time, Y[:,c,d])
                self.lines[d, c].set_data(time, Y[:,c,d])
                #self.axes[d,c].relim()
                mu.autoscale_axis(self.axes[d,c], time, Y[:,c,d])
        self.draw()
        
            
class StateChronogram(FigureCanvas):
    def __init__(self):
        super().__init__(Figure(figsize=(12, 10)))
        self.figure.tight_layout()# FIXME, not working, large margins :(
        self.axes = ax1, ax2, ax3 = self.figure.subplots(3,1, sharex=True)

        self.lines_pos = [ax1.plot([],[], label=l)[0] for l in ['x', 'y', 'z']]
        mu.decorate(ax1, title='Position', ylab='m', legend=True, grid=True)

        self.line_vel = ax2.plot([],[])[0]
        mu.decorate(ax2, title='Velocity', ylab='m/s', grid=True)

        self.lines_att = [ax3.plot([],[], label=l)[0] for l in ['$\\phi$', '$\\theta$']]
        mu.decorate(ax3, title='Attitude', xlab='time in s', ylab='degres', legend=True, grid=True)

    # no difference between new trajectory and update
    def display_new_trajectory(self, model):
        print(' #StateChronogram::display_new_trajectory')
        self.update_plot(model)
        
    def update_plot(self, model):
        time, pos, vel, euler = model.sample_state()
        for i in range(3): self.lines_pos[i].set_data(time, pos[:,i])
        self.line_vel.set_data( time, vel)
        for i in range(2): self.lines_att[i].set_data(time, np.rad2deg(euler[:,i]))
        for ax in self.axes: ax.relim()
        #self.draw()
        mu.autoscale_axis(self.axes[1], time, vel)
        self.draw()   

# pack some plot into a QWidget
class ChronogramWindow(QWidget):
    closed = Signal()
    def __init__(self, chrono_class, title=''):
        super().__init__()
        self.setWindowTitle(title)
        layout = QVBoxLayout()
        self._chronogram = chrono_class()
        layout.addWidget(self._chronogram)
        self.setLayout(layout)

    @Slot()
    def closeEvent(self, event):
        self.closed.emit()
        super().closeEvent(event)
        
    def update_plot(self, model): self._chronogram.update_plot(model)

    def display_new_trajectory(self, model):
        self._chronogram.display_new_trajectory(model)
