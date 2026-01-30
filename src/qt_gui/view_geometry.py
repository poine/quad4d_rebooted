import logging

import numpy as np

from PySide6.QtCore import Qt, QSize, Signal, Slot
from PySide6.QtGui import QAction, QIcon, QCursor, Qt, QColor, QPalette
from PySide6.QtWidgets import (QMainWindow, QMenu, QApplication, QWidget,
                               QVBoxLayout, QHBoxLayout, QLabel, QGridLayout, QPushButton,
                               QToolBar, QStatusBar, QCheckBox)

from PySide6.Qt3DCore import (Qt3DCore)
from PySide6.Qt3DExtras import (Qt3DExtras)
from PySide6.QtGui import (QGuiApplication, QMatrix4x4, QQuaternion, QVector3D)

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvas

import misc_utils

logger = logging.getLogger(__name__)

class ProjView(FigureCanvas):
    def __init__(self, ix, iy, title, xlab, ylab, cbk):
        super().__init__(Figure(figsize=(6, 6)))  
        self.ix, self.iy = ix, iy
        self.__ax = self.figure.subplots()
        self.__ax.set_aspect('equal', 'box')
        self.__ax.set_title(title)
        self.__ax.xaxis.set_label_text(xlab); self.__ax.yaxis.set_label_text(ylab)
        self.__ax.grid(True)
        self.line_waypoints = self.__ax.plot([],[])[0]
        self.markers_waypoints = []
        self.markers_moved_cbk = cbk
        self.line_traj = self.__ax.plot([],[])[0]

    def display_new_trajectory(self, model):
        logger.debug('  in display_new_trajectory')
        for m in self.markers_waypoints: m.remove()
        self.markers_waypoints = []
        _traj = model.get_trajectory()
        if _traj.has_waypoints():
            wps = _traj.get_waypoints()
            self.markers_waypoints =\
             [misc_utils.DraggableMarker(self.__ax, wp[np.ix_([self.ix,self.iy])], f'{i+1}', self.markers_moved_cbk, self)
              for i,wp in enumerate(wps)]
            self.line_waypoints.set_data(wps[:,self.ix], wps[:,self.iy])
        else:
            self.line_waypoints.set_data([],[])
        _time, Ys = model.sample_output()
        self.line_traj.set_data(Ys[:,self.ix,0], Ys[:,self.iy,0])
        self.__ax.set(xlim=model.extends[self.ix], ylim=model.extends[self.iy])
        #self.__ax.relim()
        self.draw()
        
    def refresh(self, wps, Y):
        self.line_waypoints.set_data(wps[:,self.ix], wps[:,self.iy])
        self.line_traj.set_data(Y[:,self.ix,0], Y[:,self.iy,0])
        for wp, m in zip(wps, self.markers_waypoints):              
            m.set_position(wp[np.ix_([self.ix,self.iy])])
        self.draw() 

class TopView(ProjView):
   def __init__(self, cbk):
        super().__init__(0, 1, 'Top', 'x in m (East)', 'y in m (North)', cbk) 

class FrontView(ProjView):
   def __init__(self, cbk):
        super().__init__(0, 2, 'Front', 'x in m (East)', 'z in m (Up)', cbk) 
            
class RightView(ProjView):
    def __init__(self, cbk):
        super().__init__(1, 2, 'Right', 'y in m (North)', 'z in m (Up)', cbk)   
        

class Window(QWidget):
    closed = Signal()
    def __init__(self, model, controller): # remove controller arg
        super().__init__()
        self.setWindowTitle('Geometry view')
        self.model, self.controller = model, controller
        layout = QGridLayout() 
        layout.setContentsMargins(0,0,0,0)
        layout.setSpacing(0)
        self.views = [_V(self.on_wp_moved) for _V in [TopView, FrontView, RightView]]
        poss = [(0,0), (1,0), (1,1)]
        for v, pos in zip(self.views, poss): layout.addWidget(v, *pos)
        self.setLayout(layout)

    @Slot()
    def closeEvent(self, event):
        self.closed.emit()
        super().closeEvent(event)
        
    def on_wp_moved(self, view):
        # that sucks, maybe we keep all coordinates in each proj view ?
        proj_wps = np.array([m.get_center() for m in view.markers_waypoints])
        wps = self.model.get_waypoints()
        wps[:,view.ix] = proj_wps[:,0]
        wps[:,view.iy] = proj_wps[:,1]
        self.controller.on_geom_waypoint_moved(wps)

    def display_new_trajectory(self, model):
        logger.debug(' in display_new_trajectory')
        for v in self.views: v.display_new_trajectory(model)

    def update_plot(self, model):
        wps = model.get_waypoints()
        time, Y = model.sample_output()
        for v in self.views: v.refresh(wps, Y)
        
        
        
