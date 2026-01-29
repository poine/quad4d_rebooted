import numpy as np

from PySide6.QtGui import QAction, QIcon, QCursor, Qt, QColor, QPalette
from PySide6.QtWidgets import (QMainWindow, QMenu, QApplication, QWidget,
                               QVBoxLayout, QHBoxLayout, QLabel, QGridLayout, QPushButton,
                               QToolBar, QStatusBar, QCheckBox)

from PySide6.Qt3DCore import (Qt3DCore)
from PySide6.Qt3DExtras import (Qt3DExtras)
from PySide6.QtGui import (QGuiApplication, QMatrix4x4, QQuaternion, QVector3D)

from matplotlib.figure import Figure
#from matplotlib.patches import Circle
from matplotlib.backends.backend_qt5agg import FigureCanvas


import view_three_d as tdvg
import misc_utils


class ProjView(FigureCanvas):
    def __init__(self, ix, iy, title, xlab, ylab, model, cbk):
        super().__init__(Figure(figsize=(6, 6)))  
        self.ix, self.iy = ix, iy
        self.__ax = self.figure.subplots()
        self.__ax.set(xlim=model.extends[self.ix], ylim=model.extends[self.iy])
        self.__ax.set_aspect('equal', 'box')
        self.__ax.set_title(title)
        self.__ax.xaxis.set_label_text(xlab); self.__ax.yaxis.set_label_text(ylab)
        self.__ax.grid(True)
        self.line_traj = self.__ax.plot(model.wps[:,self.ix], model.wps[:,self.iy])[0]
        ls, Y = model.sample_output()
        self.line_trajsmooth = self.__ax.plot(Y[:,self.ix,0], Y[:,self.iy,0])[0]
        self.markers = [misc_utils.DraggableWP(self.__ax, wp[np.ix_([self.ix,self.iy])], 0.2, f'{i+1}', cbk, self) for i,wp in enumerate(model.wps)]
        for m in self.markers: m.connect()

    def redraw(self, wps, Y):
        self.line_traj.set_data(wps[:,self.ix], wps[:,self.iy])
        self.line_trajsmooth.set_data(Y[:,self.ix,0], Y[:,self.iy,0])
        for wp, m in zip(wps, self.markers):
            m.set_position(wp[np.ix_([self.ix,self.iy])])
        self.draw() 

class TopView(ProjView):
   def __init__(self, model, cbk):
        super().__init__(0, 1, 'Top', 'x in m (East)', 'y in m (North)', model, cbk) 

class FrontView(ProjView):
   def __init__(self, model, cbk):
        super().__init__(0, 2, 'Front', 'x in m (East)', 'z in m (Up)', model, cbk) 
            
class RightView(ProjView):
    def __init__(self, model, cbk):
        super().__init__(1, 2, 'Right', 'y in m (North)', 'z in m (Up)', model, cbk)   
        

class Window(QWidget):
    def __init__(self, model, controller): # remove controller arg
        super().__init__()
        self.model, self.controller = model, controller
        layout = QVBoxLayout()
        layout3 = QGridLayout() 
        layout.addLayout(layout3)
        layout3.setContentsMargins(0,0,0,0)
        layout3.setSpacing(0)
        self.tview = TopView(model, self.on_wp_moved)
        self.fview = FrontView(model, self.on_wp_moved)
        self.rview = RightView(model, self.on_wp_moved)
        #self.threeDview = tdvg.ThreeDWidget(model)
        layout3.addWidget(self.tview, 0, 0)
        layout3.addWidget(self.fview, 1, 0)
        layout3.addWidget(self.rview, 1, 1)
        #layout3.addWidget(self.threeDview, 0, 1)
        self.setLayout(layout)

        
    def on_wp_moved(self, view):
        #print('geometry_view::Window::on_wp_moved', view)
        # that sucks, maybe we keep all coordinates in each proj view ?
        proj_wps = np.array([m.get_center() for m in view.markers])
        wps = self.model.wps
        wps[:,view.ix] = proj_wps[:,0]
        wps[:,view.iy] = proj_wps[:,1]
        self.controller.on_geom_waypoint_moved(wps)
        #self.model.set_waypoints(wps)
        #ls, Y = self.model.sample_output()
        #self._redraw(wps, Y)
        

    def update_plot(self, model):
        wps = self.model.get_waypoints()
        time, Y = self.model.sample_output()
        self.tview.redraw(wps, Y)
        self.fview.redraw(wps, Y)
        self.rview.redraw(wps, Y)
        
        
    # def redraw(self):
    #     wps = self.model.get_waypoints()
    #     time, Y = self.model.sample_output()
    #     self._redraw(wps, Y)
        
    # def _redraw(self, wps, Y):
    #     self.tview.redraw(wps, Y)
    #     self.fview.redraw(wps, Y)
    #     self.rview.redraw(wps, Y)
    #     #self.threeDview.redraw(wps, Y)
        
