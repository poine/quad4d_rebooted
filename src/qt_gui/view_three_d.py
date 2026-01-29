import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
import pyqtgraph.opengl as gl
from PySide6.QtGui import (QVector3D)

import stl #numpy-stl

import numpy as np

class ThreeDWidget(gl.GLViewWidget):
    def __init__(self, model):
        super().__init__()
        self.setCameraPosition(distance=20)
        grid_size = QVector3D(10,10, 1)
        gx = gl.GLGridItem(grid_size)
        gx.rotate(90, 0, 1, 0)
        gx.translate(-5, 0, 5)
        self.addItem(gx)
        gy = gl.GLGridItem(grid_size)
        gy.rotate(90, 1, 0, 0)
        gy.translate(0, -5, 5)
        self.addItem(gy)
        gz = gl.GLGridItem(grid_size)
        self.addItem(gz)

        if model is not None:
            self.draw_trajectory(model)

        # https://github.com/mathworks/quadcopter-simulation-ros-gazebo/blob/master/Gazebo/meshes/quadrotor/quadrotor_2.stl
        m = stl.mesh.Mesh.from_file('media/quadrotor_2.stl')
        md = gl.MeshData(m.vectors)
        self.mi = gl.GLMeshItem(meshdata=md, color=(0.17, 0.63, 0.17, 1))
        self.addItem(self.mi)

        self.add_triedra('World (ENU)')

    def add_triedra(self, txt='', l=0.5, transform=np.eye(4)):
        _parent = gl.GLGraphicsItem.GLGraphicsItem()
        _parent.setTransform(transform)
        poss = [[[0,0,0], [l,0,0]], [[0,0,0], [0,l,0]], [[0,0,0], [0,0,l]]]
        colors = [(1,0,0,1), (0,1,0,1), (0,0,1,1)]
        for pos, col in zip(poss, colors):
            l = gl.GLLinePlotItem(_parent, pos=pos, color=col, width=3)
            #self.addItem(l)

        #glaxis = gl.GLAxisItem() # no luck
        #self.addItem(glaxis)
            
        t = gl.GLTextItem(_parent, pos=(0,0,0), text=txt)
        #self.addItem(t)
        self.addItem(_parent)
        
    def draw_trajectory(self, model):
        wps = model.wps
        size = 0.2*np.ones(len(wps))
        color = np.empty((len(wps), 4))
        color[:,0] = 1.; color[:,3] = 1
        self.wps = gl.GLScatterPlotItem(pos=wps, size=size, color=color, pxMode=False)
        self.addItem(self.wps)
        self.wp_texts = [gl.GLTextItem(pos=wp, text=f'{i+1}') for i, wp in enumerate(wps)]
        for t in self.wp_texts: self.addItem(t)
        #1f77b4, ff7f0e, 2ca02c
        colors = [(0.12, 0.47, 0.7, 1), (1., 0.5, 0.05, 1), (0.17, 0.63, 0.17, 1)]
        
        self.line_traj = gl.GLLinePlotItem(pos=wps, color=(0.12, 0.47, 0.7, 1), width=2., antialias=True)
        self.addItem(self.line_traj)

        ls, Y = model.sample_output()
        self.line_traj_smoothed = gl.GLLinePlotItem(pos=Y[:,:3,0], color=(1., 0.5, 0.055, 1), width=3., antialias=True)
        self.addItem(self.line_traj_smoothed)

        

        
    def redraw(self, wps, Y): 
        self.wps.setData(pos=wps)
        self.line_traj.setData(pos=wps)
        self.line_traj_smoothed.setData(pos=Y[:,:3,0])

    def _draw_trajectory(self, Y):  # FIXME: merge with redraw... figure waypoints
        try: self.line_traj_smoothed
        except AttributeError:
            self.line_traj_smoothed = gl.GLLinePlotItem(pos=Y[:,:3,0], color=(1., 0.5, 0.055, 1), width=3., antialias=True)
            self.addItem(self.line_traj_smoothed)
            
        self.line_traj_smoothed.setData(pos=Y[:,:3,0])
        
    def draw_quad(self, Y, rmat):
        T = np.eye(4); T[:3,3] = Y[:3,0]
        T[:3,:3] = rmat
        self.mi.setTransform(pg.Transform3D(T))
