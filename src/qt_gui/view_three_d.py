from PySide6.QtCore import Qt
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
import pyqtgraph.opengl as gl
from PySide6.QtGui import (QFont, QVector3D)

import stl #numpy-stl

import numpy as np

class ThreeDWidget(gl.GLViewWidget):
    def __init__(self, model=None):
        super().__init__()
        self.setCameraPosition(distance=20)

        self.scene_items={}
        self.build_grid(model)
        self.build_frames()
        if model is not None: # FIXME: remove that shity test
            self.build_arena(model)
            self.waypoints_item = None
            self.waypoints_text_items = None
            self.waypoints_line_item = None
            self.traj_line_item = None
            #self.display_new_trajectory(model)

        defaults = [('grid', True), ('arena', False) , ('frames', False)]
        for k, s in defaults:
            try: self.set_item_visible(k,s)
            except KeyError: pass

        
            
        # https://github.com/mathworks/quadcopter-simulation-ros-gazebo/blob/master/Gazebo/meshes/quadrotor/quadrotor_2.stl
        m = stl.mesh.Mesh.from_file('media/quadrotor_2.stl')
        md = gl.MeshData(m.vectors)
        self.quad_item = gl.GLMeshItem(meshdata=md, color=(0.17, 0.63, 0.17, 1))
        self.addItem(self.quad_item)
        self.show_quad(False)


    def build_grid(self, model): # FIXME: needs love
        extends = model.extends if model is not None else ((-5, 5), (-5, 5), (0, 10.))
        grid_item = gl.GLGraphicsItem.GLGraphicsItem()
        grid_size = QVector3D(10,10, 1)
        gx = gl.GLGridItem(grid_size, parentItem=grid_item)
        gx.rotate(90, 0, 1, 0)
        gx.translate(-5, 0, 5)
        gy = gl.GLGridItem(grid_size, parentItem=grid_item)
        gy.rotate(90, 1, 0, 0)
        gy.translate(0, -5, 5)
        gz = gl.GLGridItem(grid_size, parentItem=grid_item)
        self.addItem(grid_item)
        self.scene_items['grid'] = grid_item
        
    def build_arena(self, model):
        arena_item = gl.GLGraphicsItem.GLGraphicsItem()
        e = (xm,xM), (ym,yM), (zm,zM) = model.extends
        poss = [[[xm,ym,zm], [xM,ym,zm]], [[xm,yM,zm], [xM,yM,zm]], [[xm,ym,zm], [xm,yM,zm]], [[xM,ym,zm], [xM,yM,zm]],
                [[xm,ym,zM], [xM,ym,zM]], [[xm,yM,zM], [xM,yM,zM]], [[xm,ym,zM], [xm,yM,zM]], [[xM,ym,zM], [xM,yM,zM]],
                [[xm,ym,zm], [xm,ym,zM]], [[xM,ym,zm], [xM,ym,zM]], [[xm,yM,zm], [xm,yM,zM]], [[xM,yM,zm], [xM,yM,zM]],
                ]
        col=[1,0,0,1]
        for pos in poss: gl.GLLinePlotItem(arena_item, pos=pos, color=col, width=1)
        self.addItem(arena_item)
        self.scene_items['arena'] = arena_item

    def build_frames(self):
        frames_item = gl.GLGraphicsItem.GLGraphicsItem()
        self.addItem(frames_item)
        self.build_triedra(frames_item, 'World (ENU)')
        self.scene_items['frames'] = frames_item
        
    def set_item_visible(self, what, state):
        self.scene_items[what].setVisible(state)

    def is_item_visible(self, what): return self.scene_items[what].visible()
        
    def show_quad(self, v): self.quad_item.setVisible(v)
        
    def build_triedra(self, parent, txt='', l=0.5, transform=np.eye(4)):
        frame_item = gl.GLGraphicsItem.GLGraphicsItem(parent)
        frame_item.setTransform(transform)
        #glaxis = gl.GLAxisItem() # no luck
        poss = [[[0,0,0], [l,0,0]], [[0,0,0], [0,l,0]], [[0,0,0], [0,0,l]]]
        colors = [(1,0,0,1), (0,1,0,1), (0,0,1,1)]
        for pos, col in zip(poss, colors):
            gl.GLLinePlotItem(frame_item, pos=pos, color=col, width=3)
        gl.GLTextItem(frame_item, pos=(0,0,l/2), text=txt, alignment=Qt.AlignCenter, font=QFont('Helvetica', 10))


    def display_new_trajectory(self, model):
        print(' #3D::display_new_trajectory')

        if self.waypoints_item is not None: self.removeItem(self.waypoints_item)
        if self.waypoints_line_item is not None: self.removeItem(self.waypoints_line_item)
        if self.waypoints_text_items is not None:
            for it in self.waypoints_text_items: self.removeItem(it)
        if self.traj_line_item is not None: self.removeItem(self.traj_line_item)

        _traj = model.get_trajectory()
        if _traj.has_waypoints():
            wps = _traj.get_waypoints()
            color = np.empty((len(wps), 4)); color[:,0] = 1.; color[:,3] = 1
            size = 0.2*np.ones(len(wps))
            self.waypoints_item = gl.GLScatterPlotItem(pos=wps, size=size, color=color, pxMode=False)
            self.addItem(self.waypoints_item)
            self.waypoints_text_items = [gl.GLTextItem(pos=wp, text=f'{i+1}') for i, wp in enumerate(wps)]
            for it in self.waypoints_text_items: self.addItem(it)
            self.waypoints_line_item = gl.GLLinePlotItem(pos=wps, color=(0.12, 0.47, 0.7, 1), width=2., antialias=True)
            self.addItem(self.waypoints_line_item)
        else:
            self.waypoints_item = None
            self.waypoints_text_items = None
            self.waypoints_line_item = None
        _time, Ys = model.sample_output()
        self.traj_line_item = gl.GLLinePlotItem(pos=Ys[:,:3,0], color=(1., 0.5, 0.055, 1), width=2., antialias=True)
        self.addItem(self.traj_line_item)
        
    #def draw_trajectory(self, model):
        #wps = model.wps
        #size = 0.2*np.ones(len(wps))
        #color = np.empty((len(wps), 4))
        #color[:,0] = 1.; color[:,3] = 1
        #self.wps = gl.GLScatterPlotItem(pos=wps, size=size, color=color, pxMode=False)
        #self.addItem(self.wps)
        #self.wp_texts = [gl.GLTextItem(pos=wp, text=f'{i+1}') for i, wp in enumerate(wps)]
        #for t in self.wp_texts: self.addItem(t)
        #1f77b4, ff7f0e, 2ca02c
        #colors = [(0.12, 0.47, 0.7, 1), (1., 0.5, 0.05, 1), (0.17, 0.63, 0.17, 1)]
        
        #self.line_traj = gl.GLLinePlotItem(pos=wps, color=(0.12, 0.47, 0.7, 1), width=2., antialias=True)
        #self.addItem(self.line_traj)

        #ls, Y = model.sample_output()
        #self.line_traj_smoothed = gl.GLLinePlotItem(pos=Y[:,:3,0], color=(1., 0.5, 0.055, 1), width=2., antialias=True)
        #self.addItem(self.line_traj_smoothed)
        
    def update_trajectory(self, model): 
        print(' #3D::update_trajectory')
        if self.waypoints_item is not None:
            wps = model.get_waypoints()
            self.waypoints_item.setData(pos=wps)
            for wp, wpi in zip(wps, self.waypoints_text_items): wpi.setData(pos=wp)
            self.waypoints_line_item.setData(pos=wps)
        time, Y = model.sample_output()
        self.traj_line_item.setData(pos=Y[:,:3,0])

    # def _draw_trajectory(self, Y):  # FIXME: merge with redraw... figure waypoints
    #     try: self.line_traj_smoothed
    #     except AttributeError:
    #         self.line_traj_smoothed = gl.GLLinePlotItem(pos=Y[:,:3,0], color=(1., 0.5, 0.055, 1), width=3., antialias=True)
    #         self.addItem(self.line_traj_smoothed)
            
    #     self.line_traj_smoothed.setData(pos=Y[:,:3,0])
        
    def set_quad_pose(self, Y, rmat):
        T = np.eye(4); T[:3,3] = Y[:3,0]
        T[:3,:3] = rmat
        self.quad_item.setTransform(pg.Transform3D(T))
