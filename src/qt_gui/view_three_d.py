import logging
import numpy as np, stl #numpy-stl

from PySide6.QtCore import Qt
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
import pyqtgraph.opengl as gl
from PySide6.QtGui import (QFont, QVector3D)

logger = logging.getLogger(__name__)



class ThreeDWidget(gl.GLViewWidget):
    def __init__(self, model=None):
        super().__init__()
        self.setCameraPosition(distance=20)

        self.scene_items={}
        self.build_grid(model)
        self.build_frames()
        if model is not None: self.build_arena(model)

        defaults = [('grid', True), ('arena', False) , ('frames', False)]
        for k, s in defaults:
            try: self.set_item_visible(k,s)
            except KeyError: pass

        self.traj_items = []
        

    def set_item_visible(self, what, state): self.scene_items[what].setVisible(state)
    def is_item_visible(self, what): return self.scene_items[what].visible()
    def show_quad(self, v, idx=0): self.traj_items[idx].show_quad(v)
        
    def build_triedra(self, parent, txt='', l=0.5, transform=np.eye(4)):
        frame_item = gl.GLGraphicsItem.GLGraphicsItem(parent)
        frame_item.setTransform(transform)
        #glaxis = gl.GLAxisItem() # no luck
        poss = [[[0,0,0], [l,0,0]], [[0,0,0], [0,l,0]], [[0,0,0], [0,0,l]]]
        colors = [(1,0,0,1), (0,1,0,1), (0,0,1,1)]
        for pos, col in zip(poss, colors):
            gl.GLLinePlotItem(frame_item, pos=pos, color=col, width=3)
        gl.GLTextItem(frame_item, pos=(0,0,l/2), text=txt, alignment=Qt.AlignCenter, font=QFont('Helvetica', 10))


    def display_new_trajectory(self, model, idx=0, show_details=True, show_super_details=False):
        logger.debug('in display_new_trajectory')
        trj = TrajItem(model.get_trajectory(idx), self, idx, show_details, show_super_details)
        if idx < len(self.traj_items):
            self.traj_items[idx].remove(self)
            self.traj_items[idx] = trj
        else:
            self.traj_items.append(trj)
        
    def update_plot(self, model, idx=0): 
        logger.debug('in update_trajectory')
        self.traj_items[idx].update(model.get_trajectory())

    def set_quad_pose(self, Tenu2flu, idx=0):
        self.traj_items[idx].set_quad_pose(Tenu2flu)
    def set_ref_pose(self, Tenu2flu, idx=0):
        self.traj_items[idx].set_ref_pose(Tenu2flu)
    def update_vehicle_traj(self, Ys, idx=0):
        self.traj_items[idx].update_vehicle_traj(Ys)
 
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
        if 0:
            T_enu2ned = np.array([[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            self.build_triedra(frames_item, 'World (NED)', 0.75, T_enu2ned)
        else:
            self.build_triedra(frames_item, 'World (ENU)')
        self.scene_items['frames'] = frames_item
        

class TrajItem:
    _colors = [(0.12, 0.47, 0.7 , 1),
               (1.  , 0.5, 0.055, 1),
               (0.17, 0.63, 0.17, 1)]
    
    def __init__(self, traj, parent, idx, show_details, show_super_details):
        self.waypoints_item = None
        self.waypoints_text_items = None
        self.waypoints_line_item = None
        self.traj_line_item = None
        self.quad_item = None
        self.real_quad_item = None
        self.real_line_item = None
        

        my_color = list(self._colors[idx])
        my_color_faded = list(self._colors[idx]); my_color_faded[3]=0.5 
        # quadrotor
        m = stl.mesh.Mesh.from_file('media/quadrotor_2.stl')
        md = gl.MeshData(m.vectors)
        self.quad_item = gl.GLMeshItem(meshdata=md, color=my_color)
        parent.addItem(self.quad_item)
        self.quad_item.setVisible(False)
        self.ref_quad_item = gl.GLMeshItem(meshdata=md, color=my_color_faded, edgeColor=my_color_faded,
                                           drawEdges=True, drawFaces=False )
        parent.addItem(self.ref_quad_item)
        self.ref_quad_item.setVisible(True)
        
        # waypoints
        if traj.has_waypoints():
            if show_details:
                wps = traj.get_waypoints()
                color = np.empty((len(wps), 4)); color[:,0] = 1.; color[:,3] = 1
                size = 0.1*np.ones(len(wps))
                self.waypoints_item = gl.GLScatterPlotItem(pos=wps, size=size, color=color, pxMode=False)
                parent.addItem(self.waypoints_item)
                _al = Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignBottom
                self.waypoints_text_items = [gl.GLTextItem(pos=wp, text=f'{i+1}', alignment=_al) for i, wp in enumerate(wps)]
                for it in self.waypoints_text_items: parent.addItem(it)
            if show_super_details:
                self.waypoints_line_item = gl.GLLinePlotItem(pos=wps, color=self._colors[idx], width=2., antialias=True)
                parent.addItem(self.waypoints_line_item)

        time = np.linspace(0, traj.duration, 1000)
        Ys = np.array([traj.get(t) for t in time])
        self.ref_traj_line_item = gl.GLLinePlotItem(pos=Ys[:,:3,0], color=my_color, width=3., antialias=True, mode='lines')
        parent.addItem(self.ref_traj_line_item)
        self.traj_line_item = gl.GLLinePlotItem(pos=Ys[:,:3,0], color=my_color_faded, width=2., antialias=True)
        parent.addItem(self.traj_line_item)

    def update(self, traj):
        if self.waypoints_item is not None:
            wps = traj.get_waypoints()
            self.waypoints_item.setData(pos=wps)
            for wp, wpi in zip(wps, self.waypoints_text_items): wpi.setData(pos=wp)
            if self.waypoints_line_item is not None: self.waypoints_line_item.setData(pos=wps)
        time = np.linspace(0, traj.duration, 1000)
        Ys = np.array([traj.get(t) for t in time])
        self.ref_traj_line_item.setData(pos=np.zeros((1,3)))

    def update_ref_traj(self, traj): return self.update(traj)  # FIXME: update that :)
    def update_vehicle_traj(self, Ys):
        self.traj_line_item.setData(pos=Ys)

        
    def remove(self, parent):
        if self.waypoints_item is not None: parent.removeItem(self.waypoints_item)
        if self.waypoints_line_item is not None: parent.removeItem(self.waypoints_line_item)
        if self.waypoints_text_items is not None:
            for it in self.waypoints_text_items: parent.removeItem(it)
        parent.removeItem(self.ref_traj_line_item)
        parent.removeItem(self.quad_item)

    def set_quad_pose(self, Tenu2flu):
        self.quad_item.setTransform(pg.Transform3D(Tenu2flu))
    def set_ref_pose(self, Tenu2flu):
        self.ref_quad_item.setTransform(pg.Transform3D(Tenu2flu))
    def show_quad(self, v): self.quad_item.setVisible(v)
