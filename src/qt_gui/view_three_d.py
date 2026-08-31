import logging
import numpy as np, stl #numpy-stl

from PySide6.QtCore import Qt
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
import pyqtgraph.opengl as gl
from PySide6.QtGui import (QFont, QVector3D)

logger = logging.getLogger(__name__)

VOLIERE_FLOOR = ((-5.5, 5.5), (-7.0, 7.0))

class ThreeDWidget(gl.GLViewWidget):
    def __init__(self, model=None):
        super().__init__()
        self.setCameraPosition(distance=20)

        self.scene_items={}
        self.build_grid(model)
        self.build_floor()
        if model is not None:
            self.build_arena(model)
            self.arena = FlightArena(self, model.arena)
        self.build_frames()

        defaults = [('grid', True), ('floor', True), ('arena', False) , ('frames', False)]
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


    def display_new_trajectory(self, model, idx=0, show_details=True, show_super_details=False,
                               show_quad=True, show_ref_quad=False, show_ref_traj=True):
        logger.debug('in display_new_trajectory')
        trj = TrajItem(model.get_trajectory(idx), self, idx, show_details, show_super_details, show_quad, show_ref_quad, show_ref_traj)
        if idx < len(self.traj_items):
            self.traj_items[idx].remove(self)
            self.traj_items[idx] = trj
        else:
            self.traj_items.append(trj)
        
    def update_plot(self, model, idx=0): 
        logger.debug('in update_trajectory')
        self.traj_items[idx].update(model.get_trajectory(idx))

    def set_trajectories(self, model, show_details=False, show_quad=True, show_ref_quad=True,
                         show_ref_traj=True):
        n_new = model.trajectory_nb()
        for i in range(n_new):
            self.display_new_trajectory(model, i, show_details=show_details,
                                        show_quad=show_quad, show_ref_quad=show_ref_quad,
                                        show_ref_traj=show_ref_traj)
        while len(self.traj_items) > n_new:
            self.traj_items.pop().remove(self)
 

    def set_quad_pose(self, Tenu2flu, idx=0):
        self.traj_items[idx].set_quad_pose(Tenu2flu)
    def set_ref_pose(self, Tenu2flu, idx=0):
        self.traj_items[idx].set_ref_pose(Tenu2flu)
    def update_vehicle_traj(self, Ys, idx=0):
        self.traj_items[idx].update_vehicle_traj(Ys)
 
    def build_grid(self, model): # FIXME: needs love
        extends = model.arena.extends if model is not None else ((-5, 5), (-5, 5), (0, 10.))
        grid_item = gl.GLGraphicsItem.GLGraphicsItem()
        # all three grids sized to the volière plan (1 m cells, centred), so
        # the two vertical walls end exactly on the floor's edges and their
        # cells line up with the plan's 1 m blocks
        (fx0, fx1), (fy0, fy1) = VOLIERE_FLOOR
        Lx, Ly, Hz = fx1 - fx0, fy1 - fy0, 10.
        gx = gl.GLGridItem(QVector3D(Hz, Ly, 1), parentItem=grid_item)  # y-z wall at x_min
        gx.rotate(90, 0, 1, 0)
        gx.translate(fx0, 0, Hz / 2)
        gy = gl.GLGridItem(QVector3D(Lx, Hz, 1), parentItem=grid_item)  # x-z wall at y_min
        gy.rotate(90, 1, 0, 0)
        gy.translate(0, fy0, Hz / 2)
        gz = gl.GLGridItem(QVector3D(Lx, Ly, 1), parentItem=grid_item)  # floor
        self.addItem(grid_item)
        self.scene_items['grid'] = grid_item
        
    def build_arena(self, model):
        arena_item = gl.GLGraphicsItem.GLGraphicsItem()
        e = (xm,xM), (ym,yM), (zm,zM) = model.arena.extends
        poss = [[[xm,ym,zm], [xM,ym,zm]], [[xm,yM,zm], [xM,yM,zm]], [[xm,ym,zm], [xm,yM,zm]], [[xM,ym,zm], [xM,yM,zm]],
                [[xm,ym,zM], [xM,ym,zM]], [[xm,yM,zM], [xM,yM,zM]], [[xm,ym,zM], [xm,yM,zM]], [[xM,ym,zM], [xM,yM,zM]],
                [[xm,ym,zm], [xm,ym,zM]], [[xM,ym,zm], [xM,ym,zM]], [[xm,yM,zm], [xm,yM,zM]], [[xM,yM,zm], [xM,yM,zM]],
                ]
        col=[1,0,0,1]
        for pos in poss: gl.GLLinePlotItem(arena_item, pos=pos, color=col, width=1)
        self.addItem(arena_item)
        self.scene_items['arena'] = arena_item


    def build_floor(self):
        """Lay the volière plan (media/voliere_plan.png, cropped to the room)
        flat on the ground (z=0), scaled to the cage. Alignment is by cage
        dimensions: the image spans VOLIERE_FLOOR in ENU metres -- tune those
        to line the plan up with the real cage."""
        import os
        path = os.path.join(os.path.dirname(__file__), 'media', 'voliere_plan.png')
        if not os.path.exists(path):
            return
        img = mpimg.imread(path)                       # (H, W, 3|4), float [0,1]
        if img.ndim == 2:
            img = np.dstack([img] * 3)
        if img.shape[2] == 3:                          # add opaque alpha
            img = np.dstack([img, np.ones(img.shape[:2], dtype=img.dtype)])
        tex = (img * 255).astype(np.ubyte)
        # GLImageItem data is indexed [x, y]: put image cols along x, rows
        # along y, and flip rows so the image top maps to +y (north)
        data = np.ascontiguousarray(tex[::-1].transpose(1, 0, 2))
        floor = gl.GLImageItem(data)
        (x0, x1), (y0, y1) = VOLIERE_FLOOR
        W, H = data.shape[0], data.shape[1]
        floor.scale((x1 - x0) / W, (y1 - y0) / H, 1.)
        floor.translate(x0, y0, 0.)
        self.addItem(floor)
        self.scene_items['floor'] = floor

    def build_frames(self):
        frames_item = gl.GLGraphicsItem.GLGraphicsItem()
        self.addItem(frames_item)
        if 0:
            T_enu2ned = np.array([[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            self.build_triedra(frames_item, 'World (NED)', 0.75, T_enu2ned)
        else:
            self.build_triedra(frames_item, 'World (ENU)')
        try:
            for g in self.arena.gate_items:
                self.build_triedra(frames_item, g.name, 0.25, g.transform())
        except AttributeError: #
            pass
        self.scene_items['frames'] = frames_item

#
# Flight arena
#
import matplotlib.image as mpimg

class Gate(gl.GLGraphicsItem.GLGraphicsItem):
    def __init__(self, parent, Tenu2flu, name, texture, dims, draw_contour=False):
        super().__init__()
        parent.addItem(self)
        self.setTransform(pg.Transform3D(Tenu2flu))
        self.name = name

        (w1,h1),(w2,h2) = dims
        if draw_contour:
            poss = 0.5*np.array([[[0,w1,h1],[0,-w1,h1],[0,-w1,-h1],[0,w1,-h1],[0,w1,h1]],
                                 [[0,w2,h2],[0,-w2,h2],[0,-w2,-h2],[0,w2,-h2],[0,w2,h2]]])
            col=[1,1,1,1]
            for pos in poss: gl.GLLinePlotItem(self, pos=pos, color=col, width=1, mode='line_strip')
        
        image = mpimg.imread(texture)
        texture = (image*255).astype(int)
        item = gl.GLImageItem(texture, parentItem=self)
        sx, sy = w1/image.shape[0], h1/image.shape[1]
        item.scale(sx, sy, 1.)
        item.rotate(90, 0,1,0)
        item.translate(0, -h1/2, w1/2)
            
class FlightArena:
    def __init__(self, parent, arena):
        self.gate_items = []
        for i, g in enumerate(arena.gates):
            self.gate_items.append(Gate(parent, g['pose'], g['name'], g['texture'], g['dim']))

    def get_frames(self): return [g.transform() for g in self.gate_items]

#
# 3D representation of a trajectory (reference and real tracks, reference and real quads, waypoints)
#
class TrajItem:
    _colors = [(1.0  , 0.83, 0.0  , 1),   # yellow      #FFD400
               (0.0  , 0.90, 0.46 , 1),   # bright green #00E676
               (0.835, 0.0 , 0.976, 1)]   # magenta/violet #D500F9
    
    def __init__(self, traj, parent, idx, show_details, show_super_details, show_quad=False, show_ref_quad=False, show_ref_traj=True):
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
        self.show_quad(show_quad)
        self.ref_quad_item = gl.GLMeshItem(meshdata=md, color=my_color_faded, edgeColor=my_color_faded,
                                           drawEdges=True, drawFaces=False )
        parent.addItem(self.ref_quad_item)
        self.show_ref_quad(show_ref_quad)
        
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
        self.ref_traj_line_item.setVisible(show_ref_traj)
        self.traj_line_item = gl.GLLinePlotItem(pos=np.zeros((1,3)), color=my_color_faded, width=2., antialias=True)
        parent.addItem(self.traj_line_item)

    def update(self, traj):
        if self.waypoints_item is not None:
            wps = traj.get_waypoints()
            self.waypoints_item.setData(pos=wps)
            for wp, wpi in zip(wps, self.waypoints_text_items): wpi.setData(pos=wp)
            if self.waypoints_line_item is not None: self.waypoints_line_item.setData(pos=wps)
        time = np.linspace(0, traj.duration, 1000)
        Ys = np.array([traj.get(t) for t in time])
        self.ref_traj_line_item.setData(pos=Ys[:,:3,0])

    def update_ref_traj(self, traj): return self.update(traj)  # FIXME: update that :)
    def update_vehicle_traj(self, Ys):
        self.traj_line_item.setData(pos=Ys)

        
    def remove(self, parent):
        if self.waypoints_item is not None: parent.removeItem(self.waypoints_item)
        if self.waypoints_line_item is not None: parent.removeItem(self.waypoints_line_item)
        if self.waypoints_text_items is not None:
            for it in self.waypoints_text_items: parent.removeItem(it)
        parent.removeItem(self.ref_traj_line_item)
        if self.traj_line_item is not None: parent.removeItem(self.traj_line_item)
        parent.removeItem(self.quad_item)
        parent.removeItem(self.ref_quad_item)

    def set_quad_pose(self, Tenu2flu):
        self.quad_item.setTransform(pg.Transform3D(Tenu2flu))
    def set_ref_pose(self, Tenu2flu):
        self.ref_quad_item.setTransform(pg.Transform3D(Tenu2flu))
    def show_quad(self, v): self.quad_item.setVisible(v)
    def show_ref_quad(self, v): self.ref_quad_item.setVisible(v)
