
#
# Unsorted and/or temporary stuff
#
import numpy as np
from PySide6.QtWidgets import QWidget 
from PySide6.QtGui import QPalette, QColor

import pat3.utils as p3_u, pat3.algebra as p3_al
import pat3.vehicles.rotorcraft.multirotor_trajectory as p3_mt
import pat3.vehicles.rotorcraft.multirotor_fdm as p3_mfdm


def pos_of_T(T): return T[:3,3]

# pat uses ned... all the following is sketchy...
def Yenu2ned(Yenu):
    Yned = np.zeros_like(Yenu)
    Yned[p3_mt._x]   =  Yenu[p3_mt._y] 
    Yned[p3_mt._y]   =  Yenu[p3_mt._x] 
    Yned[p3_mt._z]   = -Yenu[p3_mt._z]
    Yned[p3_mt._psi,0]  = p3_u.norm_mpi_pi(np.pi/2-Yenu[p3_mt._psi,0])
    Yned[p3_mt._psi,1:] = -Yenu[p3_mt._psi,1:]
    return Yned

def Yned2enu(Yned): return Yenu2ned(Yned) # that's symetrical

def Tenu2fru_of_Xned(Xned):
    T = np.eye(4)
    T[:3,3] = Xned[1], Xned[0], -Xned[2]
    rmat_ned2frd = p3_al.rmat_of_quat(Xned[p3_mfdm.sv_slice_quat])
    rmat_enu2ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    rmat_frd2flu = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    rmat_enu2flu = rmat_frd2flu@rmat_ned2frd@rmat_enu2ned
    rmat_enu2flu = rmat_enu2flu.T #FIXME : rmat_enu2flu is really rmat_flu2enu ???
    T[:3,:3] = rmat_enu2flu
    return T

def euler_enu_of_ned(eulers_ned):  # FIXME: check that assumption
    eulers_enu = np.zeros_like(eulers_ned)
    eulers_enu[:,0] = -eulers_ned[:,0]
    eulers_enu[:,1] = -eulers_ned[:,1]
    eulers_enu[:,2] = np.pi/2 - eulers_ned[:,2] 
    return eulers_enu


def rmat_enu2flu_of_euler_ned2frd(euler_ned2frd):
    rmat_ned2frd = p3_al.rmat_of_euler(euler_ned2frd)
    rmat_enu2ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    rmat_frd2flu = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    rmat_enu2flu = rmat_frd2flu@rmat_ned2frd@rmat_enu2ned
    rmat_enu2flu = rmat_enu2flu.T #FIXME : rmat_enu2flu is really rmat_flu2enu ???
    return rmat_enu2flu

# for layout testing
class ColoredWidget(QWidget):
    def __init__(self, color):
        super().__init__()
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setColor(QPalette.ColorRole.Window, QColor(color))
        self.setPalette(palette)
    def redraw(self, wps): pass

#
# Matplotlib related stuff
#

def ensure_min_ylim(ax, span=0.1):
    ax.autoscale()
    ymin, ymax = ax.get_ylim()
    if ymax-ymin < span:
        c = (ymin+ymax)/2
        ymin, ymax = c-span/2, c+span/2
        ax.set_ylim(ymin, ymax)

# I can't seem to get axis.autoscale/axis.relim to work :(
def autoscale_axis(ax, px, py, mg=0.05, min_span=0.1):
    xm, xM, ym, yM = np.min(px), np.max(px), np.min(py), np.max(py)
    #dx, dy = np.max(min_span, xM-xm), np.max(min_span, yM-ym) # wtf!!!!
    dx, dy = max(min_span, xM-xm), max(min_span, yM-ym)
    xm-=mg*dx; xM+=mg*dx; ym-=mg*dy; yM+=mg*dy 
    ax.set(xlim=(xm, xM), ylim=(ym, yM))

def decorate(ax, title=None, xlab=None, ylab=None, legend=None, grid=None):
    if title is not None: ax.set_title(title)
    if xlab  is not None: ax.xaxis.set_label_text(xlab)
    if ylab  is not None: ax.yaxis.set_label_text(ylab)
    if legend is not None and len(ax.lines)>0: ax.legend()
    if grid is not None: ax.grid(True)
    
#
# Markers that can be mouse-dragged, do we want keyboard arrow manipulations?
# We're trying to be smart by bliting the canvas
#
class DraggableMarker:
    lock = None # only one marker can be dragged at a time
    def __init__(self, ax, pos, label, cbk, cbk_arg, ylims=None, autoconnect=True):
        self.txt = ax.text(pos[0], pos[1], label,
                           bbox = dict(boxstyle="circle", fc="lightgrey"),
                           horizontalalignment='center', verticalalignment='center')
        self.cbk, self.cbk_arg = cbk, cbk_arg
        self.ylims = ylims
        self.background = None
        if autoconnect: self.connect()

    def remove(self): self.txt.remove()
        
    def get_center(self): return self.txt.get_position()
        
    def connect(self):
        events = ['button_press_event', 'button_release_event', 'motion_notify_event']
        cbks = [self.on_press, self.on_release, self.on_motion]
        self.cids = [self.txt.figure.canvas.mpl_connect(e,c) for e,c in zip(events, cbks)]

    def disconnect(self):
        for cid in self.cids: self.txt.figure.canvas.mpl_disconnect(self.cidpress)
        
    def on_press(self, event):
        if (event.inaxes != self.txt.axes or DraggableMarker.lock is not None):
            return
        if not self.txt.contains(event)[0]: return
        DraggableMarker.lock = self
        # draw everything but the selected marker and store the pixel buffer
        canvas, axes = self.txt.figure.canvas, self.txt.axes
        self.txt.set_animated(True)
        canvas.draw()
        self.background = canvas.copy_from_bbox(axes.bbox)
        # now redraw just the marker
        axes.draw_artist(self.txt)
        # and blit just the redrawn area
        canvas.blit(axes.bbox)

    def on_motion(self, event):
        if (event.inaxes != self.txt.axes or DraggableMarker.lock is not self):
            return
        pos = (event.xdata, event.ydata) if self.ylims is None else (event.xdata, np.clip(event.ydata, *self.ylims))
        self.txt.set_position(pos)
        canvas, axes = self.txt.figure.canvas, self.txt.axes
        # restore the background region
        canvas.restore_region(self.background)
        # redraw just the current marker
        axes.draw_artist(self.txt)
        # blit just the redrawn area
        canvas.blit(axes.bbox)

    def on_release(self, event):
        if DraggableMarker.lock is not self:
            return
        DraggableMarker.lock = None
        # turn off the animation property and reset the background
        self.txt.set_animated(False)
        self.background = None
        # redraw the full figure
        self.txt.figure.canvas.draw()
        self.cbk(self.cbk_arg) # notify we've been moved

    def set_position(self, pos): self.txt.set_position(pos)
   
