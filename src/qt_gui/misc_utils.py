#
#
# Unsorted and temporaty stuff
#
import numpy as np
from PySide6.QtWidgets import QWidget 
from PySide6.QtGui import QPalette, QColor

# I can't seem to get axis.autoscale to work :(
# was missing ax.relim(). Temporarily Keeping that for reference
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
    if legend is not None: ax.legend()
    if grid is not None: ax.grid(True)
    
class ColoredWidget(QWidget):
    def __init__(self, color):
        super().__init__()
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setColor(QPalette.ColorRole.Window, QColor(color))
        self.setPalette(palette)
    def redraw(self, wps): pass
       
#
# Matplolib markers that can be mouse-dragged
#
class DraggableMarker:
    lock = None # only one can be dragged at a time
    def __init__(self, ax, pos, label, cbk, cbk_arg, ylims=None, autoconnect=True):
        self.txt = ax.text(pos[0], pos[1], label,
                           bbox = dict(boxstyle="circle", fc="lightgrey"),
                           horizontalalignment='center', verticalalignment='center')
        self.cbk, self.cbk_arg = cbk, cbk_arg
        self.ylims = ylims
        self.background = None
        if autoconnect: self.connect()

    def remove(self):
        self.txt.remove()
        
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
        # draw everything but the selected rectangle and store the pixel buffer
        canvas, axes = self.txt.figure.canvas, self.txt.axes
        self.txt.set_animated(True)
        canvas.draw()
        self.background = canvas.copy_from_bbox(axes.bbox)
        # now redraw just the rectangle
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
        # redraw just the current rectangle
        axes.draw_artist(self.txt)
        # blit just the redrawn area
        canvas.blit(axes.bbox)

    def on_release(self, event):
        if DraggableMarker.lock is not self:
            return
        DraggableMarker.lock = None
        # turn off the rect animation property and reset the background
        self.txt.set_animated(False)
        self.background = None
        # redraw the full figure
        self.txt.figure.canvas.draw()
        self.cbk(self.cbk_arg)

    def set_position(self, pos): self.txt.set_position(pos)
   
