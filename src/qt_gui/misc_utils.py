#
#
# Unsorted and temporaty stuff
#
from PySide6.QtWidgets import QWidget 
from PySide6.QtGui import QPalette, QColor


class ColoredWidget(QWidget):
    def __init__(self, color):
        super().__init__()
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setColor(QPalette.ColorRole.Window, QColor(color))
        self.setPalette(palette)
    def redraw(self, wps): pass
       

from matplotlib.figure import Figure
from matplotlib.patches import Circle

 
class DraggableWP:
    lock = None # only one can be dragged at a time
    def __init__(self, ax, pos, radius, label, cbk, cbk_arg):
        self.point = Circle(pos, radius); ax.add_artist(self.point)
        self.txt = ax.text(self.point.center[0], self.point.center[1], label,
                           bbox = dict(boxstyle="circle", fc="lightgrey"),
                           horizontalalignment='center', verticalalignment='center')
        self.cbk, self.cbk_arg = cbk, cbk_arg
        self.press = None
        self.background = None

    def get_center(self):
        return self.txt.get_position()
        
    def connect(self):
        """Connect to all the events we need."""
        self.cidpress = self.point.figure.canvas.mpl_connect(
            'button_press_event', self.on_press)
        self.cidrelease = self.point.figure.canvas.mpl_connect(
            'button_release_event', self.on_release)
        self.cidmotion = self.point.figure.canvas.mpl_connect(
            'motion_notify_event', self.on_motion)
        canvas = self.txt.figure.canvas
        self.cidpress = canvas.mpl_connect('button_press_event', self.on_foo)


    def on_foo(self, event):
        #breakpoint()
        #print('foo', event)
        pass
    
    def on_press(self, event):
        """Check whether mouse is over us; if so, store some data."""
        if (event.inaxes != self.point.axes or DraggableWP.lock is not None):
            return
        contains, attrd = self.point.contains(event)
        if not contains:
            return
        #print('event contains', self.point.xy)
        self.press = self.point.center, (event.xdata, event.ydata)
        DraggableWP.lock = self

        # draw everything but the selected rectangle and store the pixel buffer
        canvas = self.point.figure.canvas
        axes = self.point.axes
        self.point.set_animated(True)
        canvas.draw()
        self.background = canvas.copy_from_bbox(self.point.axes.bbox)

        # now redraw just the rectangle
        axes.draw_artist(self.point)

        # and blit just the redrawn area
        canvas.blit(axes.bbox)

    def on_motion(self, event):
        """Move the rectangle if the mouse is over us."""
        if (event.inaxes != self.point.axes
                or DraggableWP.lock is not self):
            return
        (x0, y0), (xpress, ypress) = self.press
        self.point.set(center=(event.xdata, event.ydata))

        canvas = self.point.figure.canvas
        axes = self.point.axes
        # restore the background region
        canvas.restore_region(self.background)

        # redraw just the current rectangle
        axes.draw_artist(self.point)

        # blit just the redrawn area
        canvas.blit(axes.bbox)

    def on_release(self, event):
        """Clear button press information."""
        if DraggableWP.lock is not self:
            return

        self.press = None
        DraggableWP.lock = None

        # turn off the rect animation property and reset the background
        self.point.set_animated(False)
        self.background = None
        #
        # FIXME
        self.txt.set_position(self.point.get_center())
        
        # redraw the full figure
        self.point.figure.canvas.draw()
        self.cbk(self.cbk_arg)
   
    def disconnect(self):
        """Disconnect all callbacks."""
        self.point.figure.canvas.mpl_disconnect(self.cidpress)
        self.point.figure.canvas.mpl_disconnect(self.cidrelease)
        self.point.figure.canvas.mpl_disconnect(self.cidmotion)

    def set_position(self, pos):
        self.txt.set_position(pos)
        self.point.set(center=pos)
   
