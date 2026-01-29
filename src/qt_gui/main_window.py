#
# Main window
# (menus and status bar)
#

from PySide6.QtCore import Qt, QSize #, Signal, Slot
from PySide6.QtGui import QAction, QIcon, QCursor, Qt, QColor, QPalette
from PySide6.QtWidgets import (QMainWindow, QMenu, QVBoxLayout,
                               QWidget, QLabel, QToolBar, QDialog,
                               QLineEdit, QStatusBar, QCheckBox, QToolButton,
                               QPushButton)

import numpy as np
from functools import partial
#from functools import partial # Python is such a sad sad language. World is such a sad sad world. There's no such thing as God
import traceback

import view_chronograms as view_chrono
import view_geometry as view_geom
import view_three_d
import traj_factory


class ExportCsvDialog(QDialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.setWindowTitle("Export to CSV")
        self.edit = QLineEdit("/tmp/foo.csv")
        self.button_export = QPushButton("Export")
        layout = QVBoxLayout()
        layout.addWidget(self.edit)
        layout.addWidget(self.button_export)
        self.setLayout(layout)
        self.button_export.clicked.connect(parent.on_export_to_csv)

class OptimizeDialog(QDialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.setWindowTitle("Export to CSV")
        self.max_vel_edit = QLineEdit("3.")
        self.max_acc_edit = QLineEdit("8.")
        self.button_optimize = QPushButton("Optimize")
        layout = QVBoxLayout()
        layout.addWidget(self.max_vel_edit)
        layout.addWidget(self.max_acc_edit)
        layout.addWidget(self.button_optimize)
        self.setLayout(layout)
        self.button_optimize.clicked.connect(parent.on_optimize)
         
class MainWindow(QMainWindow):
    def __init__(self, model, controller):
        super().__init__()
        self.setWindowTitle('Click\'n Fly')
        self.resize(1280, 1024)
        self.model, self.controller = model, controller

        self.threed_view = view_three_d.ThreeDWidget(model)
        self.setCentralWidget(self.threed_view)
        
        self.geometry_window = view_geom.Window(model, controller)
        self.geometry_window.closed.connect(partial(self.on_child_closed, which="geometry")) # that sucks, i need the checkbutton
        
        self.output_chronogram_window = view_chrono.ChronogramWindow(view_chrono.OutputChronogram, 'Output Chronograms')
        self.output_chronogram_window.closed.connect(partial(self.on_child_closed, which="output_chrono"))
        
        # model and controller not passed to constructor...
        #self.space_indexed_window = view_chrono.ChronogramWindow(view_chrono.SpaceIndexedChronogram, 'Space Indexed Chronograms')
        self.space_idx_chronogram_window = view_chrono.SiChronoWindow(self.model, self.controller)
        self.space_idx_chronogram_window.closed.connect(partial(self.on_child_closed, which="si_chrono"))

        self.state_chronogram_window = view_chrono.ChronogramWindow(view_chrono.StateChronogram, 'State Chronograms')
        self.state_chronogram_window.closed.connect(partial(self.on_child_closed, which="state_chrono"))

        self.build_window(model, controller)
        self.display_new_trajectory(model)
        
    def on_child_closed(self, which):
        print('#main_window::on_child_closed', which)
        self.view_actions[which].setChecked(False)
        
    def build_window(self, model, controller):
        toolbar = QToolBar("My main toolbar")
        toolbar.setIconSize(QSize(16, 16))
        self.addToolBar(toolbar)

        button_animate = QAction(QIcon("bug.png"), "Animate", self)
        #button_animate = QToolButton(QIcon("bug.png"), "Animate", self)   
        #button_animate.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        button_animate.setCheckable(True)
        button_animate.setStatusTip("Show trajecory animation")
        button_animate.triggered.connect(self.controller.toggle_animate)
        toolbar.addAction(button_animate)

        menu = self.menuBar()
        self.build_trajectory_menu(menu)
        self.build_view_menu(menu)
        
        self.setStatusBar(QStatusBar(self))

    def build_trajectory_menu(self, menu):
        traj_menu = menu.addMenu("&Trajectory")
        traj_submenu = traj_menu.addMenu("Load from factory")  # list trajectories available in factory
        for traj in traj_factory.TrajFactory.trajectories.keys():
            action = QAction(traj, self)
            action.triggered.connect(partial(self.controller.load_from_factory, which=traj))
            traj_submenu.addAction(action)
        action_export_to_csv = QAction("Export to CSV", self)
        action_export_to_csv.triggered.connect(self.show_csv_export_dialog)
        traj_menu.addAction(action_export_to_csv)

        action_optimize = QAction("Optimize", self)
        action_optimize.triggered.connect(self.show_optimize_dialog)
        traj_menu.addAction(action_optimize)

        
    def build_view_menu(self, menu):
        view_menu = menu.addMenu("&View")

        three_d_submenu = view_menu.addMenu("3D view")

        action = QAction("Show Grid", self)
        action.setCheckable(True); action.setChecked(self.threed_view.is_item_visible('grid'))
        action.triggered.connect(partial(self.on_three_d_set_visible, src=action, what='grid'))
        three_d_submenu.addAction(action)

        action = QAction("Show Arena Boundaries", self)
        action.setCheckable(True); action.setChecked(self.threed_view.is_item_visible('arena'))
        action.triggered.connect(partial(self.on_three_d_set_visible, src=action, what='arena'))
        three_d_submenu.addAction(action)

        action = QAction("Show Frames", self)
        action.setCheckable(True); action.setChecked(self.threed_view.is_item_visible('frames'))
        action.triggered.connect(partial(self.on_three_d_set_visible, src=action, what='frames'))
        three_d_submenu.addAction(action)
        
        self.view_actions = {}
        button_view_geometry = QAction("View Geometry", self)
        button_view_geometry.setCheckable(True)
        button_view_geometry.triggered.connect(self.show_geometry)
        self.view_actions['geometry'] = button_view_geometry
        view_menu.addAction(button_view_geometry)

        button_view_output_chronogram = QAction("View Output Chronogram", self)
        button_view_output_chronogram.setCheckable(True)
        button_view_output_chronogram.triggered.connect(self.show_output_chronogram)
        self.view_actions['output_chrono'] = button_view_output_chronogram
        view_menu.addAction(button_view_output_chronogram)

        button_view_si_chronogram = QAction("View Space Indexed Chronogram", self)
        button_view_si_chronogram.setCheckable(True)
        button_view_si_chronogram.triggered.connect(self.show_si_chronogram)
        self.view_actions['si_chrono'] = button_view_si_chronogram
        view_menu.addAction(button_view_si_chronogram)

        button_view_state_chronogram = QAction("View State Chronogram", self)
        button_view_state_chronogram.setCheckable(True)
        button_view_state_chronogram.triggered.connect(self.show_state_chronogram)
        #button_view_state_chronogram.triggered.connect(lambda s: partial(self.toggle_aux_window, window=self.state_window, state=s))
        self.view_actions['state_chrono'] = button_view_state_chronogram
        view_menu.addAction(button_view_state_chronogram)

        
    def on_three_d_set_visible(self, src, what):
        print(what, src.isChecked())
        self.threed_view.set_item_visible(what, src.isChecked())
        
    def show_optimize_dialog(self):
        self.optimize_dialog = OptimizeDialog(self)
        self.optimize_dialog.show()

    def on_optimize(self):
        print(f'soon {self.optimize_dialog.max_vel_edit.text()}')

    def show_csv_export_dialog(self):
        self.export_to_csv_dialog = ExportCsvDialog(self)
        self.export_to_csv_dialog.show()

    def on_export_to_csv(self):
        #print('export', self.export_to_csv_dialog.edit.text())
        self.controller.export_to_csv(self.export_to_csv_dialog.edit.text())
        self.export_to_csv_dialog.hide()
        
    def toggle_aux_window(self, window, state):
        if state:
            window.show()
            window.display_new_trajectory(self.model)
        else: window.hide()
        
    def show_output_chronogram(self, s): self.toggle_aux_window(self.output_chronogram_window, s)
    def show_state_chronogram(self, s): self.toggle_aux_window(self.state_chronogram_window, s)
    def show_geometry(self, s): self.toggle_aux_window(self.geometry_window, s)

    
    def show_si_chronogram(self, s):
        w = self.space_idx_chronogram_window
        if s:
            try: self.model.trajectory.get_dyn_ctl_pts
            except AttributeError:
                print('trajectory is not space indexed')
                return
            w.show()
            w.si_chrono_view._draw(self.model)
            w.si_chrono_view.draw_geometry(self.model)
        else:
            w.hide()
        
    
    def closeEvent(self, event):
        for w in [self.output_chronogram_window, self.space_idx_chronogram_window, self.state_chronogram_window]:
            w.close()
        event.accept()    

    # display a new trajectory (its type might have changed)
    def display_new_trajectory(self, model):
        print('#main_window::display_new_trajectory')
        #traceback.print_stack()
        if self.geometry_window.isVisible():
            self.geometry_window.display_new_trajectory(model)
        self.threed_view.display_new_trajectory(model)

    # update a trajectory (its type has not changed)
    def update_plot(self, model):
        print('#main_window::update plot')
        if self.space_idx_chronogram_window.isVisible():
            self.space_idx_chronogram_window.si_chrono_view._draw(model)
            self.space_idx_chronogram_window.si_chrono_view.draw_geometry(model)
        if self.state_chronogram_window.isVisible():
            self.state_chronogram_window.update_plot(model)
        if self.output_chronogram_window.isVisible():
            self.output_chronogram_window.update_plot(model)
        if self.geometry_window.isVisible():
            self.geometry_window.update_plot(model)

        self.threed_view.update_trajectory(model)
            
    def draw_current_pose(self, elapsed, Y, rmat):
        self.threed_view.set_quad_pose(Y, rmat)
