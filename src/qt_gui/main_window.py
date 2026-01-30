#
# Main window
# (menus, status bar, aux windows handling)
#

from PySide6.QtCore import Qt, QSize #, Signal, Slot
from PySide6.QtGui import QAction, QIcon, QCursor, Qt, QColor, QPalette
from PySide6.QtWidgets import (QMainWindow, QMenu, QVBoxLayout,
                               QWidget, QLabel, QToolBar, QDialog,
                               QLineEdit, QStatusBar, QCheckBox, QToolButton,
                               QPushButton)

import numpy as np
from functools import partial
#from functools import partial # Python is such a sad sad "language". World is such a sad sad world. There's no such thing as God
import traceback
import logging

import view_chronograms as view_chrono, view_geometry as view_geom, view_three_d
import traj_factory

logger = logging.getLogger(__name__)

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

        self.views = {}
        self.threed_view = self.views['3D'] = view_three_d.ThreeDWidget(model)
        self.setCentralWidget(self.threed_view)
        
        self.geometry_window = self.views['geometry'] = view_geom.Window(model, controller)
        self.output_chronogram_window = self.views['output_chrono'] = view_chrono.ChronogramWindow(view_chrono.OutputChronogram, 'Output Chronograms')
        # model and controller not passed to constructor...
        #self.space_indexed_window = view_chrono.ChronogramWindow(view_chrono.SpaceIndexedChronogram, 'Space Indexed Chronograms')
        self.space_idx_chronogram_window = self.views['si_chrono'] = view_chrono.SiChronoWindow(self.model, self.controller)
        self.state_chronogram_window = self.views['state_chrono'] = view_chrono.ChronogramWindow(view_chrono.StateChronogram, 'State Chronograms')
        for k in self.views:
            if k!='3D': self.views[k].closed.connect(partial(self.on_child_closed, which=k))
        
        self.build_window(model, controller)
        self.display_new_trajectory(model)
        
    def on_child_closed(self, which):
        logger.debug('in on_child_closed', which)
        self.show_view_actions[which].setChecked(False)
        
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
        self.anim_label =  QLabel('foo')
        toolbar.addWidget(self.anim_label)
        

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
        def add_3dview_item_visible_action(description, item_key):
            action = QAction(description, self)
            action.setCheckable(True); action.setChecked(self.threed_view.is_item_visible(item_key))
            action.triggered.connect(partial(self.on_three_d_set_item_visible, src=action, what=item_key))
            three_d_submenu.addAction(action)
        for d,i in zip(['Show Grid', 'Show Arena Boundaries', 'Show Frames'], ['grid', 'arena', 'frames']):
            add_3dview_item_visible_action(d, i)
        
        self.show_view_actions = {}
        def add_show_wiew_action(desc, cbk, key):
            action = QAction(desc, self); action.setCheckable(True)
            action.triggered.connect(cbk)
            #action.triggered.connect(lambda s: partial(self.toggle_aux_window, key=key, state=s))
            self.show_view_actions[key] = action
            view_menu.addAction(action)
        for d,c,k in [('View Geometry', self.show_geometry, 'geometry'),
                      ('View Space Indexed Chronogram', self.show_si_chronogram, 'si_chrono'),
                      ('View Output Chronogram', self.show_output_chronogram, 'output_chrono'),
                      ("View State Chronogram", self.show_state_chronogram, 'state_chrono')]:
            add_show_wiew_action(d, c, k)
        
    def on_three_d_set_item_visible(self, src, what):
        self.threed_view.set_item_visible(what, src.isChecked())
        
    def toggle_aux_window(self, key, state):
        aux_window = self.views[key]
        if state:
            aux_window.show()
            aux_window.display_new_trajectory(self.model)
        else: aux_window.hide()
        
    def show_output_chronogram(self, s): self.toggle_aux_window('output_chrono', s)
    def show_si_chronogram(self, s): self.toggle_aux_window('si_chrono', s)
    def show_state_chronogram(self, s): self.toggle_aux_window('state_chrono', s)
    def show_geometry(self, s): self.toggle_aux_window('geometry', s)
    
    def closeEvent(self, event):
        for k in self.views:
            if k != '3D': self.views[k].close()
        event.accept()    

    def show_optimize_dialog(self):
        self.optimize_dialog = OptimizeDialog(self)
        self.optimize_dialog.show()

    def on_optimize(self):
        logger.debug(f'soon {self.optimize_dialog.max_vel_edit.text()}')

    def show_csv_export_dialog(self):
        self.export_to_csv_dialog = ExportCsvDialog(self)
        self.export_to_csv_dialog.show()

    def on_export_to_csv(self):
        self.controller.export_to_csv(self.export_to_csv_dialog.edit.text())
        self.export_to_csv_dialog.hide()


    # display a new trajectory (its structure/type might have changed)
    def display_new_trajectory(self, model):
        logger.debug('in display_new_trajectory')
        _tr = self.model.get_trajectory()

        self.anim_label.setText(f'{_tr.duration:.1f}s')
        
        self.show_view_actions['si_chrono'].setEnabled(_tr.is_space_indexed())
        if not _tr.is_space_indexed():
            self.views['si_chrono'].hide()
            self.show_view_actions['si_chrono'].setChecked(False)
        
        for kv in self.views:
            if kv=='3D' or self.views[kv].isVisible():
                self.views[kv].display_new_trajectory(model)
        #traceback.print_stack()

    # update a trajectory (its structure has not changed)
    def update_plot(self, model):
        logger.debug('in update plot')
        for kv in self.views:
            if kv=='3D' or self.views[kv].isVisible():
                self.views[kv].update_plot(model)
            
    def draw_current_pose(self, elapsed, Y, rmat):
        self.anim_label.setText(f'{elapsed:>4.1f}/{self.model.get_trajectory().duration:.1f}s')
        self.threed_view.set_quad_pose(Y, rmat)
