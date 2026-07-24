#!/usr/bin/env python3

import logging
from custom_scenarios import (load_custom_scenarios, save_custom_scenario,
                              load_recent_names, save_recent_name,
                              CustomScenarioDialog)
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (QDialog, QWidget, QLabel, QPushButton, QSpinBox,
                               QVBoxLayout, QHBoxLayout, QListWidget,
                               QListWidgetItem, QGroupBox, QScrollArea)

logger = logging.getLogger(__name__)

STYLE = """
    QDialog { background-color: #131715; }
    QLabel { color: #E8ECEA; font-size: 13px; }
    QLabel#title { color: #FFFFFF; font-size: 16px; font-weight: 600; }
    QLabel#scenInfo { color: #8B938F; font-size: 12px; }

    QListWidget {
        background-color: #1B201D; border: 1px solid #2A312D;
        border-radius: 6px; color: #E8ECEA; font-size: 12px; padding: 4px;
    }
    QListWidget::item { padding: 7px 8px; border-radius: 4px; color: #B7BEB9; }
    QListWidget::item:hover { background-color: #232925; }
    QListWidget::item:selected {
        background-color: #3A423D; color: #FFFFFF;
        border-left: 3px solid #E8ECEA;
    }

    QGroupBox {
        background-color: #1B201D; border: 1px solid #2A312D;
        border-radius: 6px; margin-top: 10px; padding: 10px;
        font-size: 11px; font-weight: 600; color: #6E7770;
    }
    QGroupBox::title {
        subcontrol-origin: margin; subcontrol-position: top left;
        left: 10px; padding: 2px 6px; color: #6E7770;
    }

    QSpinBox {
        background-color: #232925; color: #E8ECEA;
        border: 1px solid #353D38; border-radius: 5px; padding: 3px 4px;
    }

    QPushButton {
        background-color: #232925; color: #E8ECEA;
        border: 1px solid #353D38; border-radius: 6px;
        padding: 8px 16px; font-size: 13px; font-weight: 600;
    }
    QPushButton:hover { background-color: #2B322D; }
    QPushButton:pressed { background-color: #1C211E; }
    QPushButton#primary {
        background-color: #E8ECEA; border: 1px solid #E8ECEA; color: #131715;
    }
    QPushButton#primary:hover { background-color: #FFFFFF; }
"""


class ScenarioResult:
    """Everything Application needs to start a show, independent of how
    it was put together (a predefined Scenario class or a future
    custom-built one)."""
    def __init__(self, name, desc, ids, trajs, arena=None):
        self.name, self.desc, self.ids, self.trajs = name, desc, list(ids), list(trajs)
        if arena is not None:
            self.arena = arena


class ScenarioPickerDialog(QDialog):
    """Modal dialog shown at startup: pick one of the predefined
    scenarios and optionally remap which physical drone ID plays
    which role in it."""

    def __init__(self, scenarios, preselect=0, parent=None):
        super().__init__(parent)
        predefined = list(scenarios)
        self._id_spins = []

        self._row_scenario = []

        self.setWindowTitle("Click'n Fly - Select scenario")
        self.resize(720, 440)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(14, 14, 14, 14)
        outer.setSpacing(10)

        title = QLabel("SELECT SCENARIO")
        title.setObjectName("title")
        outer.addWidget(title)

        body = QHBoxLayout()
        body.setSpacing(10)

        self.list = QListWidget()
        # grouped for practicability: the last launched shows first (quick
        # re-run), then conflict-free shows, the deconfliction testbeds, and
        # the operator's saved custom ones
        customs = load_custom_scenarios()
        by_name = {c.__name__: c for c in predefined}
        for c in customs:
            by_name.setdefault(c.__name__, c)   # customs don't shadow predefined
        recent = [by_name[n] for n in load_recent_names() if n in by_name]
        no_conflict = [c for c in predefined if not getattr(c, "conflict", False)]
        conflict    = [c for c in predefined if getattr(c, "conflict", False)]
        self._add_group("RECENT", recent)
        self._add_group("NO CONFLICT", no_conflict)
        self._add_group("WITH CONFLICT", conflict)
        self._add_group("CUSTOM", customs)
        self._custom_header_added = bool(customs)
        self.list.setMinimumWidth(300)
        self.list.currentRowChanged.connect(self._on_selection_changed)

        self.list.itemDoubleClicked.connect(self._confirm_selection)
        self.list.itemActivated.connect(self._confirm_selection)
        body.addWidget(self.list, stretch=1)

        self.detail_group = QGroupBox("DRONES")
        self.detail_layout = QVBoxLayout(self.detail_group)
        self.detail_layout.setSpacing(6)
        detail_scroll = QScrollArea()
        detail_scroll.setWidgetResizable(True)
        detail_scroll.setWidget(self.detail_group)
        detail_scroll.setStyleSheet("QScrollArea { border:none; background:transparent; }")
        body.addWidget(detail_scroll, stretch=1)

        outer.addLayout(body, stretch=1)

        buttons = QHBoxLayout()
        self.button_custom = QPushButton("New custom scenario...")
        self.button_custom.clicked.connect(self._on_new_custom)
        buttons.addWidget(self.button_custom)
        buttons.addStretch(1)
        self.button_quit = QPushButton("Quit")
        self.button_quit.clicked.connect(self.reject)
        self.button_quit.setAutoDefault(False)
        self.button_custom.setAutoDefault(False)
        self.button_start = QPushButton("Start")
        self.button_start.setObjectName("primary")
        self.button_start.clicked.connect(self.accept)
        self.button_start.setDefault(True)   
        buttons.addWidget(self.button_quit)
        buttons.addWidget(self.button_start)
        outer.addLayout(buttons)

        self.setStyleSheet(STYLE)

        # preselect is an index into the scenarios passed in; map it to the
        # grouped list and fall back to the first selectable row
        target = predefined[preselect] if 0 <= preselect < len(predefined) else None
        self._select_scenario(target)

    def _confirm_selection(self, item):
        """Accept the dialog if the activated row is a real scenario (not a
        group header). Wired to double-click and Enter."""
        row = self.list.row(item)
        if 0 <= row < len(self._row_scenario) and self._row_scenario[row] is not None:
            self.accept()

    def _add_header(self, text):
        item = QListWidgetItem(text)
        item.setFlags(Qt.ItemFlag.NoItemFlags)      # a divider, not selectable
        f = item.font(); f.setBold(True); item.setFont(f)
        item.setForeground(QColor("#6E7770"))
        self.list.addItem(item)
        self._row_scenario.append(None)

    def _add_group(self, title, items):
        if not items:
            return
        self._add_header(title)
        for cls in items:
            self.list.addItem(self._label(cls))
            self._row_scenario.append(cls)

    def _select_scenario(self, cls):
        """Select the row for a scenario class, else the first selectable row."""
        row = self._row_scenario.index(cls) if cls in self._row_scenario else -1
        if row < 0:
            row = next((i for i, c in enumerate(self._row_scenario) if c is not None), -1)
        if row >= 0:
            self.list.setCurrentRow(row)

    @staticmethod
    def _label(cls):
        # the descriptive name is enough; drop the "ScenarioN" prefix
        name = getattr(cls, "desc", None) or cls.__name__
        name = name[:1].upper() + name[1:]
        return f"{name}   ({len(cls.ids)} drone(s))"

    def _on_new_custom(self):
        dlg = CustomScenarioDialog(self)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return
        name, desc, ids, trajs = dlg.result_scenario
        save_custom_scenario(name, desc, ids, trajs)
        cls = type(str(name), (), {'desc': desc, 'ids': ids, 'trajs': trajs})
        if not self._custom_header_added:
            self._add_header("CUSTOM")
            self._custom_header_added = True
        self.list.addItem(self._label(cls))
        self._row_scenario.append(cls)
        self.list.setCurrentRow(self.list.count() - 1)

    def _on_selection_changed(self, row):
        while self.detail_layout.count():
            item = self.detail_layout.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()
        self._id_spins = []

        if row < 0:
            return
        cls = self._row_scenario[row]
        if cls is None:                 # a group header, nothing to show
            return
        for _id, traj in zip(cls.ids, cls.trajs):
            row_box = QHBoxLayout()
            spin = QSpinBox()
            spin.setRange(0, 999)
            spin.setValue(_id)
            spin.setFixedWidth(70)
            traj_lbl = QLabel(traj)
            traj_lbl.setObjectName("scenInfo")
            traj_lbl.setWordWrap(True)
            row_box.addWidget(spin)
            row_box.addWidget(traj_lbl, stretch=1)
            self._id_spins.append(spin)
            container = QWidget()
            container.setLayout(row_box)
            self.detail_layout.addWidget(container)
        self.detail_layout.addStretch(1)

    def get_scenario(self):
        cls = self._row_scenario[self.list.currentRow()]
        save_recent_name(cls.__name__)   # remember it for the RECENT group
        ids = [spin.value() for spin in self._id_spins]
        return ScenarioResult(
            name=cls.__name__,
            desc=getattr(cls, "desc", None),
            ids=ids,
            trajs=cls.trajs,
            arena=getattr(cls, "arena", None),
        )
