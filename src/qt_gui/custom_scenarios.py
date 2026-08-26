#!/usr/bin/env python3
#
# Operator-composed scenarios: pick trajectories from the TrajFactory
# library, assign a drone id to each, name the result. Saved to a YAML
# file so they reappear in the scenario picker on the next launch,
# side by side with the predefined scenarios.
#
import os, logging, yaml

from PySide6.QtWidgets import (QDialog, QWidget, QLabel, QPushButton, QSpinBox,
                               QVBoxLayout, QHBoxLayout, QListWidget, QGroupBox,
                               QLineEdit, QScrollArea, QMessageBox)

import traj_factory

logger = logging.getLogger(__name__)

DEFAULT_PATH = os.path.join(os.path.dirname(__file__), 'data', 'custom_scenarios.yaml')
RECENT_PATH = os.path.join(os.path.dirname(__file__), 'data', 'recent_scenarios.yaml')
RECENT_MAX = 3            # how many recently-launched scenarios to keep

_DEFAULT_FIRST_ID = 112   # the lab fleet numbering starts here


def load_recent_names(path=RECENT_PATH):
    """The names of the last few launched scenarios, most recent first."""
    try:
        with open(path) as f:
            names = yaml.safe_load(f) or []
    except FileNotFoundError:
        return []
    except Exception as e:
        logger.warning(f'recent scenarios file unreadable, ignoring it: {e}')
        return []
    return [str(n) for n in names][:RECENT_MAX]

FLEET_PATH = os.path.join(os.path.dirname(__file__), 'data', 'fleet_ids.yaml')
 
 
def load_fleet_ids(path=FLEET_PATH):
    """The operator's last per-slot drone ids, so changing scenario keeps the
    same mapping instead of resetting to the defaults."""
    try:
        with open(path) as f:
            ids = yaml.safe_load(f) or []
    except FileNotFoundError:
        return []
    except Exception as e:
        logger.warning(f'fleet ids file unreadable, ignoring it: {e}')
        return []
    return [int(i) for i in ids]
 
 
def save_fleet_ids(ids, path=FLEET_PATH):
    """Remember the per-slot ids; merge with what's stored so editing a
    2-drone scenario keeps the 3rd slot set from a previous 3-drone one."""
    merged = list(ids) + load_fleet_ids(path)[len(ids):]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        yaml.safe_dump([int(i) for i in merged], f, sort_keys=False)

def save_recent_name(name, path=RECENT_PATH, max_n=RECENT_MAX):
    """Record a scenario as just launched (dedup, most recent first)."""
    names = [n for n in load_recent_names(path) if n != name]
    names.insert(0, str(name))
    names = names[:max_n]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        yaml.safe_dump(names, f, allow_unicode=True, sort_keys=False)


def load_custom_scenarios(path=DEFAULT_PATH):
    """Return the saved custom scenarios as picker-compatible classes
    (same duck type as the classes in scenarios.py: __name__, desc,
    ids, trajs)."""
    try:
        with open(path) as f:
            entries = yaml.safe_load(f) or []
    except FileNotFoundError:
        return []
    except Exception as e:
        logger.warning(f'custom scenarios file unreadable, ignoring it: {e}')
        return []
    out = []
    known = traj_factory.TrajFactory._trajectories
    for e in entries:
        trajs = list(e.get('trajs', []))
        unknown = [t for t in trajs if t not in known]
        if unknown:
            logger.warning(f"custom scenario '{e.get('name')}' skipped: "
                           f"unknown trajectories {unknown}")
            continue
        out.append(type(str(e.get('name', 'custom')), (), {
            'desc': e.get('desc', 'custom'),
            'ids': list(e.get('ids', [])),
            'trajs': trajs,
        }))
    return out


def save_custom_scenario(name, desc, ids, trajs, path=DEFAULT_PATH):
    """Append (or replace, matching by name) a custom scenario."""
    try:
        with open(path) as f:
            entries = yaml.safe_load(f) or []
    except FileNotFoundError:
        entries = []
    entries = [e for e in entries if e.get('name') != name]
    entries.append({'name': name, 'desc': desc,
                    'ids': list(ids), 'trajs': list(trajs)})
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        yaml.safe_dump(entries, f, allow_unicode=True, sort_keys=False)
    logger.info(f"custom scenario '{name}' saved to {path}")

def delete_custom_scenario(name, path=DEFAULT_PATH):
    """Remove the custom scenario named `name` (no-op if absent)."""
    try:
        with open(path) as f:
            entries = yaml.safe_load(f) or []
    except FileNotFoundError:
        return
    kept = [e for e in entries if e.get('name') != name]
    if len(kept) == len(entries):
        return
    with open(path, 'w') as f:
        yaml.safe_dump(kept, f, allow_unicode=True, sort_keys=False)
    logger.info(f"custom scenario '{name}' deleted from {path}")

# same monochrome look as the scenario picker (kept local to avoid a
# circular import: scenario_picker imports this module)
STYLE = """
    QDialog { background-color: #131715; }
    QLabel { color: #E8ECEA; font-size: 13px; }
    QLabel#title { color: #FFFFFF; font-size: 16px; font-weight: 600; }
    QLabel#muted { color: #8B938F; font-size: 11px; }

    QListWidget {
        background-color: #1B201D; border: 1px solid #2A312D;
        border-radius: 6px; color: #E8ECEA; font-size: 12px; padding: 4px;
    }
    QListWidget::item { padding: 6px 8px; border-radius: 4px; color: #B7BEB9; }
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

    QLineEdit, QSpinBox {
        background-color: #232925; color: #E8ECEA;
        border: 1px solid #353D38; border-radius: 5px; padding: 4px 6px;
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
    QPushButton#small { padding: 2px 8px; font-size: 12px; }
"""


class CustomScenarioDialog(QDialog):
    """Compose a scenario from the trajectory library.

    On accept, self.result_scenario holds (name, desc, ids, trajs)."""

    def __init__(self, parent=None, initial=None):
        """`initial`, if given, is (name, ids, trajs) to pre-fill the form
        (used to Modify or Duplicate an existing scenario)."""
        super().__init__(parent)
        self.setWindowTitle("Click'n Fly - Compose custom scenario")
        self.resize(820, 520)
        self.result_scenario = None
        self._rows = []   # list of dicts: {widget, spin, traj}
        self._initials = initial
      
        outer = QVBoxLayout(self)
        outer.setContentsMargins(14, 14, 14, 14)
        outer.setSpacing(10)
        title = QLabel("COMPOSE CUSTOM SCENARIO")
        title.setObjectName("title")
        outer.addWidget(title)

        body = QHBoxLayout()
        body.setSpacing(10)

        # --- left: the trajectory library, filterable
        lib_box = QGroupBox("TRAJECTORY LIBRARY")
        lib_lay = QVBoxLayout(lib_box)
        lib_lay.setSpacing(6)
        self.filter_edit = QLineEdit()
        self.filter_edit.setPlaceholderText("filter...")
        self.filter_edit.textChanged.connect(self._apply_filter)
        lib_lay.addWidget(self.filter_edit)
        self.lib_list = QListWidget()
        self._traj_names = sorted(traj_factory.TrajFactory._trajectories.keys())
        for name in self._traj_names:
            cls = traj_factory.TrajFactory._trajectories[name]
            desc = getattr(cls, 'desc', '')
            self.lib_list.addItem(f"{name} — {desc}" if desc else name)
        self.lib_list.itemDoubleClicked.connect(lambda _i: self._add_selected())
        lib_lay.addWidget(self.lib_list, stretch=1)
        btn_add = QPushButton("Add to scenario →")
        btn_add.clicked.connect(self._add_selected)
        lib_lay.addWidget(btn_add)
        body.addWidget(lib_box, stretch=1)

        # --- right: the composition
        comp_box = QGroupBox("YOUR SCENARIO")
        comp_lay = QVBoxLayout(comp_box)
        comp_lay.setSpacing(6)
        name_row = QHBoxLayout()
        name_row.addWidget(QLabel("Name:"))
        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("my show")
        name_row.addWidget(self.name_edit, stretch=1)
        comp_lay.addLayout(name_row)

        rows_host = QWidget()
        self.rows_lay = QVBoxLayout(rows_host)
        self.rows_lay.setContentsMargins(0, 0, 0, 0)
        self.rows_lay.setSpacing(4)
        self.rows_lay.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(rows_host)
        scroll.setStyleSheet("QScrollArea { border:none; background:transparent; }")
        comp_lay.addWidget(scroll, stretch=1)

        hint = QLabel("double-click a trajectory to add it; one row = one drone")
        hint.setObjectName("muted")
        comp_lay.addWidget(hint)
        body.addWidget(comp_box, stretch=1)

        outer.addLayout(body, stretch=1)

        buttons = QHBoxLayout()
        buttons.addStretch(1)
        btn_cancel = QPushButton("Cancel")
        btn_cancel.clicked.connect(self.reject)
        btn_save = QPushButton("Save scenario")
        btn_save.setObjectName("primary")
        btn_save.clicked.connect(self._on_save)
        buttons.addWidget(btn_cancel)
        buttons.addWidget(btn_save)
        outer.addLayout(buttons)

        self.setStyleSheet(STYLE)

        # pre-fill from an existing scenario (Modify / Duplicate)
        if initial is not None:
            init_name, init_ids, init_trajs = initial
            self.name_edit.setText(str(init_name))
            for _id, tname in zip(init_ids, init_trajs):
                self._add_row(tname)
                self._rows[-1]['spin'].setValue(int(_id))

    def _apply_filter(self, text):
        text = text.lower()
        for i in range(self.lib_list.count()):
            item = self.lib_list.item(i)
            item.setHidden(text not in item.text().lower())

    def _add_selected(self):
        # currentRow indexes the full model: hidden (filtered-out) items
        # keep their row, so it maps directly onto _traj_names
        row = self.lib_list.currentRow()
        if row < 0:
            return
        self._add_row(self._traj_names[row])

    def _add_row(self, traj_name):
        spin = QSpinBox()
        spin.setRange(0, 999)
        spin.setValue(_DEFAULT_FIRST_ID + len(self._rows))
        spin.setFixedWidth(70)
        # show the loop period: each trajectory loops on its own duration,
        # so mixing different periods is fine, but it's good to see them
        try:
            dur = traj_factory.TrajFactory.get(traj_name)().duration
            lbl = QLabel(f"{traj_name}   ({dur:.1f}s)")
        except Exception as e:
            logger.warning(f"could not instantiate '{traj_name}' for duration: {e}")
            lbl = QLabel(traj_name)
        lbl.setWordWrap(True)
        btn = QPushButton("✕")
        btn.setObjectName("small")
        btn.setFixedWidth(28)

        w = QWidget()
        lay = QHBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        lay.addWidget(spin)
        lay.addWidget(lbl, stretch=1)
        lay.addWidget(btn)

        entry = {'widget': w, 'spin': spin, 'traj': traj_name}
        btn.clicked.connect(lambda: self._remove_row(entry))
        self._rows.append(entry)
        self.rows_lay.insertWidget(self.rows_lay.count() - 1, w)  # before stretch

    def _remove_row(self, entry):
        self._rows.remove(entry)
        self.rows_lay.removeWidget(entry['widget'])
        entry['widget'].deleteLater()

    def _on_save(self):
        if not self._rows:
            QMessageBox.warning(self, "Empty scenario",
                                "Add at least one trajectory.")
            return
        ids = [e['spin'].value() for e in self._rows]
        if len(set(ids)) != len(ids):
            QMessageBox.warning(self, "Duplicate drone ids",
                                "Each drone id must be unique.")
            return
        trajs = [e['traj'] for e in self._rows]
        name = self.name_edit.text().strip() or "custom"
        desc = f"custom, {len(ids)} drone(s)"
        self.result_scenario = (name, desc, ids, trajs)
        self.accept()
