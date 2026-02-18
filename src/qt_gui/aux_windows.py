from PySide6.QtWidgets import (QVBoxLayout,
                               QWidget, QLabel, QToolBar, QDialog,
                               QLineEdit, QStatusBar, QCheckBox, QToolButton,
                               QPushButton)


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
