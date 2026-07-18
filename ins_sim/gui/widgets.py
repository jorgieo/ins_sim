"""Custom configuration widgets for the INS simulator GUI.

Each widget owns one piece of simulation configuration and emits a Qt
signal whenever its value changes, so the enclosing window can read the
full config state without polling.
"""

from pathlib import Path

from importlib import resources

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

DEFAULT_TRAJECTORY_NAME = "bqn_departure.yaml"

#: Stable config keys -> human-readable checkbox labels.
VISUALIZATION_CHOICES = {
    "trajectory_3d":    "3D Trajectory",
    "attitude_errors":  "Attitude/Euler Angle Errors",
    "velocity_errors":  "Velocity Errors",
    "position_errors":  "Position Errors",
}


def packaged_trajectory_files() -> list[Path]:
    """Lists the .yaml files bundled in ins_sim/config/, sorted by name.

    Returns:
        list[pathlib.Path]: Absolute paths to every packaged ``*.yaml``
            file.
    """
    config_dir = resources.files("ins_sim.config")
    return sorted(
        (Path(str(entry)) for entry in config_dir.iterdir()
         if entry.name.endswith(".yaml")),
        key=lambda p: p.name,
    )


class TrajectoryFileSelector(QWidget):
    """Combo box of packaged trajectory YAMLs plus a Browse... button.

    The combo auto-populates with every ``.yaml`` in ``ins_sim/config/``;
    the Browse button lets the user add an arbitrary external file,
    which is appended to the combo and selected.

    Signals:
        fileSelected (str): Absolute path of the newly selected file.
    """

    fileSelected = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        self._combo = QComboBox(self)
        for path in packaged_trajectory_files():
            self._combo.addItem(path.name, userData=str(path))
        default_idx = self._combo.findText(DEFAULT_TRAJECTORY_NAME)
        if default_idx >= 0:
            self._combo.setCurrentIndex(default_idx)

        self._browse_btn = QPushButton("Browse...", self)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._combo, stretch=1)
        layout.addWidget(self._browse_btn)

        self._combo.currentIndexChanged.connect(self._on_index_changed)
        self._browse_btn.clicked.connect(self._on_browse)

    def current_path(self) -> str | None:
        """Returns the absolute path of the selected file, or None if empty."""
        return self._combo.currentData()

    def _on_index_changed(self, index: int) -> None:
        path = self._combo.itemData(index)
        if path is not None:
            self.fileSelected.emit(path)

    def _on_browse(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Select trajectory file", str(Path.home()),
            "YAML files (*.yaml *.yml);;All files (*)")
        if not path:
            return
        path = str(Path(path).resolve())
        idx = self._combo.findData(path)
        if idx < 0:
            self._combo.addItem(Path(path).name, userData=path)
            idx = self._combo.count() - 1
        # setCurrentIndex fires currentIndexChanged -> fileSelected, but
        # only if the index actually changes; re-picking the current file
        # is a no-op by design.
        self._combo.setCurrentIndex(idx)


class IterationsSpinBox(QSpinBox):
    """Spin box for the Monte Carlo trial count (1-1000, default 50).

    Emits the built-in ``valueChanged(int)`` signal.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRange(1, 1000)
        self.setValue(50)
        self.setSuffix(" trials")


class VisualizationOptions(QGroupBox):
    """Check boxes selecting which output visualizations to produce.

    Signals:
        optionsChanged (dict): ``{key: bool}`` for every entry in
            VISUALIZATION_CHOICES, emitted on any toggle.
    """

    optionsChanged = Signal(dict)

    def __init__(self, parent=None):
        super().__init__("Output Visualizations", parent)

        self._checkboxes: dict[str, QCheckBox] = {}
        layout = QVBoxLayout(self)
        for key, label in VISUALIZATION_CHOICES.items():
            box = QCheckBox(label, self)
            box.setChecked(True)
            box.toggled.connect(self._on_toggled)
            layout.addWidget(box)
            self._checkboxes[key] = box

    def selected_options(self) -> dict[str, bool]:
        """Returns the checked state of every visualization option."""
        return {key: box.isChecked() for key, box in self._checkboxes.items()}

    def _on_toggled(self, _checked: bool) -> None:
        self.optionsChanged.emit(self.selected_options())
