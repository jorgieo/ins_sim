"""Main application window for the INS simulator GUI.

Left-hand control panel holds the configuration widgets; the right-hand
frame is an empty placeholder reserved for future plotting canvases.
"""

import sys

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QApplication,
    QFormLayout,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QMainWindow,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from ins_sim.gui.widgets import (
    IterationsSpinBox,
    TrajectoryFileSelector,
    VisualizationOptions,
)

CONTROL_PANEL_WIDTH = 320


class MainWindow(QMainWindow):
    """Top-level window: control panel on the left, plot area placeholder on the right.

    Signals:
        configChanged: Emitted whenever any configuration widget changes,
            so listeners can re-read current_config().
    """

    configChanged = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("INS Monte Carlo Simulator")
        self.resize(1200, 700)

        # ---- Left control panel -------------------------------------------
        self.file_selector = TrajectoryFileSelector()
        self.iterations_spinbox = IterationsSpinBox()
        self.visualization_options = VisualizationOptions()

        trajectory_group = QGroupBox("Trajectory")
        trajectory_layout = QVBoxLayout(trajectory_group)
        trajectory_layout.addWidget(self.file_selector)

        simulation_group = QGroupBox("Simulation")
        simulation_layout = QFormLayout(simulation_group)
        simulation_layout.addRow("Monte Carlo iterations:", self.iterations_spinbox)

        control_panel = QWidget()
        control_panel.setFixedWidth(CONTROL_PANEL_WIDTH)
        control_layout = QVBoxLayout(control_panel)
        control_layout.addWidget(trajectory_group)
        control_layout.addWidget(simulation_group)
        control_layout.addWidget(self.visualization_options)
        control_layout.addStretch()

        # ---- Right placeholder for future plot canvases -------------------
        self.plot_placeholder = QFrame()
        self.plot_placeholder.setObjectName("plotPlaceholder")
        self.plot_placeholder.setFrameShape(QFrame.Shape.StyledPanel)
        self.plot_placeholder.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        central = QWidget()
        central_layout = QHBoxLayout(central)
        central_layout.addWidget(control_panel)
        central_layout.addWidget(self.plot_placeholder, stretch=1)
        self.setCentralWidget(central)

        self.file_selector.fileSelected.connect(self._on_config_changed)
        self.iterations_spinbox.valueChanged.connect(self._on_config_changed)
        self.visualization_options.optionsChanged.connect(self._on_config_changed)

    def current_config(self) -> dict:
        """Returns the full configuration state in one dict.

        Returns:
            dict: ``trajectory_path`` (str | None), ``n_iterations``
                (int), and ``visualizations`` ({key: bool}).
        """
        return {
            "trajectory_path": self.file_selector.current_path(),
            "n_iterations": self.iterations_spinbox.value(),
            "visualizations": self.visualization_options.selected_options(),
        }

    def _on_config_changed(self, *_args) -> None:
        self.configChanged.emit()


def main() -> None:
    """Launches the GUI event loop (blocks until the window closes)."""
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
