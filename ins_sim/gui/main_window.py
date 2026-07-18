"""Main application window for the INS simulator GUI.

Left-hand control panel holds the configuration widgets and run
controls; the right-hand frame is an empty placeholder reserved for
future plotting canvases. Simulations run in a background QThread via
SimulationWorker so the UI stays responsive.
"""

import sys

from PySide6.QtCore import QThread, Signal
from PySide6.QtGui import QFontDatabase
from PySide6.QtWidgets import (
    QApplication,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ins_sim.gui.canvas import VisualizationPanel
from ins_sim.gui.widgets import (
    DEFAULT_IMU_SPEC_NAME,
    DEFAULT_TRAJECTORY_NAME,
    DtSpinBox,
    IterationsSpinBox,
    VisualizationOptions,
    YamlFileSelector,
    is_imu_spec_yaml,
    is_trajectory_yaml,
)
from ins_sim.gui.workers import SimulationResult, SimulationWorker

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

        self.last_result: SimulationResult | None = None
        self._thread: QThread | None = None
        self._worker: SimulationWorker | None = None

        # ---- Left control panel -------------------------------------------
        self.file_selector = YamlFileSelector(
            DEFAULT_TRAJECTORY_NAME, "Select trajectory file",
            file_filter=is_trajectory_yaml)
        self.imu_spec_selector = YamlFileSelector(
            DEFAULT_IMU_SPEC_NAME, "Select IMU spec file",
            file_filter=is_imu_spec_yaml)
        self.iterations_spinbox = IterationsSpinBox()
        self.dt_spinbox = DtSpinBox()
        self.visualization_options = VisualizationOptions()

        trajectory_group = QGroupBox("Trajectory")
        trajectory_layout = QVBoxLayout(trajectory_group)
        trajectory_layout.addWidget(self.file_selector)

        simulation_group = QGroupBox("Simulation")
        simulation_layout = QFormLayout(simulation_group)
        simulation_layout.addRow("IMU spec:", self.imu_spec_selector)
        simulation_layout.addRow("Monte Carlo iterations:", self.iterations_spinbox)
        simulation_layout.addRow("Time step (dt):", self.dt_spinbox)

        self.run_button = QPushButton("Run Simulation")
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(True)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumHeight(140)
        self.log_view.setFont(
            QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont))

        control_panel = QWidget()
        control_panel.setFixedWidth(CONTROL_PANEL_WIDTH)
        control_layout = QVBoxLayout(control_panel)
        control_layout.addWidget(trajectory_group)
        control_layout.addWidget(simulation_group)
        control_layout.addWidget(self.visualization_options)
        control_layout.addWidget(self.run_button)
        control_layout.addWidget(self.progress_bar)
        control_layout.addWidget(self.log_view)
        control_layout.addStretch()

        # ---- Right plot area ----------------------------------------------
        self.visualization_panel = VisualizationPanel()

        central = QWidget()
        central_layout = QHBoxLayout(central)
        central_layout.addWidget(control_panel)
        central_layout.addWidget(self.visualization_panel, stretch=1)
        self.setCentralWidget(central)

        self.file_selector.fileSelected.connect(self._on_config_changed)
        self.imu_spec_selector.fileSelected.connect(self._on_config_changed)
        self.iterations_spinbox.valueChanged.connect(self._on_config_changed)
        self.dt_spinbox.valueChanged.connect(self._on_config_changed)
        self.visualization_options.optionsChanged.connect(self._on_config_changed)
        self.run_button.clicked.connect(self._on_run_clicked)

    def current_config(self) -> dict:
        """Returns the full configuration state in one dict.

        Returns:
            dict: ``trajectory_path`` (str | None), ``imu_spec_path``
                (str | None), ``n_iterations`` (int), ``dt_s`` (float),
                and ``visualizations`` ({key: bool}).
        """
        return {
            "trajectory_path": self.file_selector.current_path(),
            "imu_spec_path": self.imu_spec_selector.current_path(),
            "n_iterations": self.iterations_spinbox.value(),
            "dt_s": self.dt_spinbox.value(),
            "visualizations": self.visualization_options.selected_options(),
        }

    # ---- Simulation run lifecycle -----------------------------------------

    def _on_run_clicked(self) -> None:
        self._set_controls_enabled(False)
        self.progress_bar.setValue(0)

        self._thread = QThread(self)
        self._worker = SimulationWorker(self.current_config())
        self._worker.moveToThread(self._thread)

        self._thread.started.connect(self._worker.run)
        self._worker.progress_updated.connect(self.progress_bar.setValue)
        self._worker.log_message.connect(self.log_view.appendPlainText)
        self._worker.simulation_finished.connect(self._on_finished)
        self._worker.error_occurred.connect(self._on_error)

        self._worker.simulation_finished.connect(self._thread.quit)
        self._worker.error_occurred.connect(self._thread.quit)
        self._thread.finished.connect(self._worker.deleteLater)
        self._thread.finished.connect(self._thread.deleteLater)
        self._thread.finished.connect(self._on_thread_finished)

        self._thread.start()

    def _on_finished(self, result: SimulationResult) -> None:
        self.last_result = result
        self.visualization_panel.render(result)

    def _on_error(self, message: str) -> None:
        QMessageBox.critical(self, "Simulation failed", message)

    def _on_thread_finished(self) -> None:
        self._thread = None
        self._worker = None
        self._set_controls_enabled(True)

    def _set_controls_enabled(self, enabled: bool) -> None:
        for widget in (self.file_selector, self.imu_spec_selector,
                       self.iterations_spinbox, self.dt_spinbox,
                       self.visualization_options, self.run_button):
            widget.setEnabled(enabled)

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
