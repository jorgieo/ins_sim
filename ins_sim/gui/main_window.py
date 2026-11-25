"""Main application window for the INS simulator GUI.

Left-hand control panel holds the configuration widgets and run
controls; the right-hand VisualizationPanel shows one interactive plotly
tab per selected visualization after a run. Simulations run in a
background QThread via SimulationWorker so the UI stays responsive.
"""

import sys

from PySide6.QtCore import QThread, Qt, Signal
from PySide6.QtGui import QFontDatabase, QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
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
from ins_sim.gui.paths import imu_specs_dir, trajectories_dir
from ins_sim.gui.widgets import (
    AdvancedOptions,
    DEFAULT_IMU_SPEC_NAME,
    DEFAULT_TRAJECTORY_NAME,
    DtSpinBox,
    IterationsSpinBox,
    VisualizationOptions,
    YamlFileSelector,
    is_imu_spec_yaml,
    is_trajectory_yaml,
)
from ins_sim.gui.workers import PartialRun, SimulationResult, SimulationWorker

CONTROL_PANEL_WIDTH = 320
RUN_TEXT = "Run Simulation"
CANCEL_TEXT = "Cancel"
CANCELLING_TEXT = "Cancelling..."
#: Fewest completed trials worth plotting: the 3-sigma bands need 2.
MIN_TRIALS_TO_SHOW = 2


class MainWindow(QMainWindow):
    """Top-level window: control panel on the left, visualization tabs on the right.

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
        self._pending_partial: PartialRun | None = None
        self._closing = False   # the user chose to quit; close once the run stops

        # ---- Left control panel -------------------------------------------
        self.file_selector = YamlFileSelector(
            DEFAULT_TRAJECTORY_NAME, "Select trajectory file",
            file_filter=is_trajectory_yaml, user_dir=trajectories_dir())
        self.imu_spec_selector = YamlFileSelector(
            DEFAULT_IMU_SPEC_NAME, "Select IMU spec file",
            file_filter=is_imu_spec_yaml, user_dir=imu_specs_dir())
        self.iterations_spinbox = IterationsSpinBox()
        self.dt_spinbox = DtSpinBox()
        self.baro_aiding_checkbox = QCheckBox("Baro altitude aiding")
        self.baro_aiding_checkbox.setChecked(True)
        self.baro_aiding_checkbox.setToolTip(
            "Checked: baro-damped vertical channel (third-order loop on the "
            "barometric altitude,\nwith the errors in the IMU spec's baro "
            "block; ideal without one).\nUnchecked: free-inertial vertical "
            "channel — inherently unstable, altitude error diverges.")
        self.advanced_options = AdvancedOptions()
        self.visualization_options = VisualizationOptions()

        trajectory_group = QGroupBox("Trajectory")
        trajectory_layout = QVBoxLayout(trajectory_group)
        trajectory_layout.addWidget(self.file_selector)

        simulation_group = QGroupBox("Simulation")
        simulation_layout = QFormLayout(simulation_group)
        simulation_layout.addRow("IMU spec:", self.imu_spec_selector)
        simulation_layout.addRow("Monte Carlo iterations:", self.iterations_spinbox)
        simulation_layout.addRow("Time step (dt):", self.dt_spinbox)
        simulation_layout.addRow("", self.baro_aiding_checkbox)
        simulation_layout.addRow(self.advanced_options)

        self.run_button = QPushButton(RUN_TEXT)
        self.clear_button = QPushButton("Clear results")
        self.clear_button.setToolTip(
            "Remove the plots, clear the log and reset the progress bar")
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
        run_row = QHBoxLayout()
        run_row.addWidget(self.run_button, stretch=2)
        run_row.addWidget(self.clear_button, stretch=1)
        control_layout.addLayout(run_row)
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
        self.baro_aiding_checkbox.toggled.connect(self._on_config_changed)
        self.advanced_options.optionsChanged.connect(self._on_config_changed)
        self.visualization_options.optionsChanged.connect(self._on_config_changed)
        self.run_button.clicked.connect(self._on_run_button_clicked)
        self.clear_button.clicked.connect(self.clear_results)
        QShortcut(QKeySequence(Qt.Key.Key_Escape), self,
                  activated=self.cancel_run)

    def current_config(self) -> dict:
        """Returns the full configuration state in one dict.

        Returns:
            dict: ``trajectory_path`` (str | None), ``imu_spec_path``
                (str | None), ``n_iterations`` (int), ``dt_s`` (float),
                ``baro_aiding`` (bool), ``mechanization`` ("basic" or
                "precision"), ``oversample`` (int | None), and
                ``visualizations`` ({key: bool}).
        """
        return {
            "trajectory_path": self.file_selector.current_path(),
            "imu_spec_path": self.imu_spec_selector.current_path(),
            "n_iterations": self.iterations_spinbox.value(),
            "dt_s": self.dt_spinbox.value(),
            "baro_aiding": self.baro_aiding_checkbox.isChecked(),
            "mechanization": self.advanced_options.mechanization(),
            "oversample": self.advanced_options.oversample(),
            "visualizations": self.visualization_options.selected_options(),
        }

    # ---- Simulation run lifecycle -----------------------------------------

    def is_running(self) -> bool:
        """True while a simulation runs, including while it is cancelling."""
        return self._thread is not None

    def cancel_run(self) -> None:
        """Asks the running simulation to stop after the current trial.

        The previous results stay on screen. Does nothing when no run is
        active or a cancellation is already pending.
        """
        if self._worker is None or self._worker.cancel_requested:
            return
        self._worker.request_cancel()
        self.run_button.setEnabled(False)
        self.run_button.setText(CANCELLING_TEXT)
        self.log_view.appendPlainText("Cancelling after the current trial...")

    def _on_run_button_clicked(self) -> None:
        if self.is_running():
            self.cancel_run()
        else:
            self._on_run_clicked()

    def _on_run_clicked(self) -> None:
        self._set_progress(0)
        self._start_worker(SimulationWorker(self.current_config()))

    def _start_worker(self, worker: SimulationWorker) -> None:
        self._set_controls_enabled(False)
        self.run_button.setText(CANCEL_TEXT)
        self.run_button.setToolTip("Stop after the current trial (Esc)")
        self.run_button.setEnabled(True)

        self._thread = QThread(self)
        self._worker = worker
        self._worker.moveToThread(self._thread)

        self._thread.started.connect(self._worker.run)
        self._worker.progress_updated.connect(self.progress_bar.setValue)
        self._worker.log_message.connect(self.log_view.appendPlainText)
        self._worker.simulation_finished.connect(self._on_finished)
        self._worker.error_occurred.connect(self._on_error)
        self._worker.simulation_cancelled.connect(self._on_cancelled)

        self._worker.simulation_finished.connect(self._thread.quit)
        self._worker.error_occurred.connect(self._thread.quit)
        self._worker.simulation_cancelled.connect(self._thread.quit)
        self._thread.finished.connect(self._worker.deleteLater)
        self._thread.finished.connect(self._thread.deleteLater)
        self._thread.finished.connect(self._on_thread_finished)

        self._thread.start()

    def _on_finished(self, result: SimulationResult) -> None:
        if self._closing:
            return
        self.last_result = result
        self.visualization_panel.render(result)

    def _on_error(self, message: str) -> None:
        if self._closing:
            return
        QMessageBox.critical(self, "Simulation failed", message)

    def _on_cancelled(self, partial: PartialRun | None) -> None:
        self._set_progress(0, "Cancelled")
        if not self._closing:
            self._pending_partial = partial     # offered once the thread ends

    def _on_thread_finished(self) -> None:
        self._thread = None
        self._worker = None
        if self._closing:
            self.close()
            return
        partial, self._pending_partial = self._pending_partial, None
        if partial is not None:
            if partial.n_done < MIN_TRIALS_TO_SHOW:
                self.log_view.appendPlainText(
                    f"Only {partial.n_done} trial completed; at least "
                    f"{MIN_TRIALS_TO_SHOW} are needed to show results.")
            elif self._confirm_partial(partial.n_done, partial.n_requested):
                self._set_progress(100, f"{partial.n_done}/"
                                        f"{partial.n_requested} trials")
                self._start_worker(SimulationWorker(partial.config,
                                                    partial=partial))
                return
        self._set_controls_enabled(True)
        self.run_button.setText(RUN_TEXT)
        self.run_button.setToolTip("")
        self.run_button.setEnabled(True)

    # ---- Closing ------------------------------------------------------------

    def closeEvent(self, event) -> None:
        """Stops a running simulation before the window closes.

        Closing while the worker thread runs would destroy the running
        QThread and end the process abnormally. Instead, after the user
        confirms, the run is cancelled and the window stays open until the
        thread finishes (at most one trial), then closes itself.
        """
        if self._thread is None:
            event.accept()
            return
        event.ignore()
        if self._closing:
            return                                   # already stopping
        if not self._worker.cancel_requested and not self._confirm_stop():
            return
        self._closing = True
        self.log_view.appendPlainText(
            "Stopping the simulation; the window closes when the current "
            "trial ends...")
        self.cancel_run()

    def _confirm_stop(self) -> bool:
        answer = QMessageBox.question(
            self, "Simulation running",
            "A simulation is running. Stop it and quit?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        return answer == QMessageBox.Yes

    def _confirm_partial(self, n_done: int, n_requested: int) -> bool:
        answer = QMessageBox.question(
            self, "Simulation cancelled",
            f"Show results from the {n_done} completed trials "
            f"(of {n_requested})?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.Yes)
        return answer == QMessageBox.Yes

    def clear_results(self) -> None:
        """Removes the results, clears the log and resets the progress bar.

        The window then looks as it does on start, apart from the settings.
        Does nothing while a run is active.
        """
        if self.is_running():
            return
        self.last_result = None
        self.visualization_panel.show_placeholder()
        self.log_view.clear()
        self._set_progress(0)

    def _set_progress(self, value: int, text: str = "%p%") -> None:
        self.progress_bar.setValue(value)
        self.progress_bar.setFormat(text)

    def _set_controls_enabled(self, enabled: bool) -> None:
        for widget in (self.file_selector, self.imu_spec_selector,
                       self.iterations_spinbox, self.dt_spinbox,
                       self.baro_aiding_checkbox, self.advanced_options,
                       self.visualization_options, self.clear_button):
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
