"""Tabbed matplotlib visualization panel for simulation results.

VisualizationPanel is a QTabWidget that renders one interactive canvas
(with pan/zoom/save toolbar) per selected visualization, from the
SimulationResult a SimulationWorker emits. Figures are built with
matplotlib.figure.Figure directly (not pyplot) so no global figure
manager state leaks into the Qt embedding.

Chart language matches evaluation/visualization.py: navy = truth/mean,
steelblue = individual trials, crimson = uncertainty bounds.
"""

import numpy as np
from matplotlib.backends.backend_qt import NavigationToolbar2QT
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QAbstractItemView,
    QHeaderView,
    QLabel,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

GRID_KW = dict(color="#dddddd", linewidth=0.8)
MAX_PLOT_POINTS = 4000
NM = 1852.0


def _step(n: int) -> int:
    return max(1, n // MAX_PLOT_POINTS)


def _style_axes(ax):
    ax.grid(True, **GRID_KW)
    ax.set_axisbelow(True)


def _horiz_err_nm(result):
    """Per-run horizontal position error in NM, shape (n_trials, M)."""
    return np.linalg.norm(
        result.pos_runs[:, :, :2] - result.truth.pos_n[None, :, :2],
        axis=2) / NM


class VisualizationPanel(QTabWidget):
    """Holds one plotting canvas tab per selected visualization.

    Call render(result) with a SimulationResult to (re)build the tabs;
    only visualizations checked in result.config["visualizations"] are
    generated. Shows a placeholder tab until the first render.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.show_placeholder()

    def show_placeholder(self) -> None:
        """Clears all tabs and shows the pre-run placeholder."""
        self.clear()
        label = QLabel("Run a simulation to see results")
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        label.setStyleSheet("color: gray; font-size: 14px;")
        self.addTab(label, "Results")

    def render(self, result) -> None:
        """Rebuilds the tabs from a SimulationResult, honoring the checkboxes.

        Args:
            result: SimulationResult from SimulationWorker; the
                visualization selection rides in
                result.config["visualizations"].
        """
        self.clear()
        enabled = result.config.get("visualizations", {})
        builders = [
            ("trajectory_3d",   "3D Trajectory",   self._figure_trajectory_3d),
            ("attitude_errors", "Attitude Errors", self._figure_attitude_errors),
            ("velocity_errors", "Velocity Errors", self._figure_velocity_errors),
            ("position_errors", "Position Errors", self._figure_position_errors),
            ("cep",             "CEP",             self._figure_cep),
        ]
        for slug, title, builder in builders:
            if enabled.get(slug, False):
                extra = self._cep_table(result) if slug == "cep" else None
                self._add_canvas_tab(builder(result), title, extra=extra)
        if self.count() == 0:
            self.show_placeholder()

    # ---- Tab plumbing ------------------------------------------------------

    def _add_canvas_tab(self, fig: Figure, title: str,
                        extra: QWidget | None = None) -> None:
        """Wraps a figure in a canvas + navigation toolbar and adds a tab.

        Args:
            fig: Figure to embed.
            title: Tab label.
            extra: Optional widget placed below the canvas (e.g. the CEP
                percentile table).
        """
        canvas = FigureCanvasQTAgg(fig)
        toolbar = NavigationToolbar2QT(canvas, self)
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(toolbar)
        layout.addWidget(canvas)
        if extra is not None:
            layout.addWidget(extra)
        self.addTab(page, title)

    # ---- Figure builders ---------------------------------------------------

    def _figure_trajectory_3d(self, result) -> Figure:
        """3D track: truth vs INS-estimated trials, with 95th-pct error tube."""
        from ins_sim.evaluation.visualization import plot_error_tube

        truth = result.truth
        P = truth.pos_n
        s = _step(len(P))

        fig = Figure(layout="constrained")
        ax = fig.add_subplot(projection="3d")

        # Isotropic km units so the tube cross-section stays geometrically
        # correct (see build_summary_figure); downsampled for canvas speed.
        P_km = P[::s] / 1000.0
        r95_km = result.r95[::s] / 1000.0
        ax.plot(P_km[:, 1], P_km[:, 0], -P_km[:, 2], "k-", lw=1.5,
                label="Truth")
        for i in range(min(6, result.n_trials)):
            Q_km = result.pos_runs[i, ::s] / 1000.0
            ax.plot(Q_km[:, 1], Q_km[:, 0], -Q_km[:, 2], "-",
                    color="steelblue", alpha=0.35, lw=0.5,
                    label="INS estimate" if i == 0 else None)
        plot_error_tube(ax, P_km, r95_km)
        ax.set_xlabel("East (km)")
        ax.set_ylabel("North (km)")
        ax.set_zlabel("Up (km)")
        ax.set_title(f"{result.n_trials}-trial Monte Carlo")
        ax.legend(loc="upper left")
        return fig

    def _plot_3sigma_bands(self, fig: Figure, t_m, err_runs, panels, unit: str):
        """Draws per-axis mean-error curves with +/-3-sigma bands.

        Args:
            fig: Figure to populate.
            t_m: Time axis [min], shape (M,).
            err_runs: Per-trial error, shape (n_trials, M, 3).
            panels: list of (column index, panel title) per subplot.
            unit: Y-axis unit label.
        """
        mean = err_runs.mean(axis=0)
        sigma = err_runs.std(axis=0)
        axes = fig.subplots(len(panels), 1, sharex=True)
        for ax, (col, title) in zip(np.atleast_1d(axes), panels):
            ax.fill_between(t_m, mean[:, col] - 3 * sigma[:, col],
                            mean[:, col] + 3 * sigma[:, col],
                            color="crimson", alpha=0.25, label="±3σ")
            ax.plot(t_m, mean[:, col], color="navy", lw=1.2, label="Mean error")
            ax.axhline(0.0, color="gray", lw=0.8, linestyle=":")
            ax.set_ylabel(f"{title} ({unit})")
            ax.set_title(title)
            ax.legend(fontsize=8, loc="upper left")
            _style_axes(ax)
        np.atleast_1d(axes)[-1].set_xlabel("Time (min)")

    def _figure_attitude_errors(self, result) -> Figure:
        """Pitch/roll/heading Euler-angle errors with ±3σ bounds [deg]."""
        truth = result.truth
        euler_err = result.euler_runs - truth.euler[None, :, :]
        euler_err[:, :, 2] = (euler_err[:, :, 2] + np.pi) % (2 * np.pi) - np.pi
        euler_err_deg = np.rad2deg(euler_err)

        fig = Figure(layout="constrained")
        self._plot_3sigma_bands(
            fig, truth.t / 60.0, euler_err_deg,
            [(1, "Pitch error"), (0, "Roll error"), (2, "Heading error")],
            "deg")
        return fig

    def _figure_velocity_errors(self, result) -> Figure:
        """NED velocity errors with ±3σ bounds [m/s]."""
        truth = result.truth
        vel_err = result.vel_runs - truth.vel_n[None, :, :]

        fig = Figure(layout="constrained")
        self._plot_3sigma_bands(
            fig, truth.t / 60.0, vel_err,
            [(0, "North velocity error"), (1, "East velocity error"),
             (2, "Down velocity error")],
            "m/s")
        return fig

    def _figure_cep(self, result) -> Figure:
        """Every run's horizontal error trace, in NM vs decimal hours."""
        t_hr = result.truth.t / 3600.0
        horiz_err_nm = _horiz_err_nm(result)

        fig = Figure(layout="constrained")
        ax = fig.add_subplot()
        ax.plot(t_hr, horiz_err_nm.T, color="steelblue", alpha=0.3, lw=0.7)
        ax.plot([], [], color="steelblue", lw=1.0, label="Individual runs")

        # Drift-rate fit to the ensemble CEP over the first hour; the CEP
        # curve itself is tabulated below the plot rather than drawn.
        cep = np.percentile(horiz_err_nm, 50, axis=0)
        mask = t_hr <= 1.0
        if mask.sum() >= 2:
            coeffs = np.polyfit(t_hr[mask], cep[mask], 1)
            ax.plot(t_hr, np.polyval(coeffs, t_hr), "--", color="darkorange",
                    lw=1.8, label=f"CEP linear fit  {coeffs[0]:.2f} NM/hr")
            ax.axvline(1.0, color="gray", lw=0.8, linestyle=":")
        ax.set_xlabel("Time (hr)")
        ax.set_ylabel("Horizontal error (NM)")
        ax.set_title(f"Horizontal position error — {result.n_trials} runs")
        ax.legend()
        _style_axes(ax)
        return fig

    def _cep_table(self, result) -> QTableWidget | None:
        """Ensemble percentiles at whole-hour marks, shown under the CEP plot.

        One column per whole hour of flight; rows are the 50th (CEP) and
        95th percentile of the runs' horizontal error at that instant.
        Returns None when the flight is shorter than one hour.
        """
        truth = result.truth
        horiz_err_nm = _horiz_err_nm(result)
        hours = range(1, int(truth.t[-1] // 3600.0) + 1)

        table = QTableWidget(2, len(hours))
        if table.columnCount() == 0:
            return None
        table.setHorizontalHeaderLabels([str(h) for h in hours])
        table.setVerticalHeaderLabels(
            ["50th percentile (NM)", "95th percentile (NM)"])
        for col, h in enumerate(hours):
            idx = int(np.argmin(np.abs(truth.t - h * 3600.0)))
            sample = horiz_err_nm[:, idx]
            for row, pct in enumerate((50, 95)):
                item = QTableWidgetItem(f"{np.percentile(sample, pct):.2f}")
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                table.setItem(row, col, item)

        table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch)
        table.setFixedHeight(
            table.horizontalHeader().sizeHint().height()
            + table.rowHeight(0) + table.rowHeight(1)
            + 2 * table.frameWidth())
        return table

    def _figure_position_errors(self, result) -> Figure:
        """NED position errors with ±3σ bounds [m]."""
        truth = result.truth
        pos_err = result.pos_runs - truth.pos_n[None, :, :]

        fig = Figure(layout="constrained")
        self._plot_3sigma_bands(
            fig, truth.t / 60.0, pos_err,
            [(0, "North position error"), (1, "East position error"),
             (2, "Down position error")],
            "m")
        return fig
