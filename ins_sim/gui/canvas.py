"""Tabbed plotly visualization panel for simulation results.

VisualizationPanel is a QTabWidget that renders one interactive plotly
page (hover, zoom, pan, modebar) per selected visualization, from the
SimulationResult a SimulationWorker emits. Figures are built by the
Qt-free ins_sim.gui.figures module, written as HTML into a per-session
temp directory (plotly.js copied there once), and displayed in
QWebEngineView pages.

Note: QtWebEngineWidgets must be imported before QApplication is
created, which is why the import lives at module level — every entry
point imports this module (via main_window) before constructing the
application object.
"""

import tempfile
import threading
import time
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
from PySide6.QtCore import Qt, QUrl
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

try:
    from PySide6.QtWebEngineWidgets import QWebEngineView
except ImportError:      # pragma: no cover - PySide6-Addons not installed
    QWebEngineView = None

from ins_sim.gui import figures


class _QuietHandler(SimpleHTTPRequestHandler):
    """File handler for the figure pages that skips stderr access logging."""

    def log_message(self, *_args) -> None:
        pass

    def handle(self) -> None:
        # A web view torn down mid-download (tab re-render) aborts its
        # socket; that's routine, not worth a stderr traceback.
        try:
            super().handle()
        except ConnectionError:
            pass


def _centered_label(text: str) -> QLabel:
    label = QLabel(text)
    label.setAlignment(Qt.AlignmentFlag.AlignCenter)
    label.setStyleSheet("color: gray; font-size: 14px;")
    return label


class VisualizationPanel(QTabWidget):
    """Holds one plotly web-view tab per selected visualization.

    Call render(result) with a SimulationResult to (re)build the tabs;
    only visualizations checked in result.config["visualizations"] are
    generated. Shows a placeholder tab until the first render.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._html_dir: Path | None = None
        self._server: ThreadingHTTPServer | None = None
        self.show_placeholder()

    def show_placeholder(self) -> None:
        """Clears all tabs and shows the pre-run placeholder."""
        self._clear_pages()
        self.addTab(_centered_label("Run a simulation to see results"),
                    "Results")

    def render(self, result) -> None:
        """Rebuilds the tabs from a SimulationResult, honoring the checkboxes.

        Args:
            result: SimulationResult from SimulationWorker; the
                visualization selection rides in
                result.config["visualizations"].
        """
        builders = [
            ("cep",             "CEP",             figures.figure_cep),
            ("attitude_errors", "Attitude Errors", figures.figure_attitude),
            ("velocity_errors", "Velocity Errors", figures.figure_velocity),
            ("position_errors", "Position Errors", figures.figure_position),
            ("trajectory_3d",   "3D Trajectory",   figures.figure_trajectory_3d),
            ("map",             "Map",             figures.figure_map),
        ]
        self._clear_pages()
        enabled = result.config.get("visualizations", {})
        for slug, title, builder in builders:
            if not enabled.get(slug, False):
                continue
            page = self._web_page(builder(result), f"{slug}.html")
            if slug == "cep":
                table = self._cep_table(result)
                if table is not None:
                    page.layout().addWidget(table) # pyright: ignore[reportOptionalMemberAccess]
            self.addTab(page, title)
        if self.count() == 0:
            self.show_placeholder()

    # ---- Tab plumbing ------------------------------------------------------

    def _clear_pages(self) -> None:
        """Removes and deletes all tab pages.

        QTabWidget.clear() only detaches pages; web views must be
        deleted explicitly or their Chromium render processes leak
        across re-renders.
        """
        while self.count():
            page = self.widget(0)
            self.removeTab(0)
            page.deleteLater() # pyright: ignore[reportOptionalMemberAccess]

    def _ensure_html_dir(self) -> Path:
        """Returns the per-session HTML output directory, creating it once."""
        if self._html_dir is None:
            self._html_dir = Path(tempfile.mkdtemp(prefix="ins_sim_"))
        return self._html_dir

    def _server_port(self) -> int:
        """Starts (once) a localhost HTTP server for the figure pages.

        Pages must be served over http rather than file:// because
        Chromium blocks fetch() from file origins, which MapLibre uses
        to download basemap tiles — a file-loaded map renders overlays
        but no basemap.

        Returns:
            int: The server's ephemeral port on 127.0.0.1.
        """
        if self._server is None:
            handler = partial(_QuietHandler,
                              directory=str(self._ensure_html_dir()))
            self._server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
            threading.Thread(target=self._server.serve_forever,
                             daemon=True).start()
        return self._server.server_address[1]

    def _web_page(self, fig, filename: str) -> QWidget:
        """Wraps a plotly figure in a QWebEngineView page widget.

        The figure is written as HTML next to a shared plotly.js bundle
        (include_plotlyjs="directory" copies it on first use), so pages
        are small and work offline apart from map tiles.

        Args:
            fig: plotly.graph_objects.Figure to display.
            filename: Output HTML file name inside the session dir.

        Returns:
            QWidget: Page containing the web view, or a fallback label
                if QtWebEngine is unavailable.
        """
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 0, 0, 0)

        if QWebEngineView is None:
            layout.addWidget(_centered_label(
                "QtWebEngine is not installed — reinstall the full "
                "PySide6 package to view plotly visualizations."))
            return page

        path = self._ensure_html_dir() / filename
        fig.write_html(str(path), include_plotlyjs="directory",
                       config={"scrollZoom": True})
        view = QWebEngineView(page)
        view.load(QUrl(f"http://127.0.0.1:{self._server_port()}/{filename}"
                       f"?v={time.monotonic_ns()}"))   # query busts the cache
        layout.addWidget(view)
        return page

    # ---- CEP percentile table ---------------------------------------------

    def _cep_table(self, result) -> QTableWidget | None:
        """Ensemble percentiles at whole-hour marks, shown under the CEP plot.

        One column per whole hour of flight; rows are the 50th (CEP) and
        95th percentile of the runs' horizontal error at that instant.
        Returns None when the flight is shorter than one hour.
        """
        truth = result.truth
        horiz_err_nm = figures.horizontal_error_nm(result)
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
