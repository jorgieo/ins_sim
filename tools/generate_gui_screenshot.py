"""Captures the GUI screenshot asset for the documentation.

Opens a real (visible) MainWindow on the local display, drives a quick
Monte Carlo run, waits for the plotly tabs to finish painting in the
embedded web views, and saves a window grab to
docs/assets/gui/main_window.png. Run manually on a desktop session
(not CI — QWebEngineView grabs need a real compositor) after UI
changes:

    python tools/generate_gui_screenshot.py
"""

import sys
from pathlib import Path

# Guarantee the repo source tree wins over any installed ins_sim copy.
REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from PySide6.QtCore import QTimer  # noqa: E402
from PySide6.QtGui import QGuiApplication  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from ins_sim.gui.main_window import MainWindow  # noqa: E402

OUT_PATH = REPO_ROOT / "docs" / "assets" / "gui" / "main_window.png"
SETTLE_MS = 4000        # let the web views paint after render()
TIMEOUT_MS = 180_000


def main() -> None:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.resize(1500, 900)
    window.show()

    # Quick but representative run; all visualization boxes default on.
    window.iterations_spinbox.setValue(10)
    window.dt_spinbox.setValue(1.0)

    def check_done() -> None:
        if window.last_result is None:
            return
        poll.stop()
        QTimer.singleShot(SETTLE_MS, capture)

    def capture() -> None:
        # QWidget.grab() paints via QPainter, which Chromium's compositor
        # bypasses (web views come out blank) — grab the real screen
        # pixels of the window instead, so keep it frontmost.
        window.raise_()
        window.activateWindow()
        QTimer.singleShot(500, do_grab)

    def do_grab() -> None:
        OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
        screen = window.screen() or QGuiApplication.primaryScreen()
        pix = screen.grabWindow(window.winId())
        ok = pix.save(str(OUT_PATH))
        print(f"{'wrote' if ok else 'FAILED to write'} "
              f"{OUT_PATH.relative_to(REPO_ROOT)}")
        app.exit(0 if ok else 1)

    poll = QTimer()
    poll.setInterval(300)
    poll.timeout.connect(check_done)
    poll.start()
    QTimer.singleShot(TIMEOUT_MS, lambda: app.exit(1))
    QTimer.singleShot(0, window.run_button.click)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
