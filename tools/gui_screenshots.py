"""Takes the documentation's screenshots of the desktop app.

Runs the GUI on the packaged mission and IMU spec with the settings of the
"Your first simulation" tutorial (10 trials, 1 s time step) and saves:

- ``main_window.png``: the window after the run, on the CEP tab;
- ``map_tab.png``: the Map tab of the same run;
- ``free_inertial.png``: the Position Errors tab after a second run with
  baro altitude aiding switched off.

The window opens on screen while the screenshots are taken, and the Map tab
needs an internet connection for its basemap tiles. Run it through
``python tools/generate_doc_plots.py --guide``.
"""

import os
import sys
import tempfile
from pathlib import Path

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from ins_sim.gui.main_window import MainWindow

WINDOW_SIZE = (1500, 900)    # logical pixels
TRIALS = 10
TIME_STEP_S = 1.0
PLOT_SETTLE_MS = 5_000       # time for a plotly page to finish drawing
MAP_SETTLE_MS = 10_000       # basemap tiles come over the network
POLL_MS = 250
TIMEOUT_MS = 300_000


def take_screenshots(out_dir: Path) -> None:
    """Runs the tutorial's simulations in the GUI and saves the screenshots.

    Run from source, the app keeps its mission and IMU-spec folders in the
    current folder. The screenshots are taken from an empty temporary
    folder, so the app starts from fresh copies of the packaged files
    rather than from any local edits.

    Args:
        out_dir: Folder to write the PNG files into.
    """
    out_dir = Path(out_dir).resolve()
    previous_cwd = Path.cwd()
    with tempfile.TemporaryDirectory(prefix="ins_sim_screenshots_",
                                     ignore_cleanup_errors=True) as home:
        os.chdir(home)
        try:
            _run_tutorial(out_dir)
        finally:
            os.chdir(previous_cwd)


def _run_tutorial(out_dir: Path) -> None:
    """Drives the main window through the tutorial's two runs."""
    app = QApplication.instance() or QApplication(sys.argv)
    window = MainWindow()
    window.resize(*WINDOW_SIZE)
    window.show()
    window.iterations_spinbox.setValue(TRIALS)
    window.dt_spinbox.setValue(TIME_STEP_S)
    tabs = window.visualization_panel

    def save(name: str) -> None:
        if not window.grab().save(str(out_dir / name)):
            raise OSError(f"could not write {out_dir / name}")
        print(f"  {name}", flush=True)

    def select_tab(title: str) -> None:
        titles = [tabs.tabText(i) for i in range(tabs.count())]
        tabs.setCurrentIndex(titles.index(title))

    def run():
        """Clicks Run and waits until the results are shown."""
        previous = window.last_result
        window.run_button.click()
        while window.last_result is previous or not window.run_button.isEnabled():
            yield POLL_MS

    def steps():
        yield from run()
        yield PLOT_SETTLE_MS
        save("main_window.png")
        select_tab("Map")
        yield MAP_SETTLE_MS
        save("map_tab.png")
        window.baro_aiding_checkbox.setChecked(False)
        yield from run()
        select_tab("Position Errors")
        yield PLOT_SETTLE_MS
        save("free_inertial.png")

    sequence = steps()
    outcome = {"finished": False, "error": None}

    def advance() -> None:
        try:
            delay = next(sequence)
        except StopIteration:
            outcome["finished"] = True
            app.quit()
        except Exception as error:  # a step failed: stop, and raise it below
            outcome["error"] = error
            app.quit()
        else:
            QTimer.singleShot(delay, advance)

    QTimer.singleShot(0, advance)
    QTimer.singleShot(TIMEOUT_MS, app.quit)
    app.exec()

    # A simulation still running at the deadline cannot be cancelled: let it
    # finish, so that its thread is not destroyed while it runs. The thread
    # normally stops on a request queued to the event loop that has just
    # quit, so ask it to stop directly first.
    thread = window._thread
    if thread is not None:
        thread.quit()
        thread.wait()

    # Stopping early must not look like success, or the screenshots would
    # silently stay as they were.
    if outcome["error"] is not None:
        raise RuntimeError("Taking the screenshots failed") from outcome["error"]
    if not outcome["finished"]:
        raise TimeoutError(
            f"The screenshots were not finished within {TIMEOUT_MS // 1000} s")
