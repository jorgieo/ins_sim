"""PyInstaller entry point for the INS Monte Carlo Simulator GUI.

Normal launch runs the full application event loop. With INS_SIM_SMOKE=1
(used by the release pipeline, typically with QT_QPA_PLATFORM=offscreen)
the window runs a tiny simulation and the app exits once Qt WebEngine has
drawn its first plot, proving the frozen bundle's imports, Qt plugins,
WebEngine runtime and packaged config files all work without a display.
"""

import os
import sys


def _smoke_test() -> int:
    from PySide6.QtCore import QTimer
    from PySide6.QtWebEngineWidgets import QWebEngineView
    from PySide6.QtWidgets import QApplication

    from ins_sim.gui.main_window import MainWindow

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    # The packaged YAMLs must be discoverable inside the bundle.
    config = window.current_config()
    assert config["trajectory_path"], "no packaged trajectory YAML found"
    assert config["imu_spec_path"], "no packaged IMU spec YAML found"

    # End-to-end: a tiny Monte Carlo run must complete, build the result
    # tabs, and Qt WebEngine must draw the first tab's plotly figure (the
    # engine, the figures and the WebEngine runtime all work inside the
    # frozen bundle). Counting tabs alone would pass even if WebEngine
    # could not render a page.
    window.iterations_spinbox.setValue(3)
    window.dt_spinbox.setValue(1.0)
    status = {"code": 1}

    def plots_drawn(count) -> None:
        if count:   # 0 or None until plotly has drawn the figure
            tabs = window.visualization_panel.count()
            status["code"] = 0
            print(f"smoke: simulation finished, {tabs} tabs, first plot drawn")
            app.quit()

    def check_done() -> None:
        if window.last_result is None:
            return
        view = window.visualization_panel.findChild(QWebEngineView)
        if view is None:
            status["code"] = 2
            print("smoke: simulation finished, but no tab shows a web view")
            app.quit()
            return
        view.page().runJavaScript(
            "document.querySelectorAll('.js-plotly-plot').length", 0, plots_drawn)

    poll = QTimer()
    poll.setInterval(500)
    poll.timeout.connect(check_done)
    poll.start()
    QTimer.singleShot(180_000, app.quit)   # hard timeout keeps code 1
    QTimer.singleShot(0, window.run_button.click)
    app.exec()
    return status["code"]


if __name__ == "__main__":
    if os.environ.get("INS_SIM_SMOKE") == "1":
        sys.exit(_smoke_test())
    from ins_sim.gui.main_window import main
    main()
