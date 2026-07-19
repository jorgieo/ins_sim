"""PyInstaller entry point for the INS Monte Carlo Simulator GUI.

Normal launch runs the full application event loop. With INS_SIM_SMOKE=1
(used by the release pipeline, typically with QT_QPA_PLATFORM=offscreen)
the window is constructed and the app exits cleanly after a few seconds,
proving the frozen bundle's imports, Qt plugins, and packaged config
files all resolve without needing a display.
"""

import os
import sys


def _smoke_test() -> int:
    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication

    from ins_sim.gui.main_window import MainWindow

    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    # The packaged YAMLs must be discoverable inside the bundle.
    config = window.current_config()
    assert config["trajectory_path"], "no packaged trajectory YAML found"
    assert config["imu_spec_path"], "no packaged IMU spec YAML found"

    # End-to-end: a tiny Monte Carlo run must complete and build the
    # plotly web-view tabs (engine, figures, and QtWebEngine all work
    # inside the frozen bundle).
    window.iterations_spinbox.setValue(3)
    window.dt_spinbox.setValue(1.0)
    status = {"code": 1}

    def check_done() -> None:
        if window.last_result is None:
            return
        tabs = window.visualization_panel.count()
        status["code"] = 0 if tabs > 1 else 2
        print(f"smoke: simulation finished, {tabs} tabs rendered")
        app.quit()

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
