"""Settings > Reset settings to defaults (#42).

The app runs offscreen in a subprocess. Skipped without PySide6.
"""

import json
import os
import subprocess
import sys
import textwrap

import pytest

pytest.importorskip("PySide6.QtWebEngineWidgets")

_SCRIPT = textwrap.dedent("""
    import json, sys
    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication
    from ins_sim.gui.main_window import MainWindow

    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    report = {"fresh": w.current_config()}

    def change_everything():
        for selector, name in ((w.file_selector, "other_mission.yaml"),
                               (w.imu_spec_selector, "other_imu.yaml")):
            selector._combo.addItem(name, userData=f"/elsewhere/{name}")
            selector._combo.setCurrentIndex(selector._combo.count() - 1)
        w.iterations_spinbox.setValue(7)
        w.dt_spinbox.setValue(1.0)
        w.baro_aiding_checkbox.setChecked(False)
        w.advanced_options.mechanization_combo.setCurrentIndex(1)
        w.advanced_options.oversample_spinbox.setValue(10)
        for box in w.visualization_options._checkboxes.values():
            box.setChecked(False)

    change_everything()
    report["changed"] = w.current_config()
    w.reset_settings_action.trigger()
    report["reset"] = w.current_config()

    def during_run():
        # The action is disabled, and a direct call changes nothing.
        w.iterations_spinbox.setValue(2)
        w.dt_spinbox.setValue(1.0)
        w.run_button.click()
        report["enabled_running"] = w.reset_settings_action.isEnabled()
        w.reset_settings()
        report["trials_running"] = w.current_config()["n_iterations"]
        w.cancel_run()
        QTimer.singleShot(100, done)

    def done():
        if w.is_running():
            return QTimer.singleShot(100, done)
        report["enabled_after"] = w.reset_settings_action.isEnabled()
        report["menus"] = [a.text() for a in w.menuBar().actions()]
        app.quit()

    QTimer.singleShot(0, during_run)
    QTimer.singleShot(60_000, lambda: app.exit(3))   # watchdog: never hang CI
    code = app.exec()
    print("REPORT", json.dumps(report), flush=True)
    sys.exit(code)
""")


def test_reset_settings_restores_a_fresh_config():
    env = {**os.environ, "QT_QPA_PLATFORM": "offscreen"}
    proc = subprocess.run([sys.executable, "-c", _SCRIPT],
                          capture_output=True, text=True, timeout=120, env=env)
    assert proc.returncode == 0, proc.stderr
    line = next(l for l in proc.stdout.splitlines() if l.startswith("REPORT "))
    report = json.loads(line.removeprefix("REPORT "))

    fresh, changed = report["fresh"], report["changed"]
    # Every setting was changed, so the reset is checked for each of them.
    assert all(changed[key] != fresh[key] for key in fresh), (fresh, changed)
    assert report["reset"] == fresh
    assert report["enabled_running"] is False
    assert report["trials_running"] == 2
    assert report["enabled_after"] is True
    assert report["menus"] == ["&Settings"]
