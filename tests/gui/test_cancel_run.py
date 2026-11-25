"""Cancelling a run with the Cancel button or Esc (#18).

The app runs offscreen in a subprocess: one short run to completion, then a
second 5-trial run that is cancelled. Skipped without PySide6.
"""

import json
import os
import re
import subprocess
import sys
import textwrap

import pytest

pytest.importorskip("PySide6.QtWebEngineWidgets")

# SCENARIO: "early" presses Esc as soon as the second run starts; "yes" and
# "no" click Cancel once 2 trials are done and answer the question about
# showing the completed trials.
_SCRIPT = textwrap.dedent("""
    import json, sys
    from PySide6.QtCore import Qt, QTimer
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication, QMessageBox
    from ins_sim.gui.main_window import MainWindow

    scenario = {scenario!r}
    errors, asked = [], []
    QMessageBox.critical = lambda *args: errors.append(args[2])
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    w._confirm_partial = lambda n, m: asked.append([n, m]) or scenario == "yes"
    w.dt_spinbox.setValue(1.0)
    report = {{}}
    state = {{}}

    def later(fn, ms=100):
        QTimer.singleShot(ms, fn)

    def first_run_done():
        if w.is_running():
            return later(first_run_done)
        state["first"] = w.last_result
        report["first_trials"] = w.last_result.n_trials
        w.iterations_spinbox.setValue(5)
        w.run_button.click()
        report["text_running"] = w.run_button.text()
        report["controls_running"] = w.dt_spinbox.isEnabled()
        if scenario == "early":
            QTest.keyClick(w, Qt.Key.Key_Escape)
            return after_cancel()
        wait_for_two_trials()

    def wait_for_two_trials():
        if w.progress_bar.value() < 40:
            return later(wait_for_two_trials, 20)
        w.run_button.click()
        after_cancel()

    def after_cancel():
        report["text_cancelling"] = w.run_button.text()
        report["enabled_cancelling"] = w.run_button.isEnabled()
        later(done)

    def done():
        if w.is_running():
            return later(done)
        report["kept_result"] = w.last_result is state["first"]
        report["n_trials"] = w.last_result.n_trials
        report["asked"] = asked
        report["progress"] = [w.progress_bar.value(), w.progress_bar.format()]
        report["text_after"] = w.run_button.text()
        report["enabled_after"] = w.run_button.isEnabled()
        report["controls_after"] = w.dt_spinbox.isEnabled()
        report["log"] = w.log_view.toPlainText()
        report["errors"] = errors
        app.quit()

    w.iterations_spinbox.setValue(2)
    QTimer.singleShot(0, w.run_button.click)
    QTimer.singleShot(0, first_run_done)
    QTimer.singleShot(120_000, lambda: app.exit(3))  # watchdog: never hang CI
    code = app.exec()
    print("REPORT", json.dumps(report), flush=True)
    sys.exit(code)
""")


def _run(scenario):
    env = {**os.environ, "QT_QPA_PLATFORM": "offscreen"}
    proc = subprocess.run([sys.executable, "-c", _SCRIPT.format(scenario=scenario)],
                          capture_output=True, text=True, timeout=240, env=env)
    assert proc.returncode == 0, proc.stderr
    line = next(l for l in proc.stdout.splitlines() if l.startswith("REPORT "))
    report = json.loads(line.removeprefix("REPORT "))
    common = {"first_trials": 2, "text_running": "Cancel",
              "controls_running": False, "text_cancelling": "Cancelling...",
              "enabled_cancelling": False, "text_after": "Run Simulation",
              "enabled_after": True, "controls_after": True, "errors": []}
    assert {k: report[k] for k in common} == common
    return report


def test_cancel_early_keeps_previous_results():
    report = _run("early")
    assert report["kept_result"]
    assert report["asked"] == []                 # fewer than 2 trials done
    assert report["progress"] == [0, "Cancelled"]
    assert re.search(r"Cancelled after [01]/5 trials\.", report["log"])


def test_cancel_and_show_the_completed_trials():
    report = _run("yes")
    (n_done, n_requested), = report["asked"]
    assert 2 <= n_done <= 5 and n_requested == 5
    assert not report["kept_result"]
    assert report["n_trials"] == n_done
    assert report["progress"] == [100, f"{n_done}/5 trials"]
    assert f"Showing results from {n_done} of 5 trials." in report["log"]


def test_cancel_and_discard_the_completed_trials():
    report = _run("no")
    assert len(report["asked"]) == 1
    assert report["kept_result"]
    assert report["progress"] == [0, "Cancelled"]
