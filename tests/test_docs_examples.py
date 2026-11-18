"""Runs every example script, IMU preset and mission that the documentation includes.

Pages include these files with pymdownx.snippets, so the code readers see
is exactly the code tested here. When a script has a sibling ``.txt`` file,
the page shows that file as the script's output, and this test checks that
the script still prints exactly that. To refresh one after a deliberate
change:

    python docs/examples/<name>.py > docs/examples/<name>.txt

Scripts run in an empty temporary folder, so any file one saves (a plot,
say) lands there rather than in the repository.
"""

import dataclasses
import math
import runpy
from pathlib import Path

import numpy as np
import pytest
import yaml

from ins_sim.sensors.imu import load_imu_spec
from ins_sim.trajectory.kinematics import build_trajectory

EXAMPLES_DIR = Path(__file__).resolve().parents[1] / "docs" / "examples"
EXAMPLES = sorted(EXAMPLES_DIR.glob("*.py"))
IMU_PRESETS = sorted((EXAMPLES_DIR / "imu").glob("*.yaml"))
MISSIONS = sorted((EXAMPLES_DIR / "missions").glob("*.yaml"))


def _lines(text: str) -> list[str]:
    return [line.rstrip() for line in text.strip().splitlines()]


def test_examples_exist():
    assert EXAMPLES, "docs/examples/ should contain the scripts the docs include"
    assert IMU_PRESETS, "docs/examples/imu/ should contain the IMU presets"
    assert MISSIONS, "docs/examples/missions/ should contain the example missions"


@pytest.mark.parametrize("script", EXAMPLES, ids=lambda path: path.name)
def test_docs_example_runs(script, capsys, monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    runpy.run_path(str(script), run_name="__main__")
    out = capsys.readouterr().out
    assert out.strip(), f"{script.name} should print its result"
    expected = script.with_suffix(".txt")
    if expected.exists():
        assert _lines(out) == _lines(expected.read_text(encoding="utf-8")), (
            f"{script.name} no longer prints what {expected.name} shows on its docs page")


@pytest.mark.parametrize("path", IMU_PRESETS, ids=lambda path: path.name)
def test_imu_preset_loads(path):
    # The GUI lists a file in its IMU-spec menu only if it has these sections.
    assert {"gyro", "accel"} <= set(yaml.safe_load(path.read_text(encoding="utf-8")))
    spec = load_imu_spec(str(path))
    for field in dataclasses.fields(spec):
        value = getattr(spec, field.name)
        assert math.isfinite(value) and value >= 0.0, f"{path.name}: {field.name} = {value}"
    assert spec.gyro_bias_tau > 0.0 and spec.accel_bias_tau > 0.0


@pytest.mark.parametrize("path", MISSIONS, ids=lambda path: path.name)
def test_example_mission_builds(path):
    # The GUI lists a file in its trajectory menu only if it has phases.
    assert "phases" in yaml.safe_load(path.read_text(encoding="utf-8"))
    truth, _, _ = build_trajectory(str(path), tau_i=1.0)
    assert truth.t[-1] > 60.0
    for name in ("r_lb_l", "v_eb_n", "omega_ib_b", "f_ib_b"):
        assert np.all(np.isfinite(getattr(truth, name))), name
