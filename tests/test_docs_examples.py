"""Runs every example script that the documentation includes.

Pages include these files with pymdownx.snippets, so the code readers see
is exactly the code tested here. When a script has a sibling ``.txt`` file,
the page shows that file as the script's output, and this test checks that
the script still prints exactly that. To refresh one after a deliberate
change:

    python docs/examples/<name>.py > docs/examples/<name>.txt
"""

import runpy
from pathlib import Path

import pytest

EXAMPLES = sorted((Path(__file__).resolve().parents[1] / "docs" / "examples").glob("*.py"))


def _lines(text: str) -> list[str]:
    return [line.rstrip() for line in text.strip().splitlines()]


def test_examples_exist():
    assert EXAMPLES, "docs/examples/ should contain the scripts the docs include"


@pytest.mark.parametrize("script", EXAMPLES, ids=lambda path: path.name)
def test_docs_example_runs(script, capsys):
    runpy.run_path(str(script), run_name="__main__")
    out = capsys.readouterr().out
    assert out.strip(), f"{script.name} should print its result"
    expected = script.with_suffix(".txt")
    if expected.exists():
        assert _lines(out) == _lines(expected.read_text(encoding="utf-8")), (
            f"{script.name} no longer prints what {expected.name} shows on its docs page")
