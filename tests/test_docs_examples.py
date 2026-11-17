"""Runs every example script that the documentation includes.

Pages include these files with pymdownx.snippets, so the code readers see
is exactly the code tested here.
"""

import runpy
from pathlib import Path

import pytest

EXAMPLES = sorted((Path(__file__).resolve().parents[1] / "docs" / "examples").glob("*.py"))


def test_examples_exist():
    assert EXAMPLES, "docs/examples/ should contain the scripts the docs include"


@pytest.mark.parametrize("script", EXAMPLES, ids=lambda path: path.name)
def test_docs_example_runs(script, capsys):
    runpy.run_path(str(script), run_name="__main__")
    assert capsys.readouterr().out.strip(), f"{script.name} should print its result"
