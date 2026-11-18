"""Keeps the documentation's mathematical notation consistent.

Every frame-indexed quantity in docs/ must be typeset through the MathJax
macros in docs/javascripts/mathjax.js (\\Cmat, \\wvec, \\fvec, ...), so the
Groves (2013) notation is defined in exactly one place. This test fails on:

- raw frame-indexed symbols such as ``\\mathbf{C}_b^n`` or
  ``\\boldsymbol{\\omega}_{ib}`` written outside the macros;
- ``R_M``, the pre-0.3 name of the meridian radius (Groves calls it R_N);
- the ``[x \\times]`` skew-symmetric form (Groves writes ``[x \\wedge]``,
  available as ``\\skew{x}``).
"""

import re
from pathlib import Path

import pytest

DOCS = Path(__file__).resolve().parents[1] / "docs"

# Pages still written in the v0.2 notation, exempt until they are rewritten.
# Remove an entry when its page is converted to Groves notation. Every page
# has been converted; the mechanism stays for pages imported later.
LEGACY_PAGES: set[str] = set()

# The notation page shows raw symbols and other textbooks' R_M on purpose
# (the anatomy of a symbol, the divergence warning), so only the skew rule
# applies to it.
NOTATION_PAGE = "theory/notation.md"

# The migration guide names v0.2's R_M on purpose, in its rename table.
MIGRATION_PAGE = "project/migrating-from-0.2.md"

RAW_FRAME_SYMBOL = re.compile(
    r"\\mathbf\{[CfvrgqW]\}\s*_"
    r"|\\boldsymbol\{\\(?:omega|Omega|alpha|upsilon|psi)\}\s*_"
)
LEGACY_RADIUS = re.compile(r"\bR_M\b|R_\{M\}")
TIMES_SKEW = re.compile(r"\\times\s*(?:\\right)?\]")


def _pages():
    return sorted(p.relative_to(DOCS).as_posix() for p in DOCS.rglob("*.md"))


def _violations(rel_path: str) -> list[str]:
    text = (DOCS / rel_path).read_text(encoding="utf-8")
    rules = [(TIMES_SKEW, r"write skew-symmetric matrices as \skew{x}, not [x \times]")]
    if rel_path not in (NOTATION_PAGE, MIGRATION_PAGE):
        rules.append((LEGACY_RADIUS, "use R_N (meridian) / R_E (transverse), not R_M"))
    if rel_path != NOTATION_PAGE:
        rules.append((RAW_FRAME_SYMBOL,
                      r"use the notation macros (\Cmat, \wvec, \fvec, ...) for frame-indexed symbols"))
    found = []
    for lineno, line in enumerate(text.splitlines(), start=1):
        for pattern, message in rules:
            if pattern.search(line):
                found.append(f"{rel_path}:{lineno}: {message}\n    {line.strip()}")
    return found


@pytest.mark.parametrize("rel_path", [p for p in _pages() if p not in LEGACY_PAGES])
def test_page_uses_groves_notation(rel_path):
    violations = _violations(rel_path)
    assert not violations, "\n".join(violations)


def test_legacy_list_is_current():
    """Every exempt page must still exist, so the list cannot go stale."""
    missing = sorted(p for p in LEGACY_PAGES | {NOTATION_PAGE, MIGRATION_PAGE}
                     if not (DOCS / p).exists())
    assert not missing, f"these exempt pages no longer exist: {missing}"
