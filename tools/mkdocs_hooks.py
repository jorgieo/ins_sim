"""MkDocs build hooks for the ins_sim documentation (registered in mkdocs.yml).

The glossary page and the site-wide hover tooltips share one source of
truth: ``includes/abbreviations.md``. That file is auto-appended to every
page by ``pymdownx.snippets`` (which turns each term into an ``<abbr>``
tooltip), and this hook renders the same entries as a definition list
wherever a page contains the ``<!-- glossary-entries -->`` marker.
"""

import re
from pathlib import Path

ABBREVIATIONS = Path(__file__).resolve().parents[1] / "includes" / "abbreviations.md"
GLOSSARY_MARKER = "<!-- glossary-entries -->"
_ENTRY = re.compile(r"^\*\[(?P<term>[^\]]+)\]:\s*(?P<definition>.+?)\s*$")


def glossary_entries() -> list[tuple[str, str]]:
    """Returns (term, definition) pairs from the abbreviations file, sorted by term."""
    entries = []
    for line in ABBREVIATIONS.read_text(encoding="utf-8").splitlines():
        match = _ENTRY.match(line)
        if match:
            entries.append((match["term"], match["definition"]))
    return sorted(entries, key=lambda entry: entry[0].lower())


def on_page_markdown(markdown, page, config, files):  # noqa: ARG001 (MkDocs hook signature)
    """Expands the glossary marker into a Markdown definition list."""
    if GLOSSARY_MARKER not in markdown:
        return markdown
    body = "\n\n".join(f"{term}\n:   {definition}" for term, definition in glossary_entries())
    return markdown.replace(GLOSSARY_MARKER, body)
