"""Checks that THIRD-PARTY-NOTICES.txt credits every compiled file in a release bundle.

The release workflow runs this on the PyInstaller bundle after building it.
Every compiled file (an executable, a shared library or a Python extension
module, recognised by its Windows or Linux file header) must match a line of
``.github/bundle-inventory.txt``, which names the part of the notices that
credits it. Any file that matches no line is reported and fails the build, so
a library cannot ship without a notice: a new package version, a PyInstaller
hook that collects more, or a library copied from the build machine.

Symbolic links are skipped, since each one points at a file that is checked
itself.

Usage:
    python tools/check_bundle.py dist/ins_sim          # summary and failures
    python tools/check_bundle.py dist/ins_sim --list   # also every file
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from collections import Counter
from pathlib import Path

INVENTORY = Path(__file__).resolve().parents[1] / ".github" / "bundle-inventory.txt"


def is_compiled(path: Path) -> bool:
    """Whether a file is a Windows (PE) or Linux (ELF) executable or library.

    A PE file starts with "MZ" and holds the offset of its "PE\\0\\0"
    signature at byte 0x3C; an ELF file starts with its magic number.
    """
    with open(path, "rb") as f:
        head = f.read(64)
        if head.startswith(b"\x7fELF"):
            return True
        if not head.startswith(b"MZ") or len(head) < 64:
            return False
        f.seek(int.from_bytes(head[0x3C:0x40], "little"))
        return f.read(4) == b"PE\0\0"


def pattern_to_regex(pattern: str) -> re.Pattern[str]:
    """Translates an inventory pattern into a case-insensitive regex.

    ``*`` matches within one folder, ``**`` across folders, ``?`` one
    character, and ``{a,b}`` either alternative (alternatives may be empty
    and may hold wildcards, but not further braces).

    Examples:
        >>> bool(pattern_to_regex("_internal/*.pyd").fullmatch("_internal/_ssl.pyd"))
        True
        >>> bool(pattern_to_regex("_internal/*.pyd").fullmatch("_internal/numpy/x.pyd"))
        False
        >>> bool(pattern_to_regex("a/{,b/}c{1,2}.dll").fullmatch("A/B/C2.DLL"))
        True
    """
    def translate(text: str) -> str:
        out, i = [], 0
        while i < len(text):
            if text.startswith("**", i):
                out.append(".*")
                i += 2
            elif text[i] == "*":
                out.append("[^/]*")
                i += 1
            elif text[i] == "?":
                out.append("[^/]")
                i += 1
            elif text[i] == "{":
                end = text.index("}", i)
                options = text[i + 1:end].split(",")
                out.append("(?:" + "|".join(translate(o) for o in options) + ")")
                i = end + 1
            else:
                out.append(re.escape(text[i]))
                i += 1
        return "".join(out)

    return re.compile(translate(pattern), re.IGNORECASE)


def load_inventory(path: Path = INVENTORY) -> list[tuple[str, str, re.Pattern[str]]]:
    """Reads the inventory as (component, pattern, regex) rules, in file order."""
    rules = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        try:
            component, pattern = line.split()
        except ValueError:
            raise ValueError(f"{path.name}:{number}: expected '<component> <pattern>'") from None
        rules.append((component, pattern, pattern_to_regex(pattern)))
    return rules


def component_for(relative_path: str, rules) -> str | None:
    """The component of the first rule matching a bundle path, or None."""
    for component, _pattern, regex in rules:
        if regex.fullmatch(relative_path):
            return component
    return None


def compiled_files(bundle: Path) -> list[str]:
    """Paths, relative to the bundle and with forward slashes, of its compiled files."""
    found = []
    for folder, _dirs, files in os.walk(bundle):
        for name in files:
            path = Path(folder, name)
            if not path.is_symlink() and is_compiled(path):
                found.append(path.relative_to(bundle).as_posix())
    return sorted(found)


def check(bundle: Path, rules, show_all: bool = False) -> list[str]:
    """Prints the bundle's compiled files by component; returns the unmatched ones."""
    counts, unmatched = Counter(), []
    for relative in compiled_files(bundle):
        component = component_for(relative, rules)
        if component is None:
            unmatched.append(relative)
        else:
            counts[component] += 1
        if show_all:
            print(f"  {component or 'NOT IN THE INVENTORY':12s} {relative}")
    total = sum(counts.values()) + len(unmatched)
    print(f"{total} compiled files in {bundle}:")
    for component, count in sorted(counts.items()):
        print(f"  {count:4d}  {component}")
    if unmatched:
        print(f"\n{len(unmatched)} compiled files match no line of {INVENTORY.name}, so "
              "THIRD-PARTY-NOTICES.txt may not credit them:")
        for relative in unmatched:
            print(f"  {relative}")
    return unmatched


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bundle", type=Path, help="the bundle folder, e.g. dist/ins_sim")
    parser.add_argument("--list", action="store_true", help="also list every compiled file")
    args = parser.parse_args(argv)
    if not args.bundle.is_dir():
        parser.error(f"{args.bundle} is not a folder")
    unmatched = check(args.bundle, load_inventory(), show_all=args.list)
    return 1 if unmatched else 0


if __name__ == "__main__":
    sys.exit(main())
