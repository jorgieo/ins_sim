"""Tests the release check that every compiled file in the app bundle is credited.

tools/check_bundle.py fails the release build when a compiled file matches no
line of .github/bundle-inventory.txt. These tests check the inventory itself:
every component it names is credited in THIRD-PARTY-NOTICES.txt, the files
the bundles contain match the component that credits them, and the Qt modules
that The Qt Company offers only under the GPL-3.0 match nothing.
"""

from pathlib import Path

import pytest

from tools.check_bundle import (
    INVENTORY,
    compiled_files,
    component_for,
    load_inventory,
    main,
    pattern_to_regex,
)

ROOT = Path(__file__).resolve().parents[1]
NOTICES = (ROOT / "THIRD-PARTY-NOTICES.txt").read_text(encoding="utf-8")
RULES = load_inventory(INVENTORY)

# Files from the Windows and Linux bundles, and the component credited for each.
SHIPPED = [
    ("ins_sim.exe", "PyInstaller"),
    ("ins_sim", "PyInstaller"),
    ("_internal/PySide6/Qt6WebEngineCore.dll", "Qt"),
    ("_internal/PySide6/Qt/lib/libQt6XcbQpa.so.6", "Qt"),
    ("_internal/PySide6/plugins/platforms/qwindows.dll", "Qt"),
    ("_internal/PySide6/Qt/plugins/platforminputcontexts/libibusplatforminputcontextplugin.so", "Qt"),
    ("_internal/PySide6/QtWebEngineProcess.exe", "Qt"),
    ("_internal/PySide6/Qt/libexec/QtWebEngineProcess", "Qt"),
    ("_internal/PySide6/opengl32sw.dll", "Qt"),
    ("_internal/PySide6/QtWebEngineWidgets.pyd", "Qt"),
    ("_internal/PySide6/QtCore.abi3.so", "Qt"),
    ("_internal/shiboken6/libshiboken6.abi3.so.6.11", "Qt"),
    ("_internal/PySide6/Qt/lib/libicudata.so.73", "ICU"),
    ("_internal/VCRUNTIME140.dll", "Microsoft"),
    ("_internal/PySide6/MSVCP140_2.dll", "Microsoft"),
    ("_internal/api-ms-win-crt-math-l1-1-0.dll", "Microsoft"),
    ("_internal/python312.dll", "Python"),
    ("_internal/libpython3.12.so.1.0", "Python"),
    ("_internal/_ssl.pyd", "Python"),
    ("_internal/python3.12/lib-dynload/_ssl.cpython-312-x86_64-linux-gnu.so", "Python"),
    ("_internal/libcrypto-3.dll", "Python"),
    ("_internal/numpy/core/_multiarray_umath.cp312-win_amd64.pyd", "NumPy"),
    ("_internal/numpy.libs/libquadmath-96973f99.so.0.0.0", "NumPy"),
    ("_internal/scipy.libs/libgfortran-040039e1.so.5.0.0", "SciPy"),
    ("_internal/yaml/_yaml.cp312-win_amd64.pyd", "PyYAML"),
]

# Qt modules that The Qt Company offers only under the GPL-3.0 (or a
# commercial license), which a PyInstaller hook collecting QML plugins would
# otherwise bring in.
GPL_ONLY_QT = ["Charts", "ChartsQml", "DataVisualization", "Graphs", "Quick3D",
               "Quick3DRuntimeRender", "QuickTimeline", "VirtualKeyboard",
               "VirtualKeyboardSettings", "WaylandCompositor"]

NOT_CREDITED = [
    *(f"_internal/PySide6/Qt6{module}.dll" for module in GPL_ONLY_QT),
    *(f"_internal/PySide6/Qt/lib/libQt6{module}.so.6" for module in GPL_ONLY_QT),
    "_internal/PySide6/plugins/platforminputcontexts/qtvirtualkeyboardplugin.dll",
    "_internal/PySide6/Qt/plugins/platforminputcontexts/libqtvirtualkeyboardplugin.so",
    "_internal/PySide6/qml/QtCharts/qtchartsqml2plugin.dll",
    "_internal/PySide6/plugins/tls/qopensslbackend.dll",
    "_internal/libcrypto-3-x64.dll",          # an OpenSSL found on PATH
    "_internal/PySide6/avcodec-61.dll",       # FFmpeg, in the PySide6 wheels
    "_internal/libreadline.so.8",             # a library from the build machine
    "_internal/numpy_extra/x.so",             # near misses of the patterns
    "_internal/nested/_ssl.pyd",
]


def test_every_component_is_credited_in_the_notices():
    components = {component for component, _pattern, _regex in RULES}
    assert components, "the inventory should list the bundle's components"
    for component in components:
        assert component.lower() in NOTICES.lower(), (
            f"bundle-inventory.txt credits files to {component}, "
            "which THIRD-PARTY-NOTICES.txt does not mention")


@pytest.mark.parametrize("path, component", SHIPPED, ids=[p for p, _ in SHIPPED])
def test_shipped_file_matches_its_component(path, component):
    assert component_for(path, RULES) == component


@pytest.mark.parametrize("path", NOT_CREDITED)
def test_file_without_a_notice_matches_nothing(path):
    assert component_for(path, RULES) is None


@pytest.mark.parametrize("pattern, path, matches", [
    ("_internal/*.pyd", "_internal/_ssl.pyd", True),
    ("_internal/*.pyd", "_internal/numpy/x.pyd", False),     # * stays in one folder
    ("_internal/numpy/**", "_internal/numpy/core/x.pyd", True),
    ("a/{,b/}c{1,2}.dll", "a/c1.dll", True),                   # empty alternative
    ("a/{,b/}c{1,2}.dll", "A/B/C2.DLL", True),                 # case does not matter
    ("a/{,b/}c{1,2}.dll", "a/b/c3.dll", False),
    ("lib{Qt*.so,x}", "libQt6Core.so", True),                  # wildcards in alternatives
    ("a.b", "aXb", False),                                     # dots are literal
])
def test_pattern_syntax(pattern, path, matches):
    assert bool(pattern_to_regex(pattern).fullmatch(path)) is matches


# The smallest headers the check recognises: a Windows PE file ("MZ", then
# the offset of its "PE" signature at byte 0x3C) and a Linux ELF file.
PE_HEADER = b"MZ" + bytes(58) + (64).to_bytes(4, "little") + b"PE\0\0"
ELF_HEADER = b"\x7fELF\x02\x01\x01"


def test_check_fails_on_a_compiled_file_without_a_notice(tmp_path, capsys):
    bundle = tmp_path / "ins_sim"
    (bundle / "_internal" / "numpy").mkdir(parents=True)
    (bundle / "ins_sim.exe").write_bytes(PE_HEADER)
    (bundle / "_internal" / "numpy" / "core.pyd").write_bytes(PE_HEADER)
    (bundle / "_internal" / "base_library.zip").write_bytes(b"PK\x03\x04")
    (bundle / "_internal" / "notes.txt").write_text("MZ starts this text, but it has no PE header" * 3)
    assert compiled_files(bundle) == ["_internal/numpy/core.pyd", "ins_sim.exe"]
    assert main([str(bundle)]) == 0

    (bundle / "_internal" / "libunknown.so.1").write_bytes(ELF_HEADER)
    assert main([str(bundle), "--list"]) == 1
    out = capsys.readouterr().out
    assert "_internal/libunknown.so.1" in out.split("match no line")[1]
