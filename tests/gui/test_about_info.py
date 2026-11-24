"""The About dialog must find notices in both deployment layouts."""

from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import patch
from tempfile import TemporaryDirectory
from pathlib import Path

from ins_sim.gui import about_info


class AboutInfoTests(TestCase):
    def test_checkout_notices_are_readable(self):
        notices = about_info.third_party_notices()
        self.assertIn("PySide6", notices)
        self.assertIn("Qt", notices)


    def test_frozen_notices_use_bundle_data_root(self):
        with TemporaryDirectory() as temporary:
            notice = Path(temporary) / "THIRD-PARTY-NOTICES.txt"
            notice.write_text("Bundled notices", encoding="utf-8")
            with patch.object(about_info, "sys", SimpleNamespace(
                frozen=True, _MEIPASS=temporary
            )):
                self.assertEqual(about_info.notices_path(), notice)
                self.assertEqual(about_info.third_party_notices(), "Bundled notices")


    def test_versions_have_required_components(self):
        names = [name for name, _ in about_info.component_versions("6.8.0")]
        self.assertEqual(names, [
            "ins_sim", "Python", "PySide6", "Qt", "NumPy", "SciPy", "Plotly"
        ])
