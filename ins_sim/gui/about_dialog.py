"""About dialog for the desktop application."""

from PySide6.QtCore import qVersion
from PySide6.QtWidgets import QDialog, QDialogButtonBox, QLabel, QPlainTextEdit, QVBoxLayout

from ins_sim.gui.about_info import component_versions, third_party_notices


class AboutDialog(QDialog):
    """Show installed versions, project links and scrollable license notices."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("About INS Monte Carlo Simulator")
        self.resize(650, 550)

        layout = QVBoxLayout(self)
        heading = QLabel("INS Monte Carlo Simulator")
        heading.setStyleSheet("font-size: 18px; font-weight: 600")
        layout.addWidget(heading)

        versions = QLabel("<br>".join(
            f"<b>{name}:</b> {version}" for name, version in component_versions(qVersion())
        ))
        layout.addWidget(versions)

        links = QLabel(
            '<a href="https://jorgieo.github.io/ins_sim/">Documentation</a> · '
            '<a href="https://github.com/jorgieo/ins_sim">Source repository</a>'
        )
        links.setOpenExternalLinks(True)
        layout.addWidget(links)

        layout.addWidget(QLabel("Third-party notices"))
        notices = QPlainTextEdit()
        notices.setReadOnly(True)
        notices.setPlainText(third_party_notices())
        layout.addWidget(notices, stretch=1)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
