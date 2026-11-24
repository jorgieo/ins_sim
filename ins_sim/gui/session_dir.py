"""Per-session temp folders for the plot pages, and removal of abandoned ones.

Each app session writes its plotly pages into its own temp folder. The
folder holds a ``.lock`` file that the session keeps locked while it runs,
so another session can tell an abandoned folder (the app quit or crashed,
and the OS released the lock) from one that is still in use. On start, the
app removes the abandoned folders; on quit, it removes its own.

Qt-free, so it is collected and tested without PySide6.
"""

import os
import re
import shutil
import sys
import tempfile
import time
from pathlib import Path

SESSION_PREFIX = "ins_sim_plots_"
LOCK_NAME = ".lock"

#: Folders made by versions up to 0.4.0: ``tempfile.mkdtemp(prefix="ins_sim_")``,
#: which appends 8 characters. Other ``ins_sim_*`` folders (such as the
#: screenshot tool's) never match.
_LEGACY_NAME = re.compile(r"ins_sim_[a-z0-9_]{8}")
#: Files a legacy session folder can contain: plot pages and plotly.js.
_LEGACY_FILE = re.compile(r"[\w.-]+\.html|plotly(\.min)?\.js")
#: A lock-less folder younger than this may belong to a session that is
#: still running (an older version, or one between creating the folder and
#: its lock), so it is left alone [s].
UNLOCKED_MIN_AGE_S = 3600.0

if sys.platform == "win32":
    import msvcrt

    def _try_lock(fh) -> bool:
        fh.seek(0)
        try:
            msvcrt.locking(fh.fileno(), msvcrt.LK_NBLCK, 1)
        except OSError:
            return False
        return True

    def _unlock(fh) -> None:
        fh.seek(0)
        msvcrt.locking(fh.fileno(), msvcrt.LK_UNLCK, 1)
else:
    import fcntl

    def _try_lock(fh) -> bool:
        try:
            fcntl.flock(fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            return False
        return True

    def _unlock(fh) -> None:
        fcntl.flock(fh.fileno(), fcntl.LOCK_UN)


class SessionDir:
    """A temp folder owned by this session, locked until `close`.

    Args:
        parent: Folder to create it in. Defaults to the system temp folder.

    Examples:
        >>> import tempfile
        >>> from pathlib import Path
        >>> from ins_sim.gui.session_dir import SessionDir
        >>> with tempfile.TemporaryDirectory() as parent:
        ...     session = SessionDir(parent)
        ...     _ = (session.path / "cep.html").write_text("<html></html>")
        ...     session.close()
        ...     list(Path(parent).iterdir())
        []
    """

    def __init__(self, parent: str | os.PathLike | None = None):
        self.path = Path(tempfile.mkdtemp(prefix=SESSION_PREFIX, dir=parent))
        self._lock = open(self.path / LOCK_NAME, "w+b")
        self._lock.write(b"\0")
        self._lock.flush()
        if not _try_lock(self._lock):          # pragma: no cover - fresh file
            raise OSError(f"could not lock {self.path / LOCK_NAME}")

    def close(self) -> None:
        """Releases the lock and deletes the folder. Safe to call twice."""
        if self._lock is None:
            return
        try:
            _unlock(self._lock)
        except OSError:
            pass
        self._lock.close()
        self._lock = None
        shutil.rmtree(self.path, ignore_errors=True)

    def __enter__(self) -> "SessionDir":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()


def _age_s(path: Path) -> float:
    return time.time() - path.stat().st_mtime


def _is_abandoned(folder: Path, min_age_s: float) -> bool:
    """True if folder is a session folder that no running session uses."""
    name = folder.name
    if name.startswith(SESSION_PREFIX):
        lock_path = folder / LOCK_NAME
        if not lock_path.exists():
            return _age_s(folder) > min_age_s
        with open(lock_path, "r+b") as fh:
            if not _try_lock(fh):
                return False                   # a running session holds it
            _unlock(fh)
        return True
    if _LEGACY_NAME.fullmatch(name):
        entries = list(folder.iterdir())
        return (all(e.is_file() and _LEGACY_FILE.fullmatch(e.name) for e in entries)
                and _age_s(folder) > min_age_s)
    return False


def remove_abandoned(parent: str | os.PathLike | None = None,
                     min_age_s: float = UNLOCKED_MIN_AGE_S) -> list[Path]:
    """Deletes the plot folders that earlier sessions left behind.

    A folder is removed when it was made by this module and its lock is free,
    or when it is a lock-less folder (from a version up to 0.4.0, or one
    whose lock was never written) older than ``min_age_s`` that holds only
    plot pages and plotly.js. Anything else is left alone, and errors are
    skipped rather than raised, so startup never fails here.

    Args:
        parent: Folder to search. Defaults to the system temp folder.
        min_age_s: Minimum age of a lock-less folder before it is removed
            [s]. Defaults to `UNLOCKED_MIN_AGE_S`.

    Returns:
        list[pathlib.Path]: The folders that were removed.
    """
    parent = Path(parent) if parent is not None else Path(tempfile.gettempdir())
    removed = []
    try:
        candidates = [p for p in parent.iterdir() if p.name.startswith("ins_sim_")]
    except OSError:
        return removed
    for folder in candidates:
        try:
            if folder.is_symlink() or not folder.is_dir():
                continue
            if not _is_abandoned(folder, min_age_s):
                continue
            shutil.rmtree(folder)
        except OSError:
            continue
        removed.append(folder)
    return removed
