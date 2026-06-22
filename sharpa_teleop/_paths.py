"""Repository and submodule paths for sharpa_teleop."""

from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent
REPO_ROOT = PACKAGE_ROOT.parent
FA_ROOT = REPO_ROOT / "finger_aloha"
SHARPA_CONTROLLER_ROOT = REPO_ROOT / "sharpa_controller"
CONF_DIR = PACKAGE_ROOT / "conf"
FA_CONF_DIR = (FA_ROOT / "hand_interfaces" / "conf").resolve()


def setup_import_paths() -> None:
    """Add finger_aloha and sharpa_controller to sys.path."""
    for path in (FA_ROOT, SHARPA_CONTROLLER_ROOT):
        entry = str(path)
        if entry not in sys.path:
            sys.path.insert(0, entry)
