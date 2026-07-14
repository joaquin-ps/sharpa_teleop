"""Repository paths for sharpa_teleop."""

from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent
REPO_ROOT = PACKAGE_ROOT.parent
DITTO_ROOT = REPO_ROOT / "ditto"
SHARPA_CONTROLLER_ROOT = REPO_ROOT / "sharpa_controller"
HAPTICS_DIR = REPO_ROOT / "ditto_haptics"
CONF_DIR = PACKAGE_ROOT / "conf"
DITTO_CONF_DIR = (DITTO_ROOT / "hand_interfaces" / "conf").resolve()


def setup_import_paths() -> None:
    """Add package root, repo root (ditto), sharpa_controller, and ditto_haptics to sys.path."""
    for path in (PACKAGE_ROOT, REPO_ROOT, SHARPA_CONTROLLER_ROOT, HAPTICS_DIR):
        entry = str(path)
        if entry not in sys.path:
            sys.path.insert(0, entry)
