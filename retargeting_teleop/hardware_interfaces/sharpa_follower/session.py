"""Sharpa Wave follower session — command retargeted ``sharpa_q`` to the hand.

Importing this module pulls in the Sharpa Wave SDK (via ``sharpa_hand``), so only
import it when Sharpa follower hardware is actually enabled.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np
from omegaconf import DictConfig

from _paths import setup_import_paths

setup_import_paths()

from sharpa_hand import (  # noqa: E402
    ANGLE_RANGES_DEG,
    JOINT_NAME_TO_INDEX,
    SharpaHand,
)

from hardware_interfaces.sharpa_follower.conventions import (  # noqa: E402
    SHARPA_FOLLOWER_SDK_JOINTS,
    SHARPA_URDF_TO_SDK_JOINT,
    urdf_q_to_sdk_targets,
)


class SharpaFollowerSession:
    """Drive the physical Sharpa hand from retargeted Sharpa URDF joint angles."""

    def __init__(self, config: DictConfig, *, verbose: bool = False) -> None:
        self.config = config
        self.verbose = verbose
        self.sharpa_hand: SharpaHand | None = None
        self._q_index: dict[str, int] | None = None
        self._started = False

    @property
    def is_connected(self) -> bool:
        return self._started

    def start(self, q_index_of: Callable[[str], int]) -> None:
        """Connect, enable index/thumb joints, and start position streaming.

        ``q_index_of`` maps a URDF joint name to its index in the retargeted
        ``sharpa_q`` (e.g. ``SharpaFingerIK.joint_q_index``).
        """
        self._apply_angle_overrides()
        hand = SharpaHand.from_config(self.config.sharpa, verbose=self.verbose)
        hand.set_enabled_joints(list(SHARPA_FOLLOWER_SDK_JOINTS))
        hand.connect()
        hand.configure()
        hand.start()
        self.sharpa_hand = hand
        self._q_index = {name: q_index_of(name) for name in SHARPA_URDF_TO_SDK_JOINT}
        self._started = True
        if self.verbose:
            print(f"  Sharpa follower joints: {list(SHARPA_FOLLOWER_SDK_JOINTS)}")

    def _apply_angle_overrides(self) -> None:
        """Widen the SDK position-clamp limits in place from config (deg).

        Mutates the shared ``ANGLE_RANGES_DEG`` table that the SDK consults when
        clamping ``send_positions`` targets, so we can extend ROM (e.g. thumb)
        without editing the sharpa_controller submodule.
        """
        overrides = self.config.sharpa.get("angle_overrides_deg")
        if not overrides:
            return
        for name, rng in overrides.items():
            if name not in JOINT_NAME_TO_INDEX:
                raise ValueError(
                    f"Unknown Sharpa joint in angle_overrides_deg: {name!r}"
                )
            lo, hi = float(rng[0]), float(rng[1])
            ANGLE_RANGES_DEG[JOINT_NAME_TO_INDEX[name]] = (lo, hi)
            if self.verbose:
                print(f"  Sharpa ROM override {name}: [{lo:.0f}, {hi:.0f}] deg")

    def send_q(self, sharpa_q: np.ndarray) -> None:
        """Send a retargeted Sharpa URDF configuration to the hand (no-op if idle)."""
        if not self._started or self.sharpa_hand is None or self._q_index is None:
            return
        targets = urdf_q_to_sdk_targets(sharpa_q, self._q_index.__getitem__)
        self.sharpa_hand.send_positions(targets)

    def stop(self) -> None:
        """Stop streaming and disconnect (idempotent)."""
        if self.sharpa_hand is not None:
            self.sharpa_hand.disconnect()
            self.sharpa_hand = None
        self._started = False
