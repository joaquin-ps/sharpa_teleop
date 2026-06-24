"""Sharpa Wave follower hardware (retargeted joint commands).

Only ``conventions`` (SDK-free) is exported here. Import the session explicitly
via ``hardware_interfaces.sharpa_follower.session`` when Sharpa hardware is
enabled, since it pulls in the Sharpa Wave SDK.
"""

from .conventions import (
    SHARPA_FOLLOWER_SDK_JOINTS,
    SHARPA_URDF_TO_SDK_JOINT,
    urdf_q_to_sdk_targets,
)

__all__ = [
    "SHARPA_FOLLOWER_SDK_JOINTS",
    "SHARPA_URDF_TO_SDK_JOINT",
    "urdf_q_to_sdk_targets",
]
