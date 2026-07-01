"""Paths to bundled URDF assets for retargeting and visualization."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import numpy as np

RETARGETING_DIR = Path(__file__).resolve().parent
ASSETS_DIR = RETARGETING_DIR / "assets"

DITTO_LEADER_DIR = ASSETS_DIR / "ditto_leader"
DITTO_3F_LEADER_DIR = ASSETS_DIR / "ditto_3f_leader"
SHARPA_RIGHT_DIR = ASSETS_DIR / "sharpa_right"

DITTO_LEADER_URDF = DITTO_LEADER_DIR / "ditto_leader.urdf"
DITTO_3F_LEADER_URDF = DITTO_3F_LEADER_DIR / "ditto_3f_leader.urdf"
SHARPA_RIGHT_URDF = SHARPA_RIGHT_DIR / "right_sharpa_wave.urdf"

# Actuated joints on the Ditto leader (matches finger_aloha motor mapping).
DITTO_LEADER_JOINT_NAMES: tuple[str, ...] = (
    "index_joint_0",
    "index_joint_1",
    "index_joint_2",
    "thumb_joint_0",
    "thumb_joint_1",
    "thumb_joint_2",
    "thumb_joint_3",
)

DITTO_INDEX_JOINT_NAMES: tuple[str, ...] = DITTO_LEADER_JOINT_NAMES[:3]
DITTO_MIDDLE_JOINT_NAMES: tuple[str, ...] = (
    "middle_joint_0",
    "middle_joint_1",
    "middle_joint_2",
)
DITTO_THUMB_JOINT_NAMES: tuple[str, ...] = DITTO_LEADER_JOINT_NAMES[3:]

# 10-DoF 3-finger Ditto leader (index + middle + thumb).
# Motor / pinocchio order: index 121-123, middle 131-133, thumb 111-114.
# (Viser URDF joint list may differ; always map by joint name, not array index.)
DITTO_3F_LEADER_JOINT_NAMES: tuple[str, ...] = (
    *DITTO_INDEX_JOINT_NAMES,
    *DITTO_MIDDLE_JOINT_NAMES,
    *DITTO_THUMB_JOINT_NAMES,
)
DITTO_3F_LEADER_MOTOR_IDS: tuple[int, ...] = (
    121, 122, 123, 131, 132, 133, 111, 112, 113, 114,
)

# Hardware→URDF sign correction (index joints negated): see hardware_interfaces/ditto_leader/conventions.py

# Dynamixel IDs on the physical Ditto leader (same order as DITTO_LEADER_JOINT_NAMES).
DITTO_LEADER_MOTOR_IDS: tuple[int, ...] = (121, 122, 123, 111, 112, 113, 114)

# Default Hydra hand_config for leader-only visualization (retargeting_teleop/conf/).
DITTO_LEADER_ONLY_HAND_CONFIG = "ditto_7dof_leader_only"
DITTO_3F_LEADER_ONLY_HAND_CONFIG = "ditto_10dof_leader_only"

# FK/IK base frame on the Ditto leader: intersection of index_joint_0 and
# index_joint_1 axes at q=0 → (0, 0, 0.046) m in base_link (+Z from index_joint_0).
DITTO_RETARGET_BASE_LINK = "retarget_base"

# IK / viz fingertip link names (Sharpa uses dedicated *_fingertip links).
DITTO_FINGERTIP_LINKS: tuple[str, ...] = (
    "index_fingerpad",
    "thumb_fingerpad",
)

DITTO_3F_FINGERTIP_LINKS: tuple[str, ...] = (
    "index_fingerpad",
    "middle_fingerpad",
    "thumb_fingerpad",
)

# All Ditto leader frames drawn in the dev viewer.
DITTO_VIZ_FRAME_LINKS: tuple[str, ...] = (
    DITTO_RETARGET_BASE_LINK,
    *DITTO_FINGERTIP_LINKS,
)

DITTO_3F_VIZ_FRAME_LINKS: tuple[str, ...] = (
    DITTO_RETARGET_BASE_LINK,
    *DITTO_3F_FINGERTIP_LINKS,
)


def ditto_leader_urdf(*, three_finger: bool = False) -> Path:
    """Bundled Ditto leader URDF (2-finger default, 3-finger with ``three_finger=True``)."""
    return DITTO_3F_LEADER_URDF if three_finger else DITTO_LEADER_URDF


def ditto_viz_frame_links(*, three_finger: bool = False) -> tuple[str, ...]:
    return DITTO_3F_VIZ_FRAME_LINKS if three_finger else DITTO_VIZ_FRAME_LINKS

SHARPA_INDEX_JOINT_NAMES: tuple[str, ...] = (
    "right_index_MCP_FE",
    "right_index_MCP_AA",
    "right_index_PIP",
    "right_index_DIP",
)

SHARPA_THUMB_JOINT_NAMES: tuple[str, ...] = (
    "right_thumb_CMC_FE",
    "right_thumb_CMC_AA",
    "right_thumb_MCP_FE",
    "right_thumb_MCP_AA",
    "right_thumb_IP",
)

# Middle finger: 3 actuated joints on hardware (DIP held at zero in retarget viz).
SHARPA_MIDDLE_JOINT_NAMES: tuple[str, ...] = (
    "right_middle_MCP_FE",
    "right_middle_MCP_AA",
    "right_middle_PIP",
)

SHARPA_MIDDLE_DIP_JOINT = "right_middle_DIP"

SHARPA_INDEX_THUMB_JOINT_NAMES: tuple[str, ...] = (
    *SHARPA_INDEX_JOINT_NAMES,
    *SHARPA_THUMB_JOINT_NAMES,
)

SHARPA_3F_RETARGET_JOINT_NAMES: tuple[str, ...] = (
    *SHARPA_INDEX_JOINT_NAMES,
    *SHARPA_MIDDLE_JOINT_NAMES,
    *SHARPA_THUMB_JOINT_NAMES,
)

SHARPA_INDEX_THUMB_FINGERTIP_LINKS: tuple[str, ...] = (
    "right_index_fingertip",
    "right_thumb_fingertip",
)

# Ditto-matched pad frames: +X = Sharpa fingertip +Z, +Z = Sharpa fingertip +X.
SHARPA_RETARGETING_PAD_LINKS: tuple[str, ...] = (
    "right_index_retargeting_pad",
    "right_middle_retargeting_pad",
    "right_thumb_retargeting_pad",
)

# FK/IK base on Sharpa right hand: index MCP_FE ∩ MCP_AA axes at q=0 in
# right_hand_C_MC → (0.001, 0.0303, 0.0959) m. Orientation: palm +X→+Z,
# palm +Z→+Y, palm +Y→+X (rpy π/2, 0, π/2).
SHARPA_RETARGET_BASE_LINK = "retarget_base"

SHARPA_VIZ_FRAME_LINKS: tuple[str, ...] = (
    SHARPA_RETARGET_BASE_LINK,
    *SHARPA_RETARGETING_PAD_LINKS,
)

# Sharpa joints exposed in the dev viewer (index + thumb only).
SHARPA_SLIDER_JOINT_PREFIXES: tuple[str, ...] = (
    "right_index_",
    "right_thumb_",
)

# Ring / pinky have no hardware; viewer mirrors middle joint angles onto them.
SHARPA_MIDDLE_COUPLED_FINGERS: tuple[str, ...] = ("ring", "pinky")

SHARPA_MIDDLE_MIRROR_JOINTS: tuple[str, ...] = (
    *SHARPA_MIDDLE_JOINT_NAMES,
    SHARPA_MIDDLE_DIP_JOINT,
)

# Sharpa joints not driven by sliders (ring / pinky follow middle; middle DIP in 3f IK).
SHARPA_LOCKED_JOINT_PREFIXES: tuple[str, ...] = (
    "right_ring_",
    "right_pinky_",
)


def mirror_sharpa_middle_to_ring_pinky(
    q: np.ndarray,
    joint_q_index: Callable[[str], int],
) -> None:
    """Copy middle-finger joint angles onto ring and pinky (in-place)."""
    for middle_joint in SHARPA_MIDDLE_MIRROR_JOINTS:
        suffix = middle_joint.removeprefix("right_middle_")
        value = float(q[joint_q_index(middle_joint)])
        for finger in SHARPA_MIDDLE_COUPLED_FINGERS:
            q[joint_q_index(f"right_{finger}_{suffix}")] = value


def mirror_sharpa_middle_to_ring_pinky_cfg(
    cfg: np.ndarray,
    joint_names: list[str],
) -> None:
    """Viser configuration vector variant of :func:`mirror_sharpa_middle_to_ring_pinky`."""
    index = {name: i for i, name in enumerate(joint_names)}
    for middle_joint in SHARPA_MIDDLE_MIRROR_JOINTS:
        if middle_joint not in index:
            continue
        suffix = middle_joint.removeprefix("right_middle_")
        value = float(cfg[index[middle_joint]])
        for finger in SHARPA_MIDDLE_COUPLED_FINGERS:
            coupled = f"right_{finger}_{suffix}"
            if coupled in index:
                cfg[index[coupled]] = value

# IK fingertip frames on the Sharpa right hand (per sharpa-urdf-usd-xml README).
SHARPA_RIGHT_FINGERTIP_FRAMES: tuple[str, ...] = (
    "right_thumb_fingertip",
    "right_index_fingertip",
    "right_middle_fingertip",
    "right_ring_fingertip",
    "right_pinky_fingertip",
)
