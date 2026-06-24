"""Paths to bundled URDF assets for retargeting and visualization."""

from __future__ import annotations

from pathlib import Path

RETARGETING_DIR = Path(__file__).resolve().parent
ASSETS_DIR = RETARGETING_DIR / "assets"

DITTO_LEADER_DIR = ASSETS_DIR / "ditto_leader"
SHARPA_RIGHT_DIR = ASSETS_DIR / "sharpa_right"

DITTO_LEADER_URDF = DITTO_LEADER_DIR / "ditto_leader.urdf"
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
DITTO_THUMB_JOINT_NAMES: tuple[str, ...] = DITTO_LEADER_JOINT_NAMES[3:]

# Hardware→URDF sign correction (index joints negated): see hardware_interfaces/ditto_leader/conventions.py

# Dynamixel IDs on the physical Ditto leader (same order as DITTO_LEADER_JOINT_NAMES).
DITTO_LEADER_MOTOR_IDS: tuple[int, ...] = (21, 22, 23, 11, 12, 13, 14)

# Default Hydra hand_config for leader-only visualization (retargeting_teleop/conf/).
DITTO_LEADER_ONLY_HAND_CONFIG = "ditto_7dof_leader_only"

# FK/IK base frame on the Ditto leader: intersection of index_joint_0 and
# index_joint_1 axes at q=0 → (0, 0, 0.046) m in base_link (+Z from index_joint_0).
DITTO_RETARGET_BASE_LINK = "retarget_base"

# IK / viz fingertip link names (Sharpa uses dedicated *_fingertip links).
DITTO_FINGERTIP_LINKS: tuple[str, ...] = (
    "index_fingerpad",
    "thumb_fingerpad",
)

# All Ditto leader frames drawn in the dev viewer.
DITTO_VIZ_FRAME_LINKS: tuple[str, ...] = (
    DITTO_RETARGET_BASE_LINK,
    *DITTO_FINGERTIP_LINKS,
)

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

SHARPA_INDEX_THUMB_JOINT_NAMES: tuple[str, ...] = (
    *SHARPA_INDEX_JOINT_NAMES,
    *SHARPA_THUMB_JOINT_NAMES,
)

SHARPA_INDEX_THUMB_FINGERTIP_LINKS: tuple[str, ...] = (
    "right_index_fingertip",
    "right_thumb_fingertip",
)

# Ditto-matched pad frames: +X = Sharpa fingertip +Z, +Z = Sharpa fingertip +X.
SHARPA_RETARGETING_PAD_LINKS: tuple[str, ...] = (
    "right_index_retargeting_pad",
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

# Sharpa joints held fixed at zero (middle / ring / pinky stretched out).
SHARPA_LOCKED_JOINT_PREFIXES: tuple[str, ...] = (
    "right_middle_",
    "right_ring_",
    "right_pinky_",
)

# IK fingertip frames on the Sharpa right hand (per sharpa-urdf-usd-xml README).
SHARPA_RIGHT_FINGERTIP_FRAMES: tuple[str, ...] = (
    "right_thumb_fingertip",
    "right_index_fingertip",
    "right_middle_fingertip",
    "right_ring_fingertip",
    "right_pinky_fingertip",
)
