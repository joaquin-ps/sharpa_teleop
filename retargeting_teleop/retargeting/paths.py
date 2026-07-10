"""Paths to bundled URDF assets for retargeting and visualization."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

import numpy as np

RETARGETING_DIR = Path(__file__).resolve().parent
ASSETS_DIR = RETARGETING_DIR / "assets"

DITTO_2F_LEADER_DIR = ASSETS_DIR / "ditto_2f_leader"
DITTO_3F_LEADER_DIR = ASSETS_DIR / "ditto_3f_leader"
SHARPA_RIGHT_DIR = ASSETS_DIR / "sharpa_right"

DITTO_2F_LEADER_URDF = DITTO_2F_LEADER_DIR / "ditto_2f_leader.urdf"
DITTO_3F_LEADER_URDF = DITTO_3F_LEADER_DIR / "ditto_3f_leader.urdf"
SHARPA_RIGHT_URDF = SHARPA_RIGHT_DIR / "right_sharpa_wave.urdf"

# Back-compat aliases (prefer DITTO_2F_LEADER_URDF / DITTO_3F_LEADER_URDF).
DITTO_LEADER_URDF = DITTO_2F_LEADER_URDF
DITTO_LEADER_DIR = DITTO_2F_LEADER_DIR

# Actuated joints on the Ditto leader (matches ditto motor mapping).
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

# Hardware→URDF sign correction: see hardware_interfaces/ditto_leader/conventions.py

# Dynamixel IDs on the physical Ditto leader (same order as DITTO_LEADER_JOINT_NAMES).
DITTO_LEADER_MOTOR_IDS: tuple[int, ...] = (121, 122, 123, 111, 112, 113, 114)

# Default Hydra hand_config for leader-only visualization (retargeting_teleop/conf/).
DITTO_LEADER_ONLY_HAND_CONFIG = "ditto_2f_leader_only"
DITTO_3F_LEADER_ONLY_HAND_CONFIG = "ditto_3f_leader_only"

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


def ditto_leader_urdf(*, three_finger: bool = False, kinematics_v2: bool | None = None) -> Path:
    """Bundled Ditto leader URDF.

    ``three_finger=True`` (or deprecated ``kinematics_v2=True``) selects the
    canonical 3f ``ditto_3f_leader`` used for hardware IK, Jᵀ, and retargeting.
    Otherwise returns the 2f ``ditto_2f_leader`` (viewer / index+thumb only).
    """
    if kinematics_v2 is not None:
        three_finger = bool(kinematics_v2) or three_finger
    if three_finger:
        return DITTO_3F_LEADER_URDF
    return DITTO_2F_LEADER_URDF


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

# Middle finger: Sharpa URDF has MCP_FE/AA, PIP, and DIP; Ditto leader drives MCP+PIP only.
SHARPA_MIDDLE_JOINT_NAMES: tuple[str, ...] = (
    "right_middle_MCP_FE",
    "right_middle_MCP_AA",
    "right_middle_PIP",
    "right_middle_DIP",
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

SHARPA_MIDDLE_MIRROR_JOINTS: tuple[str, ...] = SHARPA_MIDDLE_JOINT_NAMES

# Sharpa joints not driven by sliders (ring / pinky follow middle).
SHARPA_LOCKED_JOINT_PREFIXES: tuple[str, ...] = (
    "right_ring_",
    "right_pinky_",
)

# Suffix keys for ring/pinky mirror offsets (radians subtracted from middle angle).
RING_PINKY_MIRROR_SUFFIXES: tuple[str, ...] = ("MCP_FE", "MCP_AA", "PIP", "DIP")

# finger name -> {joint suffix -> offset rad}
RingPinkyMirrorOffsets = dict[str, dict[str, float]]


def _empty_ring_pinky_mirror_offsets() -> RingPinkyMirrorOffsets:
    return {finger: {} for finger in SHARPA_MIDDLE_COUPLED_FINGERS}


def _coerce_mapping(value: Any) -> dict[str, Any]:
    """Convert Hydra/OmegaConf nodes to a plain dict for nested config parsing."""
    if value is None:
        return {}
    if isinstance(value, dict):
        return value
    try:
        from omegaconf import OmegaConf

        if OmegaConf.is_config(value):
            container = OmegaConf.to_container(value, resolve=True)
            if isinstance(container, dict):
                return container
    except ImportError:
        pass
    if isinstance(value, Mapping):
        return dict(value)
    return {}


def parse_ring_pinky_mirror_offsets(raw: Any) -> RingPinkyMirrorOffsets:
    """Parse ``sharpa.ring_pinky_mirror_offset_rad`` from config.

    Nested form (independent per finger)::

        ring:  {PIP: 0.08, ...}
        pinky: {PIP: 0.12, ...}

    Legacy flat form (same offset on ring and pinky)::

        PIP: 0.10
    """
    data = _coerce_mapping(raw)
    if not data:
        return _empty_ring_pinky_mirror_offsets()
    if any(k in data for k in RING_PINKY_MIRROR_SUFFIXES):
        shared = {k: float(data[k]) for k in RING_PINKY_MIRROR_SUFFIXES if k in data}
        return {"ring": dict(shared), "pinky": dict(shared)}
    out = _empty_ring_pinky_mirror_offsets()
    for finger in SHARPA_MIDDLE_COUPLED_FINGERS:
        finger_raw = _coerce_mapping(data.get(finger))
        if not finger_raw:
            continue
        out[finger] = {
            suffix: float(finger_raw[suffix])
            for suffix in RING_PINKY_MIRROR_SUFFIXES
            if suffix in finger_raw
        }
    return out


def mirror_offset_for_finger(
    offsets: RingPinkyMirrorOffsets | None,
    finger: str,
    joint_suffix: str,
) -> float:
    if not offsets:
        return 0.0
    finger_offsets = offsets.get(finger) or {}
    return float(finger_offsets.get(joint_suffix, 0.0))


def ring_pinky_mirrored_angle(
    middle_angle: float,
    joint_suffix: str,
    finger: str,
    offsets: RingPinkyMirrorOffsets | None = None,
) -> float:
    """Copy middle angle to one coupled finger, minus its per-joint offset (rad).

    Positive offset on flexion joints (MCP_FE, PIP, DIP) reduces copied flexion
    so that finger lags behind middle. Leave MCP_AA at 0 unless spread should differ.
    """
    return middle_angle - mirror_offset_for_finger(offsets, finger, joint_suffix)


def mirror_sharpa_middle_to_ring_pinky(
    q: np.ndarray,
    joint_q_index: Callable[[str], int],
    offsets: RingPinkyMirrorOffsets | None = None,
) -> None:
    """Copy middle-finger joint angles onto ring and pinky (in-place)."""
    for middle_joint in SHARPA_MIDDLE_MIRROR_JOINTS:
        suffix = middle_joint.removeprefix("right_middle_")
        middle_angle = float(q[joint_q_index(middle_joint)])
        for finger in SHARPA_MIDDLE_COUPLED_FINGERS:
            q[joint_q_index(f"right_{finger}_{suffix}")] = ring_pinky_mirrored_angle(
                middle_angle, suffix, finger, offsets
            )


def mirror_sharpa_middle_to_ring_pinky_cfg(
    cfg: np.ndarray,
    joint_names: list[str],
    offsets: RingPinkyMirrorOffsets | None = None,
) -> None:
    """Viser configuration vector variant of :func:`mirror_sharpa_middle_to_ring_pinky`."""
    index = {name: i for i, name in enumerate(joint_names)}
    for middle_joint in SHARPA_MIDDLE_MIRROR_JOINTS:
        if middle_joint not in index:
            continue
        suffix = middle_joint.removeprefix("right_middle_")
        middle_angle = float(cfg[index[middle_joint]])
        for finger in SHARPA_MIDDLE_COUPLED_FINGERS:
            coupled = f"right_{finger}_{suffix}"
            if coupled in index:
                cfg[index[coupled]] = ring_pinky_mirrored_angle(
                    middle_angle, suffix, finger, offsets
                )

# IK fingertip frames on the Sharpa right hand (per sharpa-urdf-usd-xml README).
SHARPA_RIGHT_FINGERTIP_FRAMES: tuple[str, ...] = (
    "right_thumb_fingertip",
    "right_index_fingertip",
    "right_middle_fingertip",
    "right_ring_fingertip",
    "right_pinky_fingertip",
)
