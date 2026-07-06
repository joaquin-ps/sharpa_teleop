#!/usr/bin/env python3
"""Load bundled Ditto leader + Sharpa URDF assets in Viser with joint sliders.

Dev-only visualization. For live Ditto leader hardware use ``view_teleop.py``.

Usage (from sharpa_teleop repo root):
    python retargeting_teleop/viz/view_assets.py
    python retargeting_teleop/viz/view_assets.py --sharpa-only
    python retargeting_teleop/viz/view_assets.py --leader-only
    python retargeting_teleop/viz/view_assets.py --3f   # 3-finger Ditto leader URDF

From retargeting_teleop/ package dir:
    python viz/view_assets.py
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Callable

_PKG = Path(__file__).resolve().parent.parent
_REPO = _PKG.parent
for _path in (_PKG, _REPO):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import numpy as np
import pinocchio as pin
import viser
import viser.transforms as tf
from viser.extras import ViserUrdf
from viser.extras._urdf import _viser_name_from_frame

from retargeting.ditto_ik import DittoFingerIK, FingerName
from retargeting.ik_utils import IK_INTERACTIVE, IK_POLISH, IK_STREAM, IkSolveParams
from retargeting.retargeter import DittoToSharpaRetargeter, RetargetResult
from retargeting.sharpa_ik import SharpaFingerIK
from retargeting.paths import (
    DITTO_INDEX_JOINT_NAMES,
    DITTO_MIDDLE_JOINT_NAMES,
    DITTO_THUMB_JOINT_NAMES,
    SHARPA_3F_RETARGET_JOINT_NAMES,
    SHARPA_RIGHT_URDF,
    SHARPA_VIZ_FRAME_LINKS,
    SHARPA_LOCKED_JOINT_PREFIXES,
    SHARPA_SLIDER_JOINT_PREFIXES,
    mirror_sharpa_middle_to_ring_pinky_cfg,
    ditto_leader_urdf,
    ditto_viz_frame_links,
)
from teleop.engine import RetargetTeleopEngine
from teleop.force_sources import make_force_source
from hardware_interfaces.ditto_leader import LeaderHardwareSession

if TYPE_CHECKING:
    from hardware_interfaces.sharpa_follower.session import SharpaFollowerSession
    from teleop.force_render import RetargetForceRenderTeleop

LINK_FRAME_STYLES: dict[str, dict] = {
    "retarget_base": {"origin_color": (80, 255, 80), "axes_length": 0.025},
    "index": {"origin_color": (0, 180, 255), "axes_length": 0.018},
    "middle": {"origin_color": (180, 100, 255), "axes_length": 0.018},
    "thumb": {"origin_color": (255, 140, 0), "axes_length": 0.018},
}

# Pale ghost triads: IK target pad pose on Sharpa (parent = /sharpa_right mount).
SHARPA_RETARGET_TARGET_STYLES: dict[str, dict] = {
    "index": {
        "origin_color": (170, 230, 255),
        "axes_length": 0.015,
        "axes_radius": 0.0012,
        "origin_radius": 0.003,
    },
    "middle": {
        "origin_color": (210, 170, 255),
        "axes_length": 0.015,
        "axes_radius": 0.0012,
        "origin_radius": 0.003,
    },
    "thumb": {
        "origin_color": (255, 215, 150),
        "axes_length": 0.015,
        "axes_radius": 0.0012,
        "origin_radius": 0.003,
    },
}

# Estimated task-space force arrows on the Sharpa pads (sanity check).
FORCE_ARROW_COLORS: dict[str, tuple[int, int, int]] = {
    "index": (255, 60, 60),
    "middle": (180, 80, 255),
    "thumb": (255, 0, 200),
}
# Meters of arrow length per Newton of estimated force.
FORCE_VIZ_DEFAULT_SCALE = 0.03

# Would-be leader joint-torque arrows (drawn along each joint's rotation axis).
JOINT_TORQUE_ARROW_COLOR = (80, 220, 80)
# Meters of arrow length per N·m of joint torque.
JOINT_TORQUE_VIZ_DEFAULT_SCALE = 0.5
# Slider range for the per-joint torque readout (N·m).
LEADER_TORQUE_SLIDER_RANGE = 0.5
# Throttle for the (relatively heavy) force/torque viz update.
FORCE_VIZ_UPDATE_HZ = 25.0

# Fixed Ditto leader mount pose in the viewer (base_link frame relative to world).
DITTO_DEFAULT_MOUNT_POSITION = (-0.046, -0.12, 0.1)
DITTO_DEFAULT_MOUNT_RPY_DEG = (90.0, 0.0, 90.0)

# Sharpa mount:
SHARPA_DEFAULT_MOUNT_POSITION = (0.0, 0.12, 0.0)

VIZ_IDLE_POLL_S = 1.0
VIZ_HARDWARE_POLL_HZ = 60.0
# Decoupled rate for the expensive retarget IK under live hardware, so it does
# not starve the 200 Hz finger_aloha read thread (shared GIL).
HW_RETARGET_HZ = 40.0


def _style_for_link(link_name: str) -> dict:
    if link_name == "retarget_base":
        return LINK_FRAME_STYLES["retarget_base"]
    if "thumb" in link_name:
        return LINK_FRAME_STYLES["thumb"]
    if "middle" in link_name:
        return LINK_FRAME_STYLES["middle"]
    return LINK_FRAME_STYLES["index"]


@dataclass
class RobotViz:
    name: str
    viser_urdf: ViserUrdf
    root_name: str
    mount_frame: viser.FrameHandle | None = None
    slider_handles: list[viser.GuiInputHandle[float]] = field(default_factory=list)
    slider_joint_names: list[str] = field(default_factory=list)
    initial_slider_values: list[float] = field(default_factory=list)
    locked_joint_values: dict[str, float] = field(default_factory=dict)
    mirror_middle_to_ring_pinky: bool = False
    ik_gizmos: dict[str, viser.TransformControlsHandle] = field(default_factory=dict)
    ik_dragging: dict[str, bool] = field(default_factory=dict)
    suppress_slider_callbacks: bool = False


@dataclass
class SharpaRetargetTargetViz:
    frames: dict[str, viser.FrameHandle]


def _viser_cfg_to_pin_q(hand_ik: DittoFingerIK | SharpaFingerIK, cfg: np.ndarray, joint_names: list[str]) -> np.ndarray:
    q = np.zeros(hand_ik.model.nq, dtype=float)
    for name, value in zip(joint_names, cfg):
        q[hand_ik.joint_q_index(name)] = float(value)
    return q


def _pin_q_to_viser_cfg(hand_ik: DittoFingerIK | SharpaFingerIK, q: np.ndarray, joint_names: list[str]) -> np.ndarray:
    return np.array([q[hand_ik.joint_q_index(name)] for name in joint_names], dtype=float)


def _pin_se3_from_viser(
    position: tuple[float, float, float] | np.ndarray,
    wxyz: tuple[float, float, float, float] | np.ndarray,
) -> pin.SE3:
    rotation = np.array(tf.SO3(np.asarray(wxyz)).as_matrix())
    return pin.SE3(rotation, np.asarray(position, dtype=float))


def _viser_pose_from_pin_se3(se3: pin.SE3) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    position = tuple(float(x) for x in se3.translation)
    wxyz = tuple(float(x) for x in tf.SO3.from_matrix(se3.rotation).wxyz)
    return position, wxyz


def _any_ik_dragging(robot: RobotViz) -> bool:
    return any(robot.ik_dragging.values())


def _set_configuration_from_pin_q(
    robot: RobotViz,
    hand_ik: DittoFingerIK | SharpaFingerIK,
    q: np.ndarray,
) -> None:
    joint_names = list(robot.viser_urdf.get_actuated_joint_names())
    cfg = _pin_q_to_viser_cfg(hand_ik, q, joint_names)
    robot.suppress_slider_callbacks = True
    try:
        robot.viser_urdf.update_cfg(cfg)
        for joint_name, slider in zip(robot.slider_joint_names, robot.slider_handles):
            slider.value = float(cfg[joint_names.index(joint_name)])
    finally:
        robot.suppress_slider_callbacks = False


def _sync_ik_gizmos(robot: RobotViz, ik: DittoFingerIK) -> None:
    joint_names = list(robot.viser_urdf.get_actuated_joint_names())
    q = _viser_cfg_to_pin_q(ik, _build_full_configuration(robot), joint_names)
    for finger, gizmo in robot.ik_gizmos.items():
        pose = ik.pad_pose_in_base(q, finger)  # type: ignore[arg-type]
        position, wxyz = _viser_pose_from_pin_se3(pose)
        gizmo.position = position
        gizmo.wxyz = wxyz


def _apply_ik_from_gizmo(
    robot: RobotViz,
    ik: DittoFingerIK,
    finger: FingerName,
    gizmo: viser.TransformControlsHandle,
    *,
    solve_params: IkSolveParams | None = None,
) -> None:
    joint_names = list(robot.viser_urdf.get_actuated_joint_names())
    q_seed = _viser_cfg_to_pin_q(ik, _build_full_configuration(robot), joint_names)
    target = _pin_se3_from_viser(gizmo.position, gizmo.wxyz)
    q, _residual = ik.solve_finger_pad(
        finger,
        target,
        q_seed,
        solve_params=solve_params,
    )
    _set_configuration_from_pin_q(robot, ik, q)


def _add_ditto_ik_gizmos(
    server: viser.ViserServer,
    robot: RobotViz,
    ik: DittoFingerIK,
    *,
    fingers: tuple[FingerName, ...] = ("index", "thumb"),
    on_gizmo_change: Callable[[IkSolveParams], None] | None = None,
) -> None:
    """World-space drag targets parented under the Ditto mount (= base_link)."""
    gizmo_styles = {
        "index": {"scale": 0.018, "opacity": 0.9},
        "middle": {"scale": 0.018, "opacity": 0.9},
        "thumb": {"scale": 0.018, "opacity": 0.9},
    }
    for finger in fingers:
        style = gizmo_styles[finger]
        gizmo = server.scene.add_transform_controls(
            f"{robot.root_name}/ik_gizmo/{finger}",
            scale=style["scale"],
            depth_test=False,
            opacity=style["opacity"],
            visible=True,
        )
        robot.ik_gizmos[finger] = gizmo
        robot.ik_dragging[finger] = False

        def _make_drag_handler(
            finger_name: FingerName,
            gizmo_handle: viser.TransformControlsHandle,
        ):
            @gizmo_handle.on_update
            def _on_gizmo_update(event: viser.TransformControlsEvent) -> None:
                if event.phase == "start":
                    robot.ik_dragging[finger_name] = True
                    return
                if event.phase == "end":
                    robot.ik_dragging[finger_name] = False
                    _apply_ik_from_gizmo(
                        robot,
                        ik,
                        finger_name,
                        gizmo_handle,
                        solve_params=IK_POLISH,
                    )
                    if on_gizmo_change is not None:
                        on_gizmo_change(IK_POLISH)
                    return
                _apply_ik_from_gizmo(
                    robot,
                    ik,
                    finger_name,
                    gizmo_handle,
                    solve_params=IK_INTERACTIVE,
                )
                if on_gizmo_change is not None:
                    on_gizmo_change(IK_INTERACTIVE)

            return _on_gizmo_update

        _make_drag_handler(finger, gizmo)  # type: ignore[arg-type]

    _sync_ik_gizmos(robot, ik)


def _joint_allowed(joint_name: str, prefixes: tuple[str, ...]) -> bool:
    return any(joint_name.startswith(prefix) for prefix in prefixes)


def _build_full_configuration(robot: RobotViz) -> np.ndarray:
    joint_names = list(robot.viser_urdf.get_actuated_joint_names())
    q = np.zeros(len(joint_names), dtype=float)
    for name, value in robot.locked_joint_values.items():
        q[joint_names.index(name)] = value
    for joint_name, slider in zip(robot.slider_joint_names, robot.slider_handles):
        q[joint_names.index(joint_name)] = float(slider.value)
    if robot.mirror_middle_to_ring_pinky:
        mirror_sharpa_middle_to_ring_pinky_cfg(q, joint_names)
    return q


def _apply_configuration(robot: RobotViz) -> None:
    robot.viser_urdf.update_cfg(_build_full_configuration(robot))


def _visual_prefix(root_name: str) -> str:
    return f"{root_name}/visual"


def _add_link_frames(
    server: viser.ViserServer,
    robot: RobotViz,
    frame_links: tuple[str, ...],
) -> None:
    """Attach axis triads as children of ViserUrdf link frames."""
    urdf = robot.viser_urdf._urdf
    prefix = _visual_prefix(robot.root_name)

    for link_name in frame_links:
        style = _style_for_link(link_name)
        parent_path = _viser_name_from_frame(urdf.scene, link_name, prefix)
        server.scene.add_frame(
            f"{parent_path}/viz_axes",
            show_axes=True,
            axes_length=style["axes_length"],
            axes_radius=0.0015,
            origin_radius=0.004,
            origin_color=style["origin_color"],
        )


def _add_sharpa_retarget_target_frames(
    server: viser.ViserServer,
    sharpa_robot: RobotViz,
    *,
    fingers: tuple[FingerName, ...] = ("index", "thumb"),
) -> SharpaRetargetTargetViz:
    """Ghost pad frames in Sharpa base_link showing the IK target pose."""
    frames: dict[str, viser.FrameHandle] = {}
    for finger in fingers:
        style = SHARPA_RETARGET_TARGET_STYLES[finger]
        frames[finger] = server.scene.add_frame(
            f"{sharpa_robot.root_name}/retarget_target/{finger}",
            show_axes=True,
            axes_length=style["axes_length"],
            axes_radius=style["axes_radius"],
            origin_radius=style["origin_radius"],
            origin_color=style["origin_color"],
        )
    return SharpaRetargetTargetViz(frames=frames)


def _sync_sharpa_retarget_target_frames(
    target_viz: SharpaRetargetTargetViz,
    result: RetargetResult,
) -> None:
    targets = {
        "index": result.index_target_in_sharpa_base,
        "middle": result.middle_target_in_sharpa_base,
        "thumb": result.thumb_target_in_sharpa_base,
    }
    for finger, handle in target_viz.frames.items():
        position, wxyz = _viser_pose_from_pin_se3(targets[finger])
        handle.position = position
        handle.wxyz = wxyz


def _update_force_arrows(
    server: viser.ViserServer,
    robot: RobotViz,
    forces_in_base: dict[str, tuple[np.ndarray, np.ndarray]],
    scale: float,
    *,
    name: str = "force_estimate",
) -> None:
    """Draw pad force arrows in a robot's base frame (child of its mount).

    Iterates over whatever fingers are present this tick: a tactile read can be
    missing for a finger on any given frame, so we never assume both exist.
    """
    points = []
    colors = []
    for finger, (origin, force) in forces_in_base.items():
        points.append([origin, origin + scale * force])
        colors.append(FORCE_ARROW_COLORS.get(finger, (255, 255, 255)))
    if not points:
        return
    server.scene.add_arrows(
        f"{robot.root_name}/{name}",
        points=np.asarray(points, dtype=float),
        colors=np.asarray(colors, dtype=np.uint8),
        shaft_radius=0.0015,
        head_radius=0.004,
        head_length=0.008,
    )


def _update_joint_torque_arrows(
    server: viser.ViserServer,
    robot: RobotViz,
    joint_torques: list[tuple[np.ndarray, np.ndarray, float]],
    scale: float,
    *,
    name: str = "leader_joint_torque",
) -> None:
    """Draw torque arrows along each joint axis (moment = torque·axis), base frame."""
    points = []
    for origin, axis, torque in joint_torques:
        points.append([origin, origin + scale * torque * axis])
    if not points:
        return
    server.scene.add_arrows(
        f"{robot.root_name}/{name}",
        points=np.asarray(points, dtype=float),
        colors=np.asarray(
            [JOINT_TORQUE_ARROW_COLOR] * len(points), dtype=np.uint8
        ),
        shaft_radius=0.0012,
        head_radius=0.0032,
        head_length=0.006,
    )


def _create_joint_sliders(
    server: viser.ViserServer,
    robot: RobotViz,
    folder_label: str,
    slider_joint_names: list[str],
    *,
    on_before_apply: Callable[[], None] | None = None,
) -> None:
    limits = robot.viser_urdf.get_actuated_joint_limits()

    with server.gui.add_folder(folder_label):
        for joint_name in slider_joint_names:
            lower, upper = limits[joint_name]
            lo = lower if lower is not None else -np.pi
            hi = upper if upper is not None else np.pi
            if lo < -0.1 and hi > 0.1:
                initial = 0.0
            else:
                initial = 0.5 * (lo + hi)

            slider = server.gui.add_slider(
                label=joint_name,
                min=lo,
                max=hi,
                step=1e-3,
                initial_value=initial,
            )
            robot.slider_handles.append(slider)
            robot.slider_joint_names.append(joint_name)
            robot.initial_slider_values.append(initial)

    def _on_slider_update(_: viser.GuiEvent) -> None:
        if robot.suppress_slider_callbacks:
            return
        if on_before_apply is not None:
            on_before_apply()
        _apply_configuration(robot)

    for slider in robot.slider_handles:
        slider.on_update(_on_slider_update)


def _slider_joint_names_for_robot(
    viser_urdf: ViserUrdf,
    *,
    sharpa_index_thumb_only: bool,
    sharpa_3f_retarget: bool = False,
) -> tuple[list[str], dict[str, float]]:
    all_names = list(viser_urdf.get_actuated_joint_names())
    if sharpa_3f_retarget:
        slider_names = [
            name for name in SHARPA_3F_RETARGET_JOINT_NAMES if name in all_names
        ]
        locked_values = {
            name: 0.0
            for name in all_names
            if name not in slider_names
        }
        return slider_names, locked_values
    if not sharpa_index_thumb_only:
        return all_names, {}

    slider_names = [
        name
        for name in all_names
        if _joint_allowed(name, SHARPA_SLIDER_JOINT_PREFIXES)
    ]
    locked_values = {
        name: 0.0
        for name in all_names
        if _joint_allowed(name, SHARPA_LOCKED_JOINT_PREFIXES)
        or name.startswith("right_middle_")
    }
    return slider_names, locked_values


def _add_robot(
    server: viser.ViserServer,
    urdf_path: Path,
    name: str,
    root_name: str,
    position: tuple[float, float, float],
    slider_folder: str,
    frame_links: tuple[str, ...],
    *,
    sharpa_index_thumb_only: bool = False,
    sharpa_3f_retarget: bool = False,
    on_before_slider_apply: Callable[[], None] | None = None,
    mount_rpy_deg: tuple[float, float, float] | None = None,
) -> RobotViz:
    mount_frame = server.scene.add_frame(root_name, show_axes=False, position=position)
    if mount_rpy_deg is not None:
        mount_frame.wxyz = tf.SO3.from_rpy_radians(
            np.deg2rad(mount_rpy_deg[0]),
            np.deg2rad(mount_rpy_deg[1]),
            np.deg2rad(mount_rpy_deg[2]),
        ).wxyz

    viser_urdf = ViserUrdf(
        server,
        urdf_or_path=urdf_path,
        root_node_name=root_name,
        load_meshes=True,
        load_collision_meshes=False,
    )

    slider_joint_names, locked_values = _slider_joint_names_for_robot(
        viser_urdf,
        sharpa_index_thumb_only=sharpa_index_thumb_only,
        sharpa_3f_retarget=sharpa_3f_retarget,
    )

    robot = RobotViz(
        name=name,
        viser_urdf=viser_urdf,
        root_name=root_name,
        mount_frame=mount_frame,
        locked_joint_values=locked_values,
        mirror_middle_to_ring_pinky=sharpa_index_thumb_only or sharpa_3f_retarget,
    )
    _create_joint_sliders(
        server,
        robot,
        slider_folder,
        slider_joint_names,
        on_before_apply=on_before_slider_apply,
    )
    _apply_configuration(robot)
    _add_link_frames(server, robot, frame_links)
    return robot


def run_viewer(
    *,
    show_leader: bool = True,
    show_sharpa: bool = True,
    ditto_3f: bool = False,
    hardware: LeaderHardwareSession | None = None,
    sharpa_follower: "SharpaFollowerSession | None" = None,
    force_mode: str = "estimate",
    tactile_calibrate: bool = False,
    tactile_debug: bool = False,
    sharpa_cfg=None,
    force_render_controller: "RetargetForceRenderTeleop | None" = None,
) -> None:

    leader_urdf = ditto_leader_urdf(three_finger=ditto_3f)
    leader_frame_links = ditto_viz_frame_links(three_finger=ditto_3f)
    retarget_fingers: tuple[FingerName, ...] = (
        ("index", "middle", "thumb") if ditto_3f else ("index", "thumb")
    )

    for label, path in (
        ("Ditto leader", leader_urdf),
        ("Sharpa right hand", SHARPA_RIGHT_URDF),
    ):
        if not path.is_file():
            raise FileNotFoundError(f"Missing {label} URDF: {path}")

    server = viser.ViserServer()
    server.gui.add_markdown("## Hand Visualizer")
    help_lines = [
        "- Move Ditto / Sharpa with joint sliders or Ditto pad gizmos",
        "- **Live retargeting**: Ditto drives Sharpa index + thumb",
        "\t- Adjust cartesian scale and IK weights in the folders below",
    ]
    if hardware is not None:
        help_lines.append("- **Drive Ditto from leader hardware** for physical encoders")
    if sharpa_follower is not None:
        help_lines.append("- **Send retargeting to Sharpa hardware** to drive the real hand")
        help_lines.append(
            "- **Pad forces** (estimate or tactile) drawn on Sharpa + leader pads; "
            "would-be leader joint torques shown (not sent to hardware)"
        )
    server.gui.add_markdown("\n".join(help_lines))
    server.initial_camera.position = (0.35, -0.45, 0.25)
    server.initial_camera.look_at = (0.05, 0.0, 0.05)

    robots: list[RobotViz] = []

    ditto_robot: RobotViz | None = None
    sharpa_robot: RobotViz | None = None
    ditto_ik: DittoFingerIK | None = None
    sharpa_ik: SharpaFingerIK | None = None
    retargeter: DittoToSharpaRetargeter | None = None
    engine: RetargetTeleopEngine | None = None
    sharpa_retarget_targets: SharpaRetargetTargetViz | None = None
    retarget_error_markdown: viser.GuiMarkdownHandle | None = None
    live_retarget_enabled = {"value": True}
    hardware_drive_enabled = {
        "value": hardware is not None or force_render_controller is not None
    }
    sharpa_send_enabled = {"value": sharpa_follower is not None}
    force_viz_enabled = {"value": sharpa_follower is not None}
    force_viz_scale = {"value": FORCE_VIZ_DEFAULT_SCALE}
    torque_viz_scale = {"value": JOINT_TORQUE_VIZ_DEFAULT_SCALE}
    force_source_state = {"value": force_mode}
    tactile_ready = {"value": False}
    force_status_markdown: viser.GuiMarkdownHandle | None = None
    force_source_dropdown: viser.GuiDropdownHandle | None = None
    leader_torque_sliders: dict[str, viser.GuiSliderHandle] = {}

    def _set_force_source(mode: str) -> bool:
        """Swap the engine's contact-force source; enable tactile lazily.

        Returns True on success. Tactile/mix need the Sharpa tactile sensors; if
        they cannot be enabled we keep the current source and report failure.
        """
        if engine is None:
            return False
        if mode in ("tactile", "mix"):
            if sharpa_follower is None:
                return False
            if not tactile_ready["value"]:
                try:
                    ok = bool(sharpa_follower.enable_tactile(calibrate=tactile_calibrate))
                except Exception as exc:  # noqa: BLE001
                    print(f"  Tactile enable failed: {exc}")
                    ok = False
                tactile_ready["value"] = ok
                if not ok:
                    print(f"  Tactile sensors unavailable; staying on '{force_source_state['value']}'.")
                    return False
        engine.force_source = make_force_source(
            engine.sharpa_ik, engine.fingers, mode, tactile_debug=tactile_debug
        )
        force_source_state["value"] = mode
        return True

    def _retarget_ditto_to_sharpa(
        solve_params: IkSolveParams | None = IK_POLISH,
    ) -> None:
        if (
            not live_retarget_enabled["value"]
            or ditto_robot is None
            or sharpa_robot is None
            or ditto_ik is None
            or sharpa_ik is None
            or engine is None
        ):
            return
        ditto_joint_names = list(ditto_robot.viser_urdf.get_actuated_joint_names())
        ditto_q = _viser_cfg_to_pin_q(
            ditto_ik, _build_full_configuration(ditto_robot), ditto_joint_names
        )
        sharpa_joint_names = list(sharpa_robot.viser_urdf.get_actuated_joint_names())
        sharpa_q_seed = _viser_cfg_to_pin_q(
            sharpa_ik, _build_full_configuration(sharpa_robot), sharpa_joint_names
        )
        # Engine runs the retarget and streams to follower hardware (if enabled).
        result = engine.retarget(ditto_q, sharpa_q_seed, solve_params=solve_params)
        _set_configuration_from_pin_q(sharpa_robot, sharpa_ik, result.sharpa_q)
        if sharpa_retarget_targets is not None:
            _sync_sharpa_retarget_target_frames(sharpa_retarget_targets, result)
        if retarget_error_markdown is not None:
            parts = [
                f"index {result.index_residual:.3f}",
                f"thumb {result.thumb_residual:.3f}",
            ]
            if ditto_3f:
                parts.insert(1, f"middle {result.middle_residual:.3f}")
            retarget_error_markdown.content = (
                "**Retarget IK error (rad):** " + ", ".join(parts)
            )

    def _update_force_estimate(last_log: dict[str, float]) -> None:
        if engine is None or sharpa_robot is None or sharpa_ik is None:
            return
        sharpa_joint_names = list(sharpa_robot.viser_urdf.get_actuated_joint_names())
        q_seed = _viser_cfg_to_pin_q(
            sharpa_ik, _build_full_configuration(sharpa_robot), sharpa_joint_names
        )
        ditto_q = None
        if ditto_robot is not None and ditto_ik is not None:
            ditto_joint_names = list(ditto_robot.viser_urdf.get_actuated_joint_names())
            ditto_q = _viser_cfg_to_pin_q(
                ditto_ik, _build_full_configuration(ditto_robot), ditto_joint_names
            )
        sample = engine.estimate_force_feedback(sharpa_q_seed=q_seed, ditto_q=ditto_q)
        if sample is None:
            return

        _update_force_arrows(
            server, sharpa_robot, sample.sharpa_forces, force_viz_scale["value"]
        )
        if force_status_markdown is not None:
            mags = ", ".join(
                f"{f} {sample.magnitudes[f]:.2f}" for f in sample.magnitudes
            )
            force_status_markdown.content = (
                f"**Pad force (N) [{engine.force_source.name}]:** {mags}"
            )

        if ditto_robot is not None:
            _update_force_arrows(
                server,
                ditto_robot,
                sample.leader_forces,
                force_viz_scale["value"],
                name="leader_force",
            )
            _update_joint_torque_arrows(
                server, ditto_robot, sample.torque_arrows, torque_viz_scale["value"]
            )
            for name, slider in leader_torque_sliders.items():
                slider.value = float(
                    np.clip(
                        sample.leader_torques.get(name, 0.0),
                        -LEADER_TORQUE_SLIDER_RANGE,
                        LEADER_TORQUE_SLIDER_RANGE,
                    )
                )

        now = time.time()
        if now - last_log["t"] >= 0.5:
            last_log["t"] = now
            parts = []
            for finger in sample.sharpa_force_vec:
                fv = sample.sharpa_force_vec[finger]
                parts.append(
                    f"{finger} |F|={sample.magnitudes[finger]:.2f}N "
                    f"({fv[0]:+.2f},{fv[1]:+.2f},{fv[2]:+.2f})"
                )
            print(f"[force:{engine.force_source.name}] " + "  ".join(parts))
            if sample.leader_torques:
                print(
                    "  leader tau(Nm): "
                    + ", ".join(
                        f"{n} {t:+.3f}" for n, t in sample.leader_torques.items()
                    )
                )

    hardware_status_markdown: viser.GuiMarkdownHandle | None = None
    sharpa_status_markdown: viser.GuiMarkdownHandle | None = None
    live_retarget: viser.GuiCheckboxHandle | None = None
    with server.gui.add_folder("Controls"):
        if show_leader and show_sharpa:
            live_retarget = server.gui.add_checkbox(
                "Live Ditto → Sharpa retargeting",
                initial_value=True,
            )

        if sharpa_follower is not None:
            sharpa_send = server.gui.add_checkbox(
                "Send retargeting to Sharpa hardware",
                initial_value=True,
            )

            @sharpa_send.on_update
            def _(_: viser.GuiEvent) -> None:
                sharpa_send_enabled["value"] = bool(sharpa_send.value)
                if engine is not None:
                    engine.sharpa_send_enabled = bool(sharpa_send.value)
                if sharpa_send.value:
                    _retarget_ditto_to_sharpa(IK_POLISH)

            sharpa_status_markdown = server.gui.add_markdown(
                "**Sharpa hardware:** streaming retargeted joints"
            )

            force_viz = server.gui.add_checkbox(
                "Show pad forces + leader torques",
                initial_value=force_viz_enabled["value"],
            )

            @force_viz.on_update
            def _(_: viser.GuiEvent) -> None:
                force_viz_enabled["value"] = bool(force_viz.value)

            force_source_dropdown = server.gui.add_dropdown(
                "Force source",
                options=("estimate", "tactile", "mix"),
                initial_value=force_mode,
            )

            @force_source_dropdown.on_update
            def _(_: viser.GuiEvent) -> None:
                mode = str(force_source_dropdown.value)
                if mode == force_source_state["value"]:
                    return
                if not _set_force_source(mode):
                    # Revert the dropdown if the source could not be activated.
                    force_source_dropdown.value = force_source_state["value"]

            force_scale_slider = server.gui.add_slider(
                "Force arrow scale (m/N)",
                min=0.005,
                max=0.2,
                step=0.005,
                initial_value=force_viz_scale["value"],
            )

            @force_scale_slider.on_update
            def _(_: viser.GuiEvent) -> None:
                force_viz_scale["value"] = float(force_scale_slider.value)

            torque_scale_slider = server.gui.add_slider(
                "Joint torque arrow scale (m/Nm)",
                min=0.05,
                max=3.0,
                step=0.05,
                initial_value=torque_viz_scale["value"],
            )

            @torque_scale_slider.on_update
            def _(_: viser.GuiEvent) -> None:
                torque_viz_scale["value"] = float(torque_scale_slider.value)

            force_status_markdown = server.gui.add_markdown(
                "**Estimated pad force (N):** waiting for torque reads…"
            )

            with server.gui.add_folder("Would-be leader joint torque (Nm)"):
                leader_torque_joint_names = (
                    *DITTO_INDEX_JOINT_NAMES,
                    *(DITTO_MIDDLE_JOINT_NAMES if ditto_3f else ()),
                    *DITTO_THUMB_JOINT_NAMES,
                )
                for _jname in leader_torque_joint_names:
                    leader_torque_sliders[_jname] = server.gui.add_slider(
                        _jname,
                        min=-LEADER_TORQUE_SLIDER_RANGE,
                        max=LEADER_TORQUE_SLIDER_RANGE,
                        step=0.001,
                        initial_value=0.0,
                        disabled=True,
                    )

        if hardware is not None:
            hardware_drive = server.gui.add_checkbox(
                "Drive Ditto from leader hardware",
                initial_value=True,
            )

            @hardware_drive.on_update
            def _(_: viser.GuiEvent) -> None:
                hardware_drive_enabled["value"] = bool(hardware_drive.value)

            hardware_status_markdown = server.gui.add_markdown(
                "**Hardware status:** waiting for encoder reads…"
            )

        reset_button = server.gui.add_button("Reset all joints")

    if live_retarget is not None:

        @live_retarget.on_update
        def _(_: viser.GuiEvent) -> None:
            live_retarget_enabled["value"] = bool(live_retarget.value)
            if live_retarget.value:
                _retarget_ditto_to_sharpa(IK_POLISH)

    @reset_button.on_click
    def _(_: viser.GuiEvent) -> None:
        for robot in robots:
            for slider, q0 in zip(robot.slider_handles, robot.initial_slider_values):
                slider.value = q0
            _apply_configuration(robot)
        if ditto_robot is not None and ditto_ik is not None:
            _sync_ik_gizmos(ditto_robot, ditto_ik)
        _retarget_ditto_to_sharpa(IK_POLISH)

    if show_leader:
        ditto_ik = DittoFingerIK(leader_urdf)

        def _on_ditto_slider_change() -> None:
            if ditto_robot is None or ditto_ik is None:
                return
            if not _any_ik_dragging(ditto_robot):
                _sync_ik_gizmos(ditto_robot, ditto_ik)
            _retarget_ditto_to_sharpa(IK_POLISH)

        def _on_ditto_gizmo_change(solve_params: IkSolveParams) -> None:
            _retarget_ditto_to_sharpa(solve_params)

        ditto_robot = _add_robot(
            server,
            leader_urdf,
            name="Ditto leader" + (" (3f)" if ditto_3f else ""),
            root_name="/ditto_leader",
            position=DITTO_DEFAULT_MOUNT_POSITION,
            slider_folder="Ditto leader joints",
            frame_links=leader_frame_links,
            on_before_slider_apply=_on_ditto_slider_change,
            mount_rpy_deg=DITTO_DEFAULT_MOUNT_RPY_DEG,
        )
        _add_ditto_ik_gizmos(
            server,
            ditto_robot,
            ditto_ik,
            fingers=retarget_fingers,
            on_gizmo_change=_on_ditto_gizmo_change,
        )
        robots.append(ditto_robot)

    if show_sharpa:
        sharpa_ik = SharpaFingerIK(SHARPA_RIGHT_URDF)
        sharpa_robot = _add_robot(
            server,
            SHARPA_RIGHT_URDF,
            name="Sharpa right hand",
            root_name="/sharpa_right",
            position=SHARPA_DEFAULT_MOUNT_POSITION,
            slider_folder=(
                "Sharpa index + middle + thumb joints"
                if ditto_3f
                else "Sharpa index + thumb joints"
            ),
            frame_links=SHARPA_VIZ_FRAME_LINKS,
            sharpa_index_thumb_only=not ditto_3f,
            sharpa_3f_retarget=ditto_3f,
        )
        robots.append(sharpa_robot)

    if show_leader and show_sharpa:
        if force_render_controller is not None:
            engine = force_render_controller.engine
            retargeter = engine.retargeter
            sharpa_follower = engine.sharpa_follower
            sharpa_send_enabled["value"] = sharpa_follower is not None
            force_viz_enabled["value"] = sharpa_follower is not None
            if sharpa_follower is not None and getattr(
                sharpa_follower, "_tactile_ready", False
            ):
                tactile_ready["value"] = True
            print(
                "  Ditto leader: force-render loop running "
                "(retarget + haptics on background thread)"
            )
        else:
            engine = RetargetTeleopEngine(
                hardware=hardware,
                sharpa_follower=sharpa_follower,
                retargeter=DittoToSharpaRetargeter.from_sharpa_config(
                    sharpa_cfg, ditto_urdf=leader_urdf
                ),
                fingers=retarget_fingers,
            )
            retargeter = engine.retargeter
        assert sharpa_robot is not None and sharpa_ik is not None
        if force_render_controller is None and sharpa_follower is not None:
            print("Connecting Sharpa follower hardware...")
            try:
                sharpa_follower.start(sharpa_ik.joint_q_index)
            except Exception as exc:  # noqa: BLE001
                print(f"  Sharpa follower unavailable, continuing without it: {exc}")
                sharpa_follower = None
                engine.sharpa_follower = None
                sharpa_send_enabled["value"] = False
                if sharpa_status_markdown is not None:
                    sharpa_status_markdown.content = "**Sharpa hardware:** unavailable"
            else:
                # Activate the requested initial force source (enables tactile
                # sensors when needed); fall back to estimate if unavailable.
                if force_mode != "estimate" and not _set_force_source(force_mode):
                    if force_source_dropdown is not None:
                        force_source_dropdown.value = "estimate"
        sharpa_retarget_targets = _add_sharpa_retarget_target_frames(
            server, sharpa_robot, fingers=retarget_fingers
        )
        retarget_error_markdown = server.gui.add_markdown(
            "**Retarget IK error (rad):** —"
        )
        with server.gui.add_folder("Retarget cartesian scale"):
            index_scale_slider = server.gui.add_slider(
                "Index scale",
                min=0.25,
                max=3.0,
                step=0.01,
                initial_value=retargeter.index_cartesian_scale,
            )
            thumb_scale_slider = server.gui.add_slider(
                "Thumb scale",
                min=0.25,
                max=3.0,
                step=0.01,
                initial_value=retargeter.thumb_cartesian_scale,
            )
            middle_scale_slider = None
            if ditto_3f:
                middle_scale_slider = server.gui.add_slider(
                    "Middle scale",
                    min=0.25,
                    max=3.0,
                    step=0.01,
                    initial_value=retargeter.middle_cartesian_scale,
                )

            @index_scale_slider.on_update
            def _(_: viser.GuiEvent) -> None:
                assert retargeter is not None and index_scale_slider is not None
                retargeter.index_cartesian_scale = float(index_scale_slider.value)
                _retarget_ditto_to_sharpa(IK_POLISH)

            @thumb_scale_slider.on_update
            def _(_: viser.GuiEvent) -> None:
                assert retargeter is not None and thumb_scale_slider is not None
                retargeter.thumb_cartesian_scale = float(thumb_scale_slider.value)
                _retarget_ditto_to_sharpa(IK_POLISH)

            if middle_scale_slider is not None:

                @middle_scale_slider.on_update
                def _(_: viser.GuiEvent) -> None:
                    assert retargeter is not None
                    retargeter.middle_cartesian_scale = float(middle_scale_slider.value)
                    _retarget_ditto_to_sharpa(IK_POLISH)

        if ditto_3f and retargeter is not None:

            def _sync_ring_pinky_mirror_offsets() -> None:
                assert retargeter is not None
                if sharpa_follower is not None:
                    sharpa_follower._ring_pinky_mirror_offsets = {
                        finger: dict(offsets)
                        for finger, offsets in retargeter.ring_pinky_mirror_offset_rad.items()
                    }

            def _finger_offsets(finger: str) -> dict[str, float]:
                assert retargeter is not None
                return dict(retargeter.ring_pinky_mirror_offset_rad.get(finger) or {})

            with server.gui.add_folder("Ring mirror lag (rad)"):
                ring_off = _finger_offsets("ring")
                ring_mcp_fe = server.gui.add_slider(
                    "MCP flex",
                    min=0.0,
                    max=0.4,
                    step=0.01,
                    initial_value=float(ring_off.get("MCP_FE", 0.0)),
                )
                ring_pip = server.gui.add_slider(
                    "PIP",
                    min=0.0,
                    max=0.4,
                    step=0.01,
                    initial_value=float(ring_off.get("PIP", 0.0)),
                )
                ring_dip = server.gui.add_slider(
                    "DIP",
                    min=0.0,
                    max=0.4,
                    step=0.01,
                    initial_value=float(ring_off.get("DIP", 0.0)),
                )
                ring_aa = server.gui.add_slider(
                    "MCP abduction",
                    min=0.0,
                    max=0.2,
                    step=0.01,
                    initial_value=float(ring_off.get("MCP_AA", 0.0)),
                )

            with server.gui.add_folder("Pinky mirror lag (rad)"):
                pinky_off = _finger_offsets("pinky")
                pinky_mcp_fe = server.gui.add_slider(
                    "MCP flex",
                    min=0.0,
                    max=0.4,
                    step=0.01,
                    initial_value=float(pinky_off.get("MCP_FE", 0.0)),
                )
                pinky_pip = server.gui.add_slider(
                    "PIP",
                    min=0.0,
                    max=0.4,
                    step=0.01,
                    initial_value=float(pinky_off.get("PIP", 0.0)),
                )
                pinky_dip = server.gui.add_slider(
                    "DIP",
                    min=0.0,
                    max=0.4,
                    step=0.01,
                    initial_value=float(pinky_off.get("DIP", 0.0)),
                )
                pinky_aa = server.gui.add_slider(
                    "MCP abduction",
                    min=0.0,
                    max=0.2,
                    step=0.01,
                    initial_value=float(pinky_off.get("MCP_AA", 0.0)),
                )

            def _on_mirror_lag_change(_: viser.GuiEvent) -> None:
                assert retargeter is not None
                retargeter.ring_pinky_mirror_offset_rad = {
                    "ring": {
                        "MCP_FE": float(ring_mcp_fe.value),
                        "MCP_AA": float(ring_aa.value),
                        "PIP": float(ring_pip.value),
                        "DIP": float(ring_dip.value),
                    },
                    "pinky": {
                        "MCP_FE": float(pinky_mcp_fe.value),
                        "MCP_AA": float(pinky_aa.value),
                        "PIP": float(pinky_pip.value),
                        "DIP": float(pinky_dip.value),
                    },
                }
                _sync_ring_pinky_mirror_offsets()
                _retarget_ditto_to_sharpa(IK_POLISH)

            for slider in (
                ring_mcp_fe,
                ring_pip,
                ring_dip,
                ring_aa,
                pinky_mcp_fe,
                pinky_pip,
                pinky_dip,
                pinky_aa,
            ):
                slider.on_update(_on_mirror_lag_change)

        with server.gui.add_folder("Retarget IK weights"):
            index_pos_weight_slider = server.gui.add_slider(
                "Index position weight",
                min=0.0,
                max=5.0,
                step=0.05,
                initial_value=retargeter.index_position_weight,
            )
            index_ori_weight_slider = server.gui.add_slider(
                "Index orientation weight",
                min=0.0,
                max=5.0,
                step=0.05,
                initial_value=retargeter.index_orientation_weight,
            )
            thumb_pos_weight_slider = server.gui.add_slider(
                "Thumb position weight",
                min=0.0,
                max=5.0,
                step=0.05,
                initial_value=retargeter.thumb_position_weight,
            )
            thumb_ori_weight_slider = server.gui.add_slider(
                "Thumb orientation weight",
                min=0.0,
                max=5.0,
                step=0.05,
                initial_value=retargeter.thumb_orientation_weight,
            )
            middle_pos_weight_slider = None
            middle_ori_weight_slider = None
            if ditto_3f:
                middle_pos_weight_slider = server.gui.add_slider(
                    "Middle position weight",
                    min=0.0,
                    max=5.0,
                    step=0.05,
                    initial_value=retargeter.middle_position_weight,
                )
                middle_ori_weight_slider = server.gui.add_slider(
                    "Middle orientation weight",
                    min=0.0,
                    max=5.0,
                    step=0.05,
                    initial_value=retargeter.middle_orientation_weight,
                )

            @index_pos_weight_slider.on_update
            def _(_: viser.GuiEvent) -> None:
                assert retargeter is not None
                retargeter.index_position_weight = float(index_pos_weight_slider.value)
                _retarget_ditto_to_sharpa(IK_POLISH)

            @index_ori_weight_slider.on_update
            def _(_: viser.GuiEvent) -> None:
                assert retargeter is not None
                retargeter.index_orientation_weight = float(index_ori_weight_slider.value)
                _retarget_ditto_to_sharpa(IK_POLISH)

            @thumb_pos_weight_slider.on_update
            def _(_: viser.GuiEvent) -> None:
                assert retargeter is not None
                retargeter.thumb_position_weight = float(thumb_pos_weight_slider.value)
                _retarget_ditto_to_sharpa(IK_POLISH)

            @thumb_ori_weight_slider.on_update
            def _(_: viser.GuiEvent) -> None:
                assert retargeter is not None
                retargeter.thumb_orientation_weight = float(thumb_ori_weight_slider.value)
                _retarget_ditto_to_sharpa(IK_POLISH)

            if middle_pos_weight_slider is not None:

                @middle_pos_weight_slider.on_update
                def _(_: viser.GuiEvent) -> None:
                    assert retargeter is not None
                    retargeter.middle_position_weight = float(
                        middle_pos_weight_slider.value
                    )
                    _retarget_ditto_to_sharpa(IK_POLISH)

            if middle_ori_weight_slider is not None:

                @middle_ori_weight_slider.on_update
                def _(_: viser.GuiEvent) -> None:
                    assert retargeter is not None
                    retargeter.middle_orientation_weight = float(
                        middle_ori_weight_slider.value
                    )
                    _retarget_ditto_to_sharpa(IK_POLISH)

        _retarget_ditto_to_sharpa(IK_POLISH)

    server.scene.add_grid("/grid", width=1.0, height=1.0, position=(0.0, 0.0, 0.0))

    print("Viser asset viewer running.")
    print(f"  Ditto leader URDF: {leader_urdf}")
    print(f"  Sharpa right URDF: {SHARPA_RIGHT_URDF}")
    if hardware is not None:
        port = hardware.config.u2d2.get("usb_port", "/dev/ttyUSB0")
        print(f"  Ditto leader hardware: polling {port} (torque_off, no force feedback)")
        if not hardware.is_receiving:
            print("  Waiting for valid encoder reads (sliders/gizmos work meanwhile).")
    elif force_render_controller is not None:
        print("  Ditto leader hardware: current-mode force rendering (haptics active)")
    if sharpa_follower is not None:
        finger_desc = "index + middle + thumb" if ditto_3f else "index + thumb"
        print(f"  Sharpa follower: streaming retargeted {finger_desc} joints.")
    print("Press Ctrl+C to exit.")

    needs_fast_poll = (
        hardware is not None
        or sharpa_follower is not None
        or force_render_controller is not None
    )
    poll_interval = 1.0 / VIZ_HARDWARE_POLL_HZ if needs_fast_poll else VIZ_IDLE_POLL_S
    last_force_log = {"t": 0.0}
    last_force_viz = {"t": 0.0}
    last_hw_retarget = {"t": 0.0}
    try:
        while True:
            if (
                force_render_controller is not None
                and ditto_robot is not None
                and ditto_ik is not None
            ):
                _set_configuration_from_pin_q(
                    ditto_robot, ditto_ik, force_render_controller.engine.ditto_q
                )
                if sharpa_robot is not None and sharpa_ik is not None:
                    _set_configuration_from_pin_q(
                        sharpa_robot,
                        sharpa_ik,
                        force_render_controller.engine.sharpa_q,
                    )
            if (
                hardware is not None
                and hardware_drive_enabled["value"]
                and ditto_robot is not None
                and ditto_ik is not None
                and not _any_ik_dragging(ditto_robot)
            ):
                angles = hardware.poll_joint_angles()
                if angles is not None:
                    _set_configuration_from_pin_q(ditto_robot, ditto_ik, angles)
                    _sync_ik_gizmos(ditto_robot, ditto_ik)
                    # Retarget IK is expensive; run it decoupled from the poll so
                    # it does not starve the 200 Hz hardware read thread.
                    if time.time() - last_hw_retarget["t"] >= 1.0 / HW_RETARGET_HZ:
                        last_hw_retarget["t"] = time.time()
                        _retarget_ditto_to_sharpa(IK_STREAM)
                    if hardware_status_markdown is not None:
                        hardware_status_markdown.content = (
                            "**Hardware status:** receiving encoder data"
                        )
                elif hardware_status_markdown is not None:
                    hardware_status_markdown.content = (
                        "**Hardware status:** no valid reads yet — check USB / power / port"
                    )

            if (
                sharpa_follower is not None
                and force_viz_enabled["value"]
                and sharpa_robot is not None
                and sharpa_ik is not None
                and time.time() - last_force_viz["t"] >= 1.0 / FORCE_VIZ_UPDATE_HZ
            ):
                last_force_viz["t"] = time.time()
                _update_force_estimate(last_force_log)

            time.sleep(poll_interval)
    except KeyboardInterrupt:
        print("\nShutting down viewer...")
    finally:
        if hardware is not None:
            hardware.stop()
        if sharpa_follower is not None and force_render_controller is None:
            sharpa_follower.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--leader-only",
        action="store_true",
        help="Show only the Ditto leader URDF.",
    )
    parser.add_argument(
        "--sharpa-only",
        action="store_true",
        help="Show only the Sharpa right-hand URDF.",
    )
    parser.add_argument(
        "--3f",
        action="store_true",
        help="Use the 3-finger Ditto leader URDF (index + middle + thumb).",
    )
    args = parser.parse_args()

    if args.leader_only and args.sharpa_only:
        parser.error("Choose at most one of --leader-only / --sharpa-only.")

    run_viewer(
        show_leader=not args.sharpa_only,
        show_sharpa=not args.leader_only,
        ditto_3f=args.__dict__["3f"],
        hardware=None,
    )


if __name__ == "__main__":
    main()
