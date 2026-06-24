#!/usr/bin/env python3
"""Load bundled Ditto leader + Sharpa URDF assets in Viser with joint sliders.

Dev-only visualization. For live Ditto leader hardware use ``view_teleop.py``.

Usage (from sharpa_teleop repo root):
    python retargeting_teleop/viz/view_assets.py
    python retargeting_teleop/viz/view_assets.py --sharpa-only
    python retargeting_teleop/viz/view_assets.py --leader-only

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
from retargeting.ik_utils import IK_INTERACTIVE, IK_POLISH, IkSolveParams
from retargeting.retargeter import DittoToSharpaRetargeter, RetargetResult
from retargeting.sharpa_ik import SharpaFingerIK
from retargeting.paths import (
    DITTO_LEADER_URDF,
    DITTO_VIZ_FRAME_LINKS,
    SHARPA_RIGHT_URDF,
    SHARPA_VIZ_FRAME_LINKS,
    SHARPA_LOCKED_JOINT_PREFIXES,
    SHARPA_SLIDER_JOINT_PREFIXES,
)
from hardware_interfaces.ditto_leader import LeaderHardwareSession

if TYPE_CHECKING:
    from hardware_interfaces.sharpa_follower.session import SharpaFollowerSession

LINK_FRAME_STYLES: dict[str, dict] = {
    "retarget_base": {"origin_color": (80, 255, 80), "axes_length": 0.025},
    "index": {"origin_color": (0, 180, 255), "axes_length": 0.018},
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
    "thumb": {
        "origin_color": (255, 215, 150),
        "axes_length": 0.015,
        "axes_radius": 0.0012,
        "origin_radius": 0.003,
    },
}

# Fixed Ditto leader mount pose in the viewer (base_link frame relative to world).
DITTO_DEFAULT_MOUNT_POSITION = (-0.046, -0.12, 0.1)
DITTO_DEFAULT_MOUNT_RPY_DEG = (90.0, 0.0, 90.0)

# Sharpa mount:
SHARPA_DEFAULT_MOUNT_POSITION = (0.0, 0.12, 0.0)

VIZ_IDLE_POLL_S = 1.0
VIZ_HARDWARE_POLL_HZ = 60.0


def _style_for_link(link_name: str) -> dict:
    if link_name == "retarget_base":
        return LINK_FRAME_STYLES["retarget_base"]
    if "thumb" in link_name:
        return LINK_FRAME_STYLES["thumb"]
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
    ik_gizmos: dict[str, viser.TransformControlsHandle] = field(default_factory=dict)
    ik_dragging: dict[str, bool] = field(default_factory=dict)
    suppress_slider_callbacks: bool = False


@dataclass
class SharpaRetargetTargetViz:
    index_frame: viser.FrameHandle
    thumb_frame: viser.FrameHandle


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
    on_gizmo_change: Callable[[IkSolveParams], None] | None = None,
) -> None:
    """World-space drag targets parented under the Ditto mount (= base_link)."""
    gizmo_styles = {
        "index": {"scale": 0.018, "opacity": 0.9},
        "thumb": {"scale": 0.018, "opacity": 0.9},
    }
    for finger in ("index", "thumb"):
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
    return q


def _apply_configuration(robot: RobotViz) -> None:
    robot.viser_urdf.update_cfg(_build_full_configuration(robot))


def _apply_ditto_cfg_from_angles(robot: RobotViz, joint_angles: np.ndarray) -> None:
    """Apply leader joint angles (rad) to the Ditto Viser model and sliders."""
    joint_names = list(robot.viser_urdf.get_actuated_joint_names())
    cfg = np.asarray(joint_angles, dtype=float)
    if cfg.shape[0] != len(joint_names):
        raise ValueError(
            f"Expected {len(joint_names)} Ditto joint angles, got {cfg.shape[0]}"
        )
    robot.suppress_slider_callbacks = True
    try:
        robot.viser_urdf.update_cfg(cfg)
        for joint_name, slider in zip(robot.slider_joint_names, robot.slider_handles):
            slider.value = float(cfg[joint_names.index(joint_name)])
    finally:
        robot.suppress_slider_callbacks = False


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
) -> SharpaRetargetTargetViz:
    """Ghost pad frames in Sharpa base_link showing the IK target pose."""
    frames: dict[str, viser.FrameHandle] = {}
    for finger in ("index", "thumb"):
        style = SHARPA_RETARGET_TARGET_STYLES[finger]
        frames[finger] = server.scene.add_frame(
            f"{sharpa_robot.root_name}/retarget_target/{finger}",
            show_axes=True,
            axes_length=style["axes_length"],
            axes_radius=style["axes_radius"],
            origin_radius=style["origin_radius"],
            origin_color=style["origin_color"],
        )
    return SharpaRetargetTargetViz(
        index_frame=frames["index"],
        thumb_frame=frames["thumb"],
    )


def _sync_sharpa_retarget_target_frames(
    target_viz: SharpaRetargetTargetViz,
    result: RetargetResult,
) -> None:
    for finger, pose, handle in (
        ("index", result.index_target_in_sharpa_base, target_viz.index_frame),
        ("thumb", result.thumb_target_in_sharpa_base, target_viz.thumb_frame),
    ):
        position, wxyz = _viser_pose_from_pin_se3(pose)
        handle.position = position
        handle.wxyz = wxyz


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
) -> tuple[list[str], dict[str, float]]:
    all_names = list(viser_urdf.get_actuated_joint_names())
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
    )

    robot = RobotViz(
        name=name,
        viser_urdf=viser_urdf,
        root_name=root_name,
        mount_frame=mount_frame,
        locked_joint_values=locked_values,
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
    hardware: LeaderHardwareSession | None = None,
    sharpa_follower: "SharpaFollowerSession | None" = None,
) -> None:

    for label, path in (
        ("Ditto leader", DITTO_LEADER_URDF),
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
    server.gui.add_markdown("\n".join(help_lines))
    server.initial_camera.position = (0.35, -0.45, 0.25)
    server.initial_camera.look_at = (0.05, 0.0, 0.05)

    robots: list[RobotViz] = []

    ditto_robot: RobotViz | None = None
    sharpa_robot: RobotViz | None = None
    ditto_ik: DittoFingerIK | None = None
    sharpa_ik: SharpaFingerIK | None = None
    retargeter: DittoToSharpaRetargeter | None = None
    sharpa_retarget_targets: SharpaRetargetTargetViz | None = None
    retarget_error_markdown: viser.GuiMarkdownHandle | None = None
    live_retarget_enabled = {"value": True}
    hardware_drive_enabled = {"value": hardware is not None}
    sharpa_send_enabled = {"value": sharpa_follower is not None}

    def _retarget_ditto_to_sharpa(
        solve_params: IkSolveParams | None = IK_POLISH,
    ) -> None:
        if (
            not live_retarget_enabled["value"]
            or ditto_robot is None
            or sharpa_robot is None
            or ditto_ik is None
            or sharpa_ik is None
            or retargeter is None
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
        result = retargeter.retarget(
            ditto_q,
            sharpa_q_seed,
            solve_params=solve_params,
        )
        _set_configuration_from_pin_q(sharpa_robot, sharpa_ik, result.sharpa_q)
        if sharpa_retarget_targets is not None:
            _sync_sharpa_retarget_target_frames(sharpa_retarget_targets, result)
        if retarget_error_markdown is not None:
            retarget_error_markdown.content = (
                f"**Retarget IK error (rad):** index {result.index_residual:.3f}, "
                f"thumb {result.thumb_residual:.3f}"
            )
        if sharpa_follower is not None and sharpa_send_enabled["value"]:
            sharpa_follower.send_q(result.sharpa_q)

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
                if sharpa_send.value:
                    _retarget_ditto_to_sharpa(IK_POLISH)

            sharpa_status_markdown = server.gui.add_markdown(
                "**Sharpa hardware:** streaming retargeted joints"
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
        ditto_ik = DittoFingerIK(DITTO_LEADER_URDF)

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
            DITTO_LEADER_URDF,
            name="Ditto leader",
            root_name="/ditto_leader",
            position=DITTO_DEFAULT_MOUNT_POSITION,
            slider_folder="Ditto leader joints",
            frame_links=DITTO_VIZ_FRAME_LINKS,
            on_before_slider_apply=_on_ditto_slider_change,
            mount_rpy_deg=DITTO_DEFAULT_MOUNT_RPY_DEG,
        )
        _add_ditto_ik_gizmos(
            server,
            ditto_robot,
            ditto_ik,
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
            slider_folder="Sharpa index + thumb joints",
            frame_links=SHARPA_VIZ_FRAME_LINKS,
            sharpa_index_thumb_only=True,
        )
        robots.append(sharpa_robot)

    if show_leader and show_sharpa:
        retargeter = DittoToSharpaRetargeter()
        assert sharpa_robot is not None and sharpa_ik is not None
        if sharpa_follower is not None:
            print("Connecting Sharpa follower hardware...")
            try:
                sharpa_follower.start(sharpa_ik.joint_q_index)
            except Exception as exc:  # noqa: BLE001
                print(f"  Sharpa follower unavailable, continuing without it: {exc}")
                sharpa_follower = None
                sharpa_send_enabled["value"] = False
                if sharpa_status_markdown is not None:
                    sharpa_status_markdown.content = "**Sharpa hardware:** unavailable"
        sharpa_retarget_targets = _add_sharpa_retarget_target_frames(server, sharpa_robot)
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

        _retarget_ditto_to_sharpa(IK_POLISH)

    server.scene.add_grid("/grid", width=1.0, height=1.0, position=(0.0, 0.0, 0.0))

    print("Viser asset viewer running.")
    print(f"  Ditto leader URDF: {DITTO_LEADER_URDF}")
    print(f"  Sharpa right URDF: {SHARPA_RIGHT_URDF}")
    if hardware is not None:
        port = hardware.config.u2d2.get("usb_port", "/dev/ttyUSB0")
        print(f"  Ditto leader hardware: polling {port} (torque_off, no force feedback)")
        if not hardware.is_receiving:
            print("  Waiting for valid encoder reads (sliders/gizmos work meanwhile).")
    if sharpa_follower is not None:
        print("  Sharpa follower: streaming retargeted index + thumb joints.")
    print("Press Ctrl+C to exit.")

    poll_interval = (
        1.0 / VIZ_HARDWARE_POLL_HZ if hardware is not None else VIZ_IDLE_POLL_S
    )
    try:
        while True:
            if (
                hardware is not None
                and hardware_drive_enabled["value"]
                and ditto_robot is not None
                and ditto_ik is not None
                and not _any_ik_dragging(ditto_robot)
            ):
                angles = hardware.poll_joint_angles()
                if angles is not None:
                    _apply_ditto_cfg_from_angles(ditto_robot, angles)
                    _sync_ik_gizmos(ditto_robot, ditto_ik)
                    _retarget_ditto_to_sharpa(IK_POLISH)
                    if hardware_status_markdown is not None:
                        hardware_status_markdown.content = (
                            "**Hardware status:** receiving encoder data"
                        )
                elif hardware_status_markdown is not None:
                    hardware_status_markdown.content = (
                        "**Hardware status:** no valid reads yet — check USB / power / port"
                    )
            time.sleep(poll_interval)
    except KeyboardInterrupt:
        print("\nShutting down viewer...")
    finally:
        if hardware is not None:
            hardware.stop()
        if sharpa_follower is not None:
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
    args = parser.parse_args()

    if args.leader_only and args.sharpa_only:
        parser.error("Choose at most one of --leader-only / --sharpa-only.")

    run_viewer(
        show_leader=not args.sharpa_only,
        show_sharpa=not args.leader_only,
        hardware=None,
    )


if __name__ == "__main__":
    main()
