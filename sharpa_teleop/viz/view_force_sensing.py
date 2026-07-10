#!/usr/bin/env python3
"""Debug Sharpa joint-torque → pad-force (Jᵀ estimate) mapping in Viser.

Sharpa-only: joint sliders drive the hand; live joint torques and the estimated
pad force are drawn on the URDF. Optionally overlay tactile pad force for
direction comparison.

Visuals per active finger:
  - **Linear arrows** along each joint axis — torque magnitude + sense (Nm → m)
  - **Pad force arrow** at the retargeting pad — ``estimate_pad_force`` result
  - **Tactile pad force** (optional, different color) for A/B direction check

Usage (from sharpa_teleop repo root)::

    python sharpa_teleop/viz/view_force_sensing.py
    python sharpa_teleop/viz/view_force_sensing.py --no-tactile
    python sharpa_teleop/viz/view_force_sensing.py --calibrate
    python sharpa_teleop/viz/view_force_sensing.py --finger index

From sharpa_teleop/ package dir::

    python viz/view_force_sensing.py
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_PKG = Path(__file__).resolve().parent.parent
_REPO = _PKG.parent
for _path in (_PKG, _REPO):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import numpy as np  # noqa: E402
import pinocchio as pin  # noqa: E402
import viser  # noqa: E402
from hydra import compose, initialize_config_dir  # noqa: E402

from _paths import CONF_DIR, DITTO_CONF_DIR  # noqa: E402
from retargeting.paths import (  # noqa: E402
    DITTO_LEADER_ONLY_HAND_CONFIG,
    SHARPA_RIGHT_URDF,
    SHARPA_VIZ_FRAME_LINKS,
)
from retargeting.sharpa_ik import FingerName, SharpaFingerIK  # noqa: E402
from teleop.force_sources import TactileForceSource  # noqa: E402
from viz.view_assets import (  # noqa: E402
    FORCE_VIZ_DEFAULT_SCALE,
    RobotViz,
    _add_robot,
    _build_full_configuration,
    _update_force_arrows,
    _viser_cfg_to_pin_q,
)

FINGERS: tuple[FingerName, ...] = ("index", "middle", "thumb")
UPDATE_HZ = 30.0

# Linear joint-torque arrow: meters of length per Nm.
TORQUE_AXIS_SCALE_DEFAULT = 0.02

JOINT_TORQUE_COLOR = (255, 220, 40)
TACTILE_FORCE_COLORS: dict[str, tuple[int, int, int]] = {
    "index": (80, 220, 255),
    "middle": (120, 255, 180),
    "thumb": (100, 255, 220),
}


def _update_joint_torque_viz(
    server: viser.ViserServer,
    robot: RobotViz,
    joint_torques: list[tuple[str, np.ndarray, np.ndarray, float]],
    axis_scale: float,
) -> None:
    """Draw linear axis arrows for each joint torque.

    ``joint_torques`` entries are ``(name, origin, axis, tau_Nm)``.
    """
    lin_points: list[list[np.ndarray]] = []
    lin_colors: list[tuple[int, int, int]] = []

    for _name, origin, axis, tau in joint_torques:
        origin = np.asarray(origin, dtype=float)
        axis = np.asarray(axis, dtype=float)
        n = np.linalg.norm(axis)
        if n < 1e-9 or abs(tau) < 1e-6:
            continue
        axis = axis / n
        tip = origin + axis_scale * tau * axis
        lin_points.append([origin, tip])
        lin_colors.append(JOINT_TORQUE_COLOR)

    if lin_points:
        server.scene.add_arrows(
            f"{robot.root_name}/joint_torque_axis",
            points=np.asarray(lin_points, dtype=float),
            colors=np.asarray(lin_colors, dtype=np.uint8),
            shaft_radius=0.0012,
            head_radius=0.0032,
            head_length=0.006,
        )


def _update_tactile_force_arrows(
    server: viser.ViserServer,
    robot: RobotViz,
    forces_in_base: dict[str, tuple[np.ndarray, np.ndarray]],
    scale: float,
) -> None:
    points = []
    colors = []
    for finger, (origin, force) in forces_in_base.items():
        points.append([origin, origin + scale * force])
        colors.append(TACTILE_FORCE_COLORS.get(finger, (80, 220, 255)))
    if not points:
        return
    server.scene.add_arrows(
        f"{robot.root_name}/force_tactile",
        points=np.asarray(points, dtype=float),
        colors=np.asarray(colors, dtype=np.uint8),
        shaft_radius=0.0015,
        head_radius=0.004,
        head_length=0.008,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--no-tactile",
        action="store_true",
        help="Skip tactile overlay (estimate + joint torques only)",
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Calibrate tactile sensors on enable",
    )
    parser.add_argument(
        "--finger",
        choices=("index", "middle", "thumb", "all"),
        default="all",
        help="Which finger(s) to visualize (default: all)",
    )
    args, hydra_args = parser.parse_known_args()

    fingers: tuple[FingerName, ...]
    if args.finger == "all":
        fingers = FINGERS
    else:
        fingers = (args.finger,)  # type: ignore[assignment]

    searchpath = f"hydra.searchpath=[file://{DITTO_CONF_DIR}]"
    overrides = [searchpath]
    if not any(a.startswith("hand_config=") for a in hydra_args):
        overrides.append(f"hand_config={DITTO_LEADER_ONLY_HAND_CONFIG}")
    overrides.extend(hydra_args)

    with initialize_config_dir(version_base=None, config_dir=str(CONF_DIR)):
        cfg = compose(config_name="config", overrides=overrides)

    if not SHARPA_RIGHT_URDF.is_file():
        raise FileNotFoundError(f"Missing Sharpa URDF: {SHARPA_RIGHT_URDF}")

    from hardware_interfaces.sharpa_follower.session import SharpaFollowerSession

    sharpa_ik = SharpaFingerIK(SHARPA_RIGHT_URDF)
    follower = SharpaFollowerSession(cfg, verbose=True)
    follower.start(sharpa_ik.joint_q_index)

    show_tactile = not args.no_tactile
    tactile_source: TactileForceSource | None = None
    if show_tactile:
        ok = follower.enable_tactile(calibrate=args.calibrate)
        if not ok:
            print("Tactile unavailable — continuing with estimate only.")
            show_tactile = False
        else:
            tactile_source = TactileForceSource(sharpa_ik, fingers)

    server = viser.ViserServer()
    server.gui.add_markdown(
        "## Sharpa force sensing debug\n"
        "- **Yellow arrows**: joint torque along axis (τ · axis)\n"
        "- **Warm pad arrows**: Jᵀ estimate pad force (τ = Jᵀ F)\n"
        "- **Cool pad arrows**: tactile pad force (if enabled)\n"
        "- Pose the hand with sliders, then press on a fingertip"
    )

    robot = _add_robot(
        server,
        SHARPA_RIGHT_URDF,
        name="sharpa",
        root_name="/sharpa_right",
        position=(0.0, 0.0, 0.0),
        slider_folder="Sharpa joints",
        frame_links=SHARPA_VIZ_FRAME_LINKS,
        sharpa_3f_retarget=True,
    )
    server.scene.add_grid("/grid", width=1.0, height=1.0, position=(0.0, 0.0, 0.0))

    force_scale = {"value": FORCE_VIZ_DEFAULT_SCALE}
    torque_scale = {"value": TORQUE_AXIS_SCALE_DEFAULT}
    with server.gui.add_folder("Viz scales"):
        force_slider = server.gui.add_slider(
            "Force scale (m/N)",
            min=0.005,
            max=0.1,
            step=0.001,
            initial_value=force_scale["value"],
        )
        torque_slider = server.gui.add_slider(
            "Torque axis scale (m/Nm)",
            min=0.005,
            max=0.08,
            step=0.001,
            initial_value=torque_scale["value"],
        )

        @force_slider.on_update
        def _on_force_scale(_: viser.GuiEvent) -> None:
            force_scale["value"] = float(force_slider.value)

        @torque_slider.on_update
        def _on_torque_scale(_: viser.GuiEvent) -> None:
            torque_scale["value"] = float(torque_slider.value)

    status = server.gui.add_markdown("Waiting for Sharpa state…")

    period = 1.0 / UPDATE_HZ
    try:
        while True:
            t0 = time.perf_counter()
            joint_names = list(robot.viser_urdf.get_actuated_joint_names())
            q_slider = _viser_cfg_to_pin_q(
                sharpa_ik, _build_full_configuration(robot), joint_names
            )
            follower.send_q(q_slider)

            inputs = follower.read_wrench_inputs(q_slider)
            if inputs is None:
                status.content = "No Sharpa state yet."
                time.sleep(max(0.0, period - (time.perf_counter() - t0)))
                continue
            q_meas, finger_torques = inputs

            # Keep the mesh aligned with the measured pose used for Jᵀ.
            cfg_meas = np.array(
                [q_meas[sharpa_ik.joint_q_index(n)] for n in joint_names],
                dtype=float,
            )
            robot.suppress_slider_callbacks = True
            try:
                robot.viser_urdf.update_cfg(cfg_meas)
            finally:
                robot.suppress_slider_callbacks = False

            pin.forwardKinematics(sharpa_ik.model, sharpa_ik.data, q_meas)
            pin.updateFramePlacements(sharpa_ik.model, sharpa_ik.data)

            estimate_forces: dict[str, tuple[np.ndarray, np.ndarray]] = {}
            torque_viz: list[tuple[str, np.ndarray, np.ndarray, float]] = []
            lines: list[str] = []

            for finger in fingers:
                tau = finger_torques.get(finger)
                if tau is None:
                    continue
                frames = sharpa_ik.finger_joint_frames_in_base(q_meas, finger)
                names = sharpa_ik.finger_joint_names(finger)
                for name, (origin, axis), t in zip(names, frames, tau, strict=True):
                    torque_viz.append((name, origin, axis, float(t)))
                f_est = sharpa_ik.estimate_pad_force(q_meas, finger, tau)
                origin = np.asarray(
                    sharpa_ik.pad_pose_in_base(q_meas, finger).translation, dtype=float
                )
                estimate_forces[finger] = (origin, f_est)
                tau_str = ", ".join(f"{float(t):+.3f}" for t in tau)
                lines.append(
                    f"**{finger}** τ=[{tau_str}] Nm → "
                    f"F_est=({f_est[0]:+.2f},{f_est[1]:+.2f},{f_est[2]:+.2f}) "
                    f"|F|={np.linalg.norm(f_est):.2f} N"
                )

            _update_joint_torque_viz(
                server, robot, torque_viz, torque_scale["value"]
            )
            _update_force_arrows(
                server,
                robot,
                estimate_forces,
                force_scale["value"],
                name="force_estimate",
            )

            if show_tactile and tactile_source is not None:
                tactile_read = tactile_source.read(follower, q_meas)
                if tactile_read is not None:
                    _, f_by_finger = tactile_read
                    tactile_forces: dict[str, tuple[np.ndarray, np.ndarray]] = {}
                    for finger, f_tac in f_by_finger.items():
                        if finger not in estimate_forces:
                            continue
                        origin = estimate_forces[finger][0]
                        tactile_forces[finger] = (origin, f_tac)
                        f_est = estimate_forces[finger][1]
                        lines.append(
                            f"  tactile F=({f_tac[0]:+.2f},{f_tac[1]:+.2f},{f_tac[2]:+.2f}) "
                            f"|F|={np.linalg.norm(f_tac):.2f} N  "
                            f"dot(est,tac)={float(np.dot(f_est, f_tac)):.2f}"
                        )
                    _update_tactile_force_arrows(
                        server, robot, tactile_forces, force_scale["value"]
                    )

            status.content = "\n\n".join(lines) if lines else "No finger torques."
            time.sleep(max(0.0, period - (time.perf_counter() - t0)))
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        follower.stop()


if __name__ == "__main__":
    main()
