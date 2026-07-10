"""Retargeting force rendering: estimated Ditto joint torque → leader current.

Bilateral loop for the Ditto **index** finger:

1. read the Ditto leader (current mode) → leader ``q``
2. retarget Ditto → Sharpa and stream to the Sharpa follower (slow / throttled)
3. read Sharpa contact torque → estimate pad force → map to *estimated Ditto
   joint torque* via ``Jᵀ`` (the "follower" signal)
4. turn that estimated joint torque into a synthetic follower current and feed it
   through ditto's ``CurrentController`` force rendering → leader current

This mirrors ``joint_teleop``'s force-rendering pipeline, but the follower signal
is our model-based estimate rather than a directly-mapped Sharpa joint. The
expensive retargeting IK is throttled so it does not starve the high-rate leader
current loop (shared GIL).
"""

from __future__ import annotations

import queue
import sys
import threading
import time
from typing import TYPE_CHECKING

import numpy as np
from omegaconf import DictConfig

from _paths import setup_import_paths

setup_import_paths()

from ditto.hand_interfaces.src.current_control import (  # noqa: E402
    ControlParams,
    CurrentController,
    create_control_params_from_config,
)
from ditto.hand_interfaces.src.hand_interface import (  # noqa: E402
    create_hand_interface,
)
from ditto.hand_interfaces.src.motor_data import MotorData  # noqa: E402
from ditto.utils.utils import precise_sleep  # noqa: E402

from retargeting.ik_utils import IK_STREAM  # noqa: E402
from hardware_interfaces.ditto_leader.conventions import (  # noqa: E402
    ditto_hardware_joint_signs,
    leader_joint_names_for_motor_ids,
)
from retargeting.paths import (  # noqa: E402
    DITTO_3F_LEADER_JOINT_NAMES,
    DITTO_3F_LEADER_MOTOR_IDS,
    DITTO_INDEX_JOINT_NAMES,
    DITTO_LEADER_MOTOR_IDS,
    DITTO_MIDDLE_JOINT_NAMES,
    DITTO_THUMB_JOINT_NAMES,
    SHARPA_INDEX_JOINT_NAMES,
    SHARPA_MIDDLE_JOINT_NAMES,
    SHARPA_THUMB_JOINT_NAMES,
    ditto_leader_urdf,
)
from retargeting.retargeter import DittoToSharpaRetargeter  # noqa: E402
from teleop.engine import RetargetTeleopEngine  # noqa: E402
from teleop.force_sources import (  # noqa: E402
    TactileForceSource,
    TorqueEstimateForceSource,
    make_force_source_per_finger,
)

if TYPE_CHECKING:
    from hardware_interfaces.sharpa_follower.session import SharpaFollowerSession

_MOTOR_TO_JOINT = dict(zip(DITTO_3F_LEADER_MOTOR_IDS, DITTO_3F_LEADER_JOINT_NAMES))
_JOINT_TO_MOTOR = {joint: motor for motor, joint in _MOTOR_TO_JOINT.items()}
_JOINT_TO_FINGER = {
    **{name: "index" for name in DITTO_INDEX_JOINT_NAMES},
    **{name: "middle" for name in DITTO_MIDDLE_JOINT_NAMES},
    **{name: "thumb" for name in DITTO_THUMB_JOINT_NAMES},
}
# Sharpa URDF joint name order per finger (matches read_wrench_inputs torque order).
_SHARPA_FINGER_JOINTS = {
    "index": SHARPA_INDEX_JOINT_NAMES,
    "middle": SHARPA_MIDDLE_JOINT_NAMES,
    "thumb": SHARPA_THUMB_JOINT_NAMES,
}

# Decoupled rate for the expensive retarget IK + force estimate.
DEFAULT_RETARGET_HZ = 40.0

# Per-finger control mode lives entirely in hand_config.control.fingers:
#   position: "retarget" (IK) | "joint" (direct leader→Sharpa joint map)
#   force:    "measured" (Sharpa joint current, joint_teleop style)
#           | "estimate" | "tactile" | "mix"  (task-space contact source)
# Shortcut strings: "retarget" == {retarget, estimate}; "joint" == {joint, measured}.
_FINGER_MODE_SHORTCUTS = {
    "retarget": {"position": "retarget", "force": "estimate"},
    "joint": {"position": "joint", "force": "measured"},
}
_VALID_POSITIONS = ("retarget", "joint")
_VALID_FORCE_MODALITIES = ("measured", "estimate", "tactile")
# Task-space contact sources (everything except the direct joint-current path).
_SOURCE_FORCES = ("estimate", "tactile", "mix")
_TASK_SPACE_MODALITIES = ("estimate", "tactile")


def _normalize_force_spec(force_spec) -> dict[str, float]:
    """Expand force config to ``{modality: weight}`` (weights sum to 1).

    Accepts a shortcut string (``estimate`` | ``tactile`` | ``measured`` | ``mix``),
    or a weighted dict (``{tactile: 0.5, measured: 0.5}``).
    """
    if isinstance(force_spec, str):
        if force_spec == "mix":
            return {"estimate": 0.5, "tactile": 0.5}
        if force_spec not in (*_VALID_FORCE_MODALITIES, "mix"):
            raise ValueError(
                f"force={force_spec!r} (expected one of "
                f"{list(_VALID_FORCE_MODALITIES)} | mix | a weight dict)"
            )
        return {force_spec: 1.0}
    if hasattr(force_spec, "items"):
        weights = {str(k): float(v) for k, v in force_spec.items()}
        unknown = set(weights) - set(_VALID_FORCE_MODALITIES)
        if unknown:
            raise ValueError(
                f"Unknown force modalities {sorted(unknown)} "
                f"(expected {list(_VALID_FORCE_MODALITIES)})"
            )
        total = sum(weights.values())
        if total <= 0.0:
            raise ValueError("Force blend weights must sum to a positive value")
        return {k: v / total for k, v in weights.items()}
    raise ValueError(
        f"force must be a modality string or a weight dict, got {type(force_spec)!r}"
    )


def _normalize_finger_mode(finger: str, spec) -> dict:
    """Expand a control.fingers entry to {position, force: {modality: weight}}."""
    if isinstance(spec, str):
        if spec not in _FINGER_MODE_SHORTCUTS:
            raise ValueError(
                f"control.fingers.{finger}={spec!r} (expected one of "
                f"{list(_FINGER_MODE_SHORTCUTS)} or a {{position, force}} mapping)"
            )
        shortcut = dict(_FINGER_MODE_SHORTCUTS[spec])
        return {
            "position": shortcut["position"],
            "force": _normalize_force_spec(shortcut["force"]),
        }
    position = str(spec.get("position", "retarget"))
    if position not in _VALID_POSITIONS:
        raise ValueError(
            f"control.fingers.{finger}.position={position!r} "
            f"(expected {list(_VALID_POSITIONS)})"
        )
    force_raw = spec.get("force", "estimate")
    return {"position": position, "force": _normalize_force_spec(force_raw)}


def _finger_uses_modality(force_weights: dict[str, float], modality: str) -> bool:
    return float(force_weights.get(modality, 0.0)) > 0.0


def _finger_uses_task_space(force_weights: dict[str, float]) -> bool:
    return any(_finger_uses_modality(force_weights, m) for m in _TASK_SPACE_MODALITIES)


def config_needs_tactile(config: DictConfig) -> bool:
    """True if any finger's force blend includes tactile."""
    return any(
        _finger_uses_modality(m["force"], "tactile")
        for m in finger_modes_from_config(config).values()
    )


def viewer_initial_source(config: DictConfig) -> str:
    """Best-effort single source for the viewer's initial dropdown value.

    Returns ``estimate`` or ``tactile`` only (viewer compares those two live).
    Weighted config blends prefer tactile when present, else estimate.
    """
    modes: set[str] = set()
    for m in finger_modes_from_config(config).values():
        fw = m["force"]
        if _finger_uses_modality(fw, "tactile"):
            modes.add("tactile")
        if _finger_uses_modality(fw, "estimate"):
            modes.add("estimate")
        if _finger_uses_modality(fw, "measured") and not modes:
            modes.add("estimate")
    if "tactile" in modes:
        return "tactile"
    return "estimate"


def leader_fingers_from_config(config: DictConfig) -> tuple[str, ...]:
    """Ordered unique fingers present in the leader motor chain."""
    fingers: list[str] = []
    for motor in config.hand_config.leader.motor_ids:
        finger = _JOINT_TO_FINGER[_MOTOR_TO_JOINT[int(motor)]]
        if finger not in fingers:
            fingers.append(finger)
    return tuple(fingers)


def finger_modes_from_config(config: DictConfig) -> dict[str, dict]:
    """Read hand_config.control.fingers → {finger: {position, force: {mod: w}}}."""
    control = config.hand_config.get("control") or {}
    fingers_cfg = control.get("fingers") or {}
    return {
        finger: _normalize_finger_mode(finger, fingers_cfg.get(finger, "retarget"))
        for finger in leader_fingers_from_config(config)
    }


def _single_task_space_source_mode(force_weights: dict[str, float]) -> str | None:
    """Task-space source name when a finger uses exactly one estimate/tactile/mix mode."""
    if len(force_weights) != 1:
        return None
    only = next(iter(force_weights))
    if only in _TASK_SPACE_MODALITIES:
        return only
    if only == "mix":
        return "mix"
    return None


def is_force_render_config(config: DictConfig) -> bool:
    """True if the hand_config drives the leader in current (haptic) mode."""
    return str(config.hand_config.leader.mode) == "current"


# ditto's force_rendering law applies *negative* feedback
# (-k·follower_current), which is tuned for joint_teleop where the follower
# signal is a directly-measured follower torque. Here the follower signal is the
# estimated leader joint torque we want to *reproduce* (τ = Jᵀ·F, the contact
# reaction). Negating the synthetic current makes the law render +τ (resist the
# user into the contact) instead of -τ. Flip this if the felt force is inverted.
FORCE_RENDER_FEEDBACK_SIGN = -1.0


class RetargetForceRenderTeleop:
    """Ditto index leader (current) ↔ Sharpa follower with model-based force rendering."""

    def __init__(
        self,
        config: DictConfig,
        sharpa_follower: "SharpaFollowerSession | None" = None,
        *,
        tactile_calibrate: bool | None = None,
        retarget_hz: float = DEFAULT_RETARGET_HZ,
        force_hz: float | None = None,
        state_queue: queue.Queue | None = None,
        verbose: bool | None = None,
    ) -> None:
        self.config = config
        self.state_queue = state_queue
        self.verbose = (
            verbose
            if verbose is not None
            else bool(config.hand_config.get("verbose", config.get("verbose", True)))
        )
        self.control_frequency = float(
            config.hand_config.get(
                "control_frequency", config.get("control_frequency", 200)
            )
        )
        self.dt = 1.0 / self.control_frequency
        # IK retarget (expensive, drives Sharpa) runs slow; tactile/force sampling
        # (cheap: Sharpa read_state + tactile fetch + Jᵀ) runs as fast as the
        # control loop so the rendered haptic signal stays fresh.
        self.retarget_period = 1.0 / float(retarget_hz)
        # Force/tactile sampling defaults to the IK rate (the safe, measured-OK
        # behavior). Raise force_hz only after the [perf:worker] readout shows the
        # Sharpa read_state + tactile fetch have headroom — over-polling a blocking
        # SDK fetch stalls on its timeout and starves the leader loop.
        self.force_period = 1.0 / float(force_hz or retarget_hz)
        self.show_current_breakdown = bool(
            config.hand_config.get(
                "show_current_breakdown", config.get("show_current_breakdown", False)
            )
        )
        # Periodic [perf] / [perf:worker] Hz summaries (overrun warnings always print).
        self.show_perf = bool(
            config.hand_config.get("show_perf", config.get("show_perf", True))
        )

        leader_cfg = config.hand_config.leader
        if leader_cfg.mode != "current":
            raise ValueError(
                "RetargetForceRenderTeleop requires leader mode 'current', "
                f"got {leader_cfg.mode!r} (use hand_config=ditto_2f_tactile)."
            )
        self.leader_chain = list(leader_cfg.motor_ids)
        self._validate_leader_motors()
        _chain_joint_names = leader_joint_names_for_motor_ids(self.leader_chain)
        self._joint_sign = dict(
            zip(
                _chain_joint_names,
                ditto_hardware_joint_signs(_chain_joint_names),
                strict=True,
            )
        )

        # Per-finger control mode (position + force source) lives entirely in
        # hand_config.control.fingers (see finger_modes_from_config). Two
        # independent axes per finger:
        #   position: "retarget" (IK) | "joint" (direct leader→Sharpa joint map)
        #   force:    weighted blend of measured | estimate | tactile (sums to 1)
        self._finger_modes = finger_modes_from_config(config)
        self._finger_force_weights = {
            f: dict(m["force"]) for f, m in self._finger_modes.items()
        }
        all_fingers = tuple(self._finger_modes)
        self._position_joint_fingers = tuple(
            f for f in all_fingers if self._finger_modes[f]["position"] == "joint"
        )
        self._position_retarget_fingers = tuple(
            f for f in all_fingers if self._finger_modes[f]["position"] == "retarget"
        )
        self._force_measured_fingers = tuple(
            f
            for f, w in self._finger_force_weights.items()
            if _finger_uses_modality(w, "measured")
        )
        self._force_task_space_fingers = tuple(
            f
            for f, w in self._finger_force_weights.items()
            if _finger_uses_task_space(w)
        )
        # Legacy viewer path: single task-space modality per finger only.
        self._finger_source_modes = {
            f: mode
            for f, w in self._finger_force_weights.items()
            if (mode := _single_task_space_source_mode(w)) is not None
        }
        # finger -> [(ditto_joint, sharpa_urdf_joint, scale)] for measured force.
        self._joint_map = self._parse_joint_map()

        # Middle joints live on the 3-finger Ditto URDF; use it for all hardware.
        self.engine = RetargetTeleopEngine(
            sharpa_follower=sharpa_follower,
            fingers=self._position_retarget_fingers,
            retargeter=DittoToSharpaRetargeter.from_sharpa_config(
                self.config.get("sharpa"),
                ditto_urdf=ditto_leader_urdf(three_finger=True),
            ),
        )
        self.sharpa_follower = sharpa_follower

        # Tactile options are config-driven (hand_config.force_render.calibrate /
        # .debug). The per-finger task-space source comes from control.fingers.
        fr_cfg = config.hand_config.get("force_render") or {}
        self.tactile_calibrate = (
            bool(fr_cfg.get("calibrate", False))
            if tactile_calibrate is None
            else bool(tactile_calibrate)
        )
        self.tactile_debug = bool(fr_cfg.get("debug", False))
        self._needs_tactile = config_needs_tactile(config)
        self.engine.force_source = self._make_force_source()
        self._estimate_source = TorqueEstimateForceSource(
            self.engine.sharpa_ik, all_fingers
        )
        self._tactile_source = TactileForceSource(
            self.engine.sharpa_ik, all_fingers, debug=self.tactile_debug
        )

        self.leader_hand = None
        self.current_controller = CurrentController()
        self.leader_control_params = self._create_leader_control_params()
        self._torque_to_mA, self._torque_filter_alpha = self._parse_force_mapping()
        self._modality_filter_alpha = self._parse_modality_filters()

        self._cached_torques: dict[str, float] = {}
        self._filtered_torque_nm: dict[int, float] = {}
        self._filtered_torque_by_modality: dict[tuple[str, str], float] = {}
        self._rate_limited_command_mA: dict[int, float] = {}
        self.running = False
        # Shared with the retarget worker thread. The fast loop publishes the
        # latest leader read here; the worker consumes it for IK + force estimate
        # and publishes back into self._cached_torques. Plain attribute
        # assignment is atomic under the GIL, so no lock is needed for these
        # single-reference hand-offs.
        self._latest_leader_data: list[MotorData] | None = None
        self._retarget_thread: threading.Thread | None = None
        self._prev_switch_interval = sys.getswitchinterval()
        self._worker_perf_line: str | None = None

    # ----- setup -------------------------------------------------------------

    def _validate_leader_motors(self) -> None:
        for motor in self.leader_chain:
            if motor not in _MOTOR_TO_JOINT:
                raise ValueError(
                    f"Leader motor {motor} is not a known Ditto motor "
                    f"(7-DoF {list(DITTO_LEADER_MOTOR_IDS)} or "
                    f"10-DoF {list(DITTO_3F_LEADER_MOTOR_IDS)})"
                )

    def _make_force_source(self):
        # Viewer / single-modality path only; blended fingers use _compute_cached_torques.
        if not self._finger_source_modes:
            return TorqueEstimateForceSource(self.engine.sharpa_ik, ())
        return make_force_source_per_finger(
            self.engine.sharpa_ik,
            self._finger_source_modes,
            tactile_debug=self.tactile_debug,
        )

    def _parse_joint_map(self) -> dict[str, list[tuple[str, str, float]]]:
        """Read control.joint_map for fingers that use measured force."""
        needs_map = set(self._force_measured_fingers) | set(self._position_joint_fingers)
        control = self.config.hand_config.get("control") or {}
        joint_map_cfg = control.get("joint_map") or {}
        joint_map: dict[str, list[tuple[str, str, float]]] = {}
        for finger in needs_map:
            pairs = joint_map_cfg.get(finger)
            if not pairs:
                raise ValueError(
                    f"control.fingers.{finger} needs a control.joint_map.{finger} "
                    "(joint position and/or measured force)"
                )
            joint_map[finger] = [
                (
                    str(p["ditto_joint"]),
                    str(p["sharpa_joint"]),
                    float(p.get("scale", 1.0)),
                )
                for p in pairs
            ]
        return joint_map

    def _create_leader_control_params(self) -> list[ControlParams]:
        params_list: list[ControlParams] = []
        leader_config = self.config.hand_config.leader
        for motor_id in self.leader_chain:
            motor_config = {}
            if leader_config.get("joint_settings"):
                for joint_config in leader_config.joint_settings:
                    if (
                        joint_config["motor_id"] == motor_id
                        and "current_control" in joint_config
                    ):
                        motor_config = joint_config["current_control"]
                        break
            if motor_config:
                params = create_control_params_from_config(motor_id, motor_config)
            else:
                params = ControlParams(motor_id=motor_id)
            params_list.append(params)
            if self.verbose:
                print(
                    f"  Leader motor {motor_id}: force_rendering="
                    f"{params.enable_force_rendering}, gain={params.force_rendering_gain}, "
                    f"alpha={params.force_rendering_alpha}, "
                    f"adaptive_gain={params.enable_force_rendering_adaptive_gain}"
                    f"(c={params.force_rendering_adaptive_velocity_coeff}), "
                    f"rate_limit={params.command_rate_limit_mA_per_s} mA/s, "
                    f"damping_gain={params.force_rendering_damping_gain}"
                )
        return params_list

    def _parse_force_mapping(self) -> tuple[dict[int, float], dict[int, float]]:
        torque_to_mA: dict[int, float] = {}
        torque_filter_alpha: dict[int, float] = {}
        mapping = self.config.hand_config.get("force_render")
        entries = mapping.get("joints") if mapping else None
        for motor in self.leader_chain:
            torque_to_mA[motor] = 1000.0
            torque_filter_alpha[motor] = 1.0
        if entries:
            for entry in entries:
                motor = int(entry["motor_id"])
                torque_to_mA[motor] = float(entry.get("torque_to_mA", 1000.0))
                torque_filter_alpha[motor] = float(
                    entry.get("torque_filter_alpha", 1.0)
                )
        return torque_to_mA, torque_filter_alpha

    def _parse_modality_filters(self) -> dict[tuple[int, str], float]:
        """Per (motor, modality) input EMA alpha applied before force blending."""
        fr_cfg = self.config.hand_config.get("force_render") or {}
        defaults = {
            str(k): float(v.get("torque_filter_alpha", 1.0))
            for k, v in (fr_cfg.get("modality_filters") or {}).items()
        }
        alpha: dict[tuple[int, str], float] = {}
        for motor in self.leader_chain:
            for modality in _VALID_FORCE_MODALITIES:
                alpha[(motor, modality)] = defaults.get(modality, 1.0)
        entries = fr_cfg.get("joints") or []
        for entry in entries:
            motor = int(entry["motor_id"])
            per_mod = entry.get("modality_filters") or {}
            for modality, cfg in per_mod.items():
                modality = str(modality)
                if modality not in _VALID_FORCE_MODALITIES:
                    continue
                if isinstance(cfg, (dict, DictConfig)):
                    alpha[(motor, modality)] = float(
                        cfg.get("torque_filter_alpha", alpha[(motor, modality)])
                    )
                else:
                    alpha[(motor, modality)] = float(cfg)
        return alpha

    def _get_u2d2_port_baud(self) -> tuple[str, int]:
        u2d2_config = self.config.u2d2
        return (
            u2d2_config.get("usb_port", "/dev/ttyUSB0"),
            int(u2d2_config.get("baudrate", 4000000)),
        )

    def connect(self) -> None:
        port, baudrate = self._get_u2d2_port_baud()
        use_fake = bool(self.config.u2d2.get("fake_u2d2", False))
        if self.verbose:
            print(f"Connecting Ditto leader (current mode) on {port} @ {baudrate}...")
        self.leader_hand = create_hand_interface(
            self.config, "leader", "real", port=port, baudrate=baudrate, use_fake=use_fake
        )
        self.leader_hand.connect()

        if self.sharpa_follower is not None:
            print("Connecting Sharpa follower hardware...")
            self.sharpa_follower.start(self.engine.sharpa_ik.joint_q_index)
            if self._needs_tactile:
                ok = self.sharpa_follower.enable_tactile(
                    calibrate=self.tactile_calibrate
                )
                if not ok:
                    tactile_fingers = [
                        f
                        for f, w in self._finger_force_weights.items()
                        if _finger_uses_modality(w, "tactile")
                    ]
                    raise RuntimeError(
                        f"tactile force source on {tactile_fingers} needs tactile "
                        "sensors, but the Sharpa device did not enable them."
                    )
        else:
            print("No Sharpa follower: leader current loop only (no force rendered).")

    def setup_motors(self) -> None:
        if self.leader_hand is None:
            raise RuntimeError("Leader not connected. Call connect() first.")
        self.leader_hand.setup_motors()

    def disconnect(self) -> None:
        if self.leader_hand is not None:
            self.leader_hand.disconnect()
            self.leader_hand = None
        if self.sharpa_follower is not None:
            self.sharpa_follower.stop()

    # ----- control loop ------------------------------------------------------

    def _ditto_q_from_leader(self, leader_motor_data: list[MotorData]) -> np.ndarray:
        """Full leader pin ``q`` (URDF convention) from leader motor reads."""
        joint_names = leader_joint_names_for_motor_ids(self.leader_chain)
        signs = dict(
            zip(joint_names, ditto_hardware_joint_signs(joint_names), strict=True)
        )
        urdf_angles = np.zeros(len(joint_names), dtype=float)
        for i, motor in enumerate(self.leader_chain):
            joint = _MOTOR_TO_JOINT[motor]
            jidx = joint_names.index(joint)
            urdf_angles[jidx] = signs[joint] * leader_motor_data[i].joint_angle
        return self.engine.ditto_q_from_actuated(urdf_angles, joint_names)

    def _apply_joint_map(self, ditto_q: np.ndarray) -> None:
        """Joint-position fingers: write direct leader→Sharpa joint targets into sharpa_q.

        The retargeter preserves seed joints for fingers it doesn't solve, so
        setting these before retarget(send=False) lets one send_q stream both the
        direct-mapped (joint) and IK (retarget) joints.
        """
        ditto_idx = self.engine.ditto_ik.joint_q_index
        sharpa_idx = self.engine.sharpa_ik.joint_q_index
        for finger in self._position_joint_fingers:
            for ditto_joint, sharpa_joint, scale in self._joint_map[finger]:
                angle = float(ditto_q[ditto_idx(ditto_joint)])
                self.engine.sharpa_q[sharpa_idx(sharpa_joint)] = scale * angle

    def _measured_torques_for_finger(self, finger: str) -> dict[str, float]:
        """Measured Sharpa joint torque → Ditto leader joint (Nm) for one finger."""
        if self.sharpa_follower is None or finger not in self._joint_map:
            return {}
        inputs = self.sharpa_follower.read_wrench_inputs(self.engine.sharpa_q)
        if inputs is None:
            return {}
        _, finger_torques = inputs
        taus = finger_torques.get(finger)
        if taus is None:
            return {}
        names = _SHARPA_FINGER_JOINTS[finger]
        torques: dict[str, float] = {}
        for ditto_joint, sharpa_joint, scale in self._joint_map[finger]:
            torques[ditto_joint] = scale * float(taus[names.index(sharpa_joint)])
        return torques

    def _task_space_torques_for_finger(
        self, finger: str, modality: str, ditto_q: np.ndarray
    ) -> dict[str, float]:
        """Task-space pad force → leader joint torques (Nm) for one finger/modality."""
        if self.sharpa_follower is None:
            return {}
        if modality == "estimate":
            source = self._estimate_source
        elif modality == "tactile":
            source = self._tactile_source
        else:
            return {}
        read = source.read(self.sharpa_follower, self.engine.sharpa_q)
        if read is None:
            return {}
        q_meas, force_by_finger = read
        force = force_by_finger.get(finger)
        if force is None:
            return {}
        fb = self.engine.retargeter.leader_force_and_torque(
            finger, force, q_meas, ditto_q
        )
        return {name: float(tau) for name, tau in zip(fb.joint_names, fb.joint_torques)}

    def _filter_modality_joint_torques(
        self, raw: dict[str, float], modality: str
    ) -> dict[str, float]:
        """Per-modality input EMA on leader joint torques (before weighted blend)."""
        filtered: dict[str, float] = {}
        for ditto_joint, tau in raw.items():
            motor = _JOINT_TO_MOTOR.get(ditto_joint)
            alpha = 1.0 if motor is None else self._modality_filter_alpha.get(
                (motor, modality), 1.0
            )
            key = (ditto_joint, modality)
            prev = self._filtered_torque_by_modality.get(key)
            tau_f = (
                tau
                if alpha >= 1.0 or prev is None
                else alpha * tau + (1.0 - alpha) * prev
            )
            self._filtered_torque_by_modality[key] = tau_f
            filtered[ditto_joint] = tau_f
        return filtered

    def _compute_cached_torques(self, ditto_q: np.ndarray) -> dict[str, float]:
        """Blend per-modality leader torques (Nm) after scaling and input filtering."""
        torques: dict[str, float] = {}
        for finger, weights in self._finger_force_weights.items():
            joint_blend: dict[str, float] = {}
            for modality, weight in weights.items():
                if weight <= 0.0:
                    continue
                if modality == "measured":
                    raw = self._measured_torques_for_finger(finger)
                elif modality in _TASK_SPACE_MODALITIES:
                    raw = self._task_space_torques_for_finger(finger, modality, ditto_q)
                else:
                    continue
                filtered = self._filter_modality_joint_torques(raw, modality)
                for joint, tau in filtered.items():
                    joint_blend[joint] = joint_blend.get(joint, 0.0) + weight * tau
            torques.update(joint_blend)
        return torques

    def _build_follower_currents(
        self, leader_motor_data: list[MotorData]
    ) -> tuple[list[MotorData], dict[str, list[float]]]:
        """Estimated joint torque (cached) → synthetic follower current (motor frame)."""
        follower: list[MotorData] = []
        tau_raw: list[float] = []
        tau_filt: list[float] = []
        synth_raw: list[float] = []
        synth_filt: list[float] = []

        for i, motor in enumerate(self.leader_chain):
            joint = _MOTOR_TO_JOINT[motor]
            sign = self._joint_sign[joint]
            tau = float(self._cached_torques.get(joint, 0.0))  # Nm, URDF convention

            alpha = self._torque_filter_alpha[motor]
            prev = self._filtered_torque_nm.get(motor)
            tau_f = tau if (alpha >= 1.0 or prev is None) else alpha * tau + (1.0 - alpha) * prev
            self._filtered_torque_nm[motor] = tau_f

            # Convert URDF-convention torque to the motor/hardware current frame,
            # then flip so the negative-feedback law renders the contact reaction.
            gain_mA = FORCE_RENDER_FEEDBACK_SIGN * self._torque_to_mA[motor]
            synth = sign * tau_f * gain_mA
            synth_r = sign * tau * gain_mA

            tau_raw.append(tau)
            tau_filt.append(tau_f)
            synth_raw.append(synth_r)
            synth_filt.append(synth)
            follower.append(
                MotorData(
                    motor_id=motor,
                    raw_position=0,
                    raw_velocity=0,
                    raw_current=0,
                    position=0.0,
                    velocity=leader_motor_data[i].joint_velocity,
                    current=synth,
                    joint_angle=0.0,
                    joint_velocity=leader_motor_data[i].joint_velocity,
                )
            )
        series = {
            "tau_raw": tau_raw,
            "tau_filt": tau_filt,
            "synth_raw": synth_raw,
            "synth_filt": synth_filt,
        }
        return follower, series

    def _apply_command_rate_limit(self, commands: list[float]) -> list[float]:
        """Cap rising |I| on the net leader current; drops pass through immediately.

        Smooths contact onset (force magnitude ramping up) while keeping release
        responsive when tactile force falls or contact breaks.
        """
        limited: list[float] = []
        for i, motor in enumerate(self.leader_chain):
            cmd = float(commands[i])
            rate = self.leader_control_params[i].command_rate_limit_mA_per_s
            if rate > 0.0:
                prev = self._rate_limited_command_mA.get(motor)
                if prev is not None:
                    step = rate * self.dt
                    prev_abs = abs(prev)
                    cmd_abs = abs(cmd)
                    if cmd_abs > prev_abs:
                        cmd = float(np.copysign(min(cmd_abs, prev_abs + step), cmd))
                self._rate_limited_command_mA[motor] = cmd
            limited.append(cmd)
        return limited

    def _queue_states(
        self,
        leader_motor_data: list[MotorData],
        follower_motor_data: list[MotorData],
        current_commands: list[float],
        series: dict[str, list[float]],
        force_rendering_damping_currents: list[float],
        force_rendering_currents: list[float],
    ) -> None:
        if self.state_queue is None:
            return
        states = {
            "leader": {
                "currents": [d.current for d in leader_motor_data],
                "command_currents": list(current_commands),
                "force_rendering_currents": list(force_rendering_currents),
                "force_rendering_damping_currents": list(force_rendering_damping_currents),
                "joint_velocities": [d.joint_velocity for d in leader_motor_data],
                "joint_angles": [d.joint_angle for d in leader_motor_data],
            },
            "follower": {
                "currents": series["synth_filt"],
                "currents_raw": series["synth_raw"],
                "torques_nm": series["tau_raw"],
                "torques_nm_filtered": series["tau_filt"],
                "joint_velocities": [d.joint_velocity for d in follower_motor_data],
                "joint_angles": [d.joint_angle for d in follower_motor_data],
            },
        }
        try:
            self.state_queue.put_nowait(states)
        except queue.Full:
            pass

    def _control_step(self) -> None:
        # Fast path only: read leader, render currents from the latest cached
        # torques, write currents. All blocking / CPU-heavy work (IK retarget,
        # Sharpa send/read, tactile fetch, force estimate) runs on the worker
        # thread so this loop stays tight and consistent.
        leader_motor_data = self.leader_hand.read_states()
        if not leader_motor_data:
            return

        # Hand the freshest leader read to the retarget worker.
        self._latest_leader_data = leader_motor_data

        follower_motor_data, series = self._build_follower_currents(leader_motor_data)
        current_commands = self.current_controller.compute_bulk_current_commands(
            leader_motor_data,
            self.leader_control_params,
            time.time(),
            follower_motor_data,
            self.show_current_breakdown,
        )
        current_commands = self._apply_command_rate_limit(current_commands)
        damping_cmds = self.current_controller.last_force_rendering_damping_cmds
        force_cmds = self.current_controller.last_force_rendering_cmds
        self._queue_states(
            leader_motor_data,
            follower_motor_data,
            current_commands,
            series,
            damping_cmds,
            force_cmds,
        )
        self.leader_hand.send_current_commands(
            [int(round(c)) for c in current_commands]
        )

    def _retarget_worker(self) -> None:
        """Background loop: Sharpa force sampling (fast) + IK retarget (slow).

        The *only* thread that touches the Sharpa follower and engine kinematics,
        so the high-rate current loop never blocks on IK or Sharpa/tactile I/O.
        Two cadences share this single thread (keeps SDK access serialized):

        - every cycle (``force_hz``): read Sharpa state + tactile, estimate force
          → publish ``self._cached_torques``. This is the haptic signal; sampling
          it fast keeps the rendered force fresh and reactive.
        - every ``retarget_hz``: solve the expensive IK and stream q to Sharpa.
          This only needs to keep the follower tracking, not be reactive.
        """
        last_retarget = 0.0
        # Rolling per-stage timing so we tune from data, not assumptions.
        w_n = 0
        w_ik = 0.0
        w_est = 0.0
        w_period = 0.0
        w_last_start: float | None = None
        report_every = max(1, int(1.0 / self.force_period))  # ~1 s
        while self.running:
            cycle_start = time.perf_counter()
            if w_last_start is not None:
                w_period += cycle_start - w_last_start
            w_last_start = cycle_start

            leader_motor_data = self._latest_leader_data
            if leader_motor_data is not None:
                ditto_q = self._ditto_q_from_leader(leader_motor_data)
                self.engine.ditto_q = ditto_q

                # --- Position ---
                # Joint-position fingers: direct joint map every cycle (cheap).
                if self._position_joint_fingers:
                    self._apply_joint_map(ditto_q)
                # Retarget-position fingers: expensive IK, throttled.
                now = time.perf_counter()
                if (
                    self._position_retarget_fingers
                    and now - last_retarget >= self.retarget_period
                ):
                    last_retarget = now
                    self.engine.retarget(solve_params=IK_STREAM, send=False)
                    w_ik += time.perf_counter() - now

                # One send streams both direct (joint) and IK (retarget) joints.
                if self.sharpa_follower is not None:
                    self.sharpa_follower.send_q(self.engine.sharpa_q)

                # --- Force --- (per-modality filter → weighted blend in Nm)
                if self.sharpa_follower is not None:
                    t_est = time.perf_counter()
                    self._cached_torques = self._compute_cached_torques(ditto_q)
                    w_est += time.perf_counter() - t_est

            w_n += 1
            if w_n >= report_every:
                self._print_worker_perf(w_n, w_period, w_ik, w_est)
                w_n = 0
                w_ik = w_est = w_period = 0.0
                w_last_start = None

            # Plain sleep (not precise_sleep): the worker's timing isn't critical
            # and time.sleep fully releases the GIL, handing the leader loop an
            # uncontended core for the whole idle window.
            time.sleep(
                max(0.0, self.force_period - (time.perf_counter() - cycle_start))
            )

    def _print_worker_perf(
        self, n: int, period_sum: float, ik_sum: float, est_sum: float
    ) -> None:
        """Stash worker perf for the control loop to print (single stdout writer).

        Printing from the worker thread holds the GIL during the stdout flush and
        stalls the leader serial read — a per-second loop spike. So the worker
        only formats the line here; the control loop emits it.
        """
        actual_hz = (n - 1) / period_sum if period_sum > 0 else float("inf")
        n_ik = max(1, int(round(self.force_period / self.retarget_period * n)))
        self._worker_perf_line = (
            f"[perf:worker] {actual_hz:6.1f} Hz | force/tactile {est_sum / n * 1000:5.2f} ms/cycle "
            f"| IK {ik_sum / n_ik * 1000:5.2f} ms/solve "
            f"(target {1.0 / self.force_period:.0f} Hz)"
        )

    def _describe_finger(self, finger: str) -> str:
        """Human-readable position + force description for one finger."""
        mode = self._finger_modes[finger]
        position = "retarget IK" if mode["position"] == "retarget" else "direct joint map"
        weights = self._finger_force_weights[finger]
        parts = [
            f"{modality} {weight:.0%}"
            for modality, weight in sorted(weights.items())
        ]
        force_desc = "blend: " + ", ".join(parts)
        return f"position: {position:<16s} force: {force_desc}"

    def _print_mode_summary(self) -> None:
        """Print the per-finger control mode at startup (what the user is running)."""
        bar = "=" * 64
        print(f"\n{bar}")
        print(f"Retargeting force-render teleop @ {self.control_frequency:.0f} Hz")
        print(
            f"  loops: leader/render {self.control_frequency:.0f} Hz | "
            f"force/tactile {1.0 / self.force_period:.0f} Hz | "
            f"IK→Sharpa {1.0 / self.retarget_period:.0f} Hz (worker)"
        )
        print(f"  leader motors (current): {self.leader_chain}")
        if self.sharpa_follower is None:
            print("  Sharpa follower: OFF (leader current loop only, no force rendered)")
        print("  Per-finger control mode (hand_config.control.fingers):")
        for finger in self._finger_modes:
            print(f"    {finger:<6s} {self._describe_finger(finger)}")
        print(f"{bar}\n")

    def start_control_loop(self) -> None:
        if self.leader_hand is None:
            raise RuntimeError("Not connected. Call connect() first.")
        self._print_mode_summary()
        # The worker (pinocchio FK/IK + numpy) holds the GIL in ~ms bursts. The
        # default 5 ms GIL switch interval lets one such burst stall the leader
        # loop for a whole cycle. Drop it to 1 ms so the leader loop reclaims the
        # GIL mid-burst and stays close to target rate.
        self._prev_switch_interval = sys.getswitchinterval()
        sys.setswitchinterval(0.001)
        self.running = True
        self._retarget_thread = threading.Thread(
            target=self._retarget_worker, daemon=True
        )
        self._retarget_thread.start()
        # Rolling window stats over ~1 s for the periodic performance summary.
        win_n = 0
        win_work = 0.0       # sum of compute time (excl. sleep)
        win_period = 0.0     # sum of realized cycle period (incl. sleep)
        win_work_max = 0.0
        last_cycle_start: float | None = None
        cycle = 0
        try:
            while self.running:
                cycle_start = time.perf_counter()
                if last_cycle_start is not None:
                    win_period += cycle_start - last_cycle_start
                last_cycle_start = cycle_start

                self._control_step()
                work = time.perf_counter() - cycle_start

                win_n += 1
                win_work += work
                win_work_max = max(win_work_max, work)
                # Compute cost exceeds the budget → can't hold target rate.
                if work > self.dt:
                    self._print_perf_warning(work, cycle)

                # Periodic summary roughly once per second.
                if win_n >= self.control_frequency:
                    if self.show_perf:
                        self._print_perf_summary(
                            win_n, win_work, win_period, win_work_max
                        )
                    win_n = 0
                    win_work = win_period = win_work_max = 0.0

                cycle += 1
                precise_sleep(max(0.0, self.dt - work))
        except KeyboardInterrupt:
            print("\nStopping force rendering...")
        finally:
            self.running = False
            if self._retarget_thread is not None:
                self._retarget_thread.join(timeout=1.0)
                self._retarget_thread = None
            sys.setswitchinterval(self._prev_switch_interval)

    def _print_perf_summary(
        self, n: int, work_sum: float, period_sum: float, work_max: float
    ) -> None:
        """Print achieved loop rate over the last window (target vs actual)."""
        work_avg = work_sum / n
        # Realized rate needs n-1 inter-cycle gaps; fall back to work if missing.
        actual_hz = (n - 1) / period_sum if period_sum > 0 else 1.0 / work_avg
        max_hz = 1.0 / work_max if work_max > 0 else float("inf")
        # Emit the worker line (stashed by the worker thread) together with ours
        # in a single write, so all perf logging comes from this one thread.
        worker_line = self._worker_perf_line
        prefix = f"{worker_line}\n" if worker_line else ""
        print(
            f"{prefix}"
            f"[perf] {actual_hz:6.1f} Hz actual (target {self.control_frequency:.0f}) | "
            f"compute {work_avg * 1000:5.2f} ms avg, {work_max * 1000:5.2f} ms max "
            f"(headroom to {max_hz:.0f} Hz)"
        )

    def _print_perf_warning(self, work: float, cycle: int) -> None:
        """Warn (throttled) when compute time alone overruns the cycle budget."""
        interval = int(self.control_frequency) if self.control_frequency >= 1 else 1
        if cycle % interval != 0:
            return
        red, reset = "\033[91m", "\033[0m"
        print(
            f"{red}[perf] CONTROL LOOP TOO SLOW: compute {work * 1000:.2f} ms > "
            f"budget {self.dt * 1000:.2f} ms (max {1.0 / work:.0f} Hz < "
            f"target {self.control_frequency:.0f} Hz){reset}"
        )

    def stop(self) -> None:
        self.running = False
