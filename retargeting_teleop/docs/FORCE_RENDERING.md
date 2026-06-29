# Force rendering (simplified)

How a follower contact becomes a leader motor current. Everything is per finger,
per leader joint. Simplifying assumptions used here: **always gradual rendering**,
a **single torque deadband** $\delta$ (no separate +/- thresholds), and **no
velocity threshold** on the damping.

## 1. Getting the leader joint torque $\tau$

The rendering law takes an estimated **leader joint torque** $\tau$ (Nm). It comes
from one of two paths:

**Joint-level** (1:1 joint correspondence, e.g. index): the measured Sharpa joint
torque mapped straight back,

$$\tau = s\,\tau_s^{\text{meas}}$$

**Task-space** (retargeting): go through the fingertip contact force $F$.

- Force from Sharpa joint torque (estimate): $\;F = J_s^{-T}(q_s)\,\tau_s$
  (use the pseudo-inverse $\big(J_s^{T}\big)^{+}$ if not square)
- Force from the fingertip tactile sensor: $\;F$ read directly (rotated to base)

then map the force onto the leader joints,

$$\tau = J_d^{T}(q_d)\,F$$

where $J_s, J_d$ are the Sharpa / Ditto finger Jacobians and $F$ is expressed in the
shared base frame.

Optionally low-pass the torque before rendering ($1$ = off):

$$\tau \leftarrow \alpha_\tau\,\tau + (1-\alpha_\tau)\,\tau^{\text{prev}}$$

## 2. Force rendering

Gradual rendering with a single deadband, output low-pass filter, and clamp:

$$
I_{\text{fr}} =
\begin{cases}
0, & |\tau| \le \delta \\[4pt]
k\,\operatorname{sign}(\tau)\,\big(|\tau| - \delta\big), & |\tau| > \delta
\end{cases}
$$

$$
I_{\text{fr}} \leftarrow \alpha\,I_{\text{fr}} + (1-\alpha)\,I_{\text{fr}}^{\text{prev}},
\qquad
|I_{\text{fr}}| \le I_{\max}
$$

The current ramps up from zero at the threshold (gradual) rather than jumping.
`sign(τ)` with positive $k$ renders the contact reaction (resists the user into
contact); flip the sign of $k$ if the felt direction is inverted.

## 3. Force-rendering damping

Active only while in contact (past the same deadband). With no velocity threshold
it is plain linear damping that bleeds off leader velocity $\dot\theta$:

$$
I_{\text{damp}} =
\begin{cases}
0, & |\tau| \le \delta \\[4pt]
-k_d\,\dot\theta, & |\tau| > \delta
\end{cases}
\qquad |I_{\text{damp}}| \le I_{d,\max}
$$

## 4. Net leader current

$$I_{\text{cmd}} = I_{\text{fr}} + I_{\text{damp}}$$

## Parameters

| Symbol | Config key | Meaning |
|---|---|---|
| $k$ | `force_rendering_gain` | torque→current gain (mA/Nm) |
| $\delta$ | `force_rendering_threshold` | torque deadband (Nm) |
| $\alpha$ | `force_rendering_alpha` | output low-pass ($1$ = off) |
| $I_{\max}$ | `force_rendering_max_current` | render clamp (mA) |
| $k_d$ | `force_rendering_damping_gain` | damping gain (mA per rad/s) |
| $I_{d,\max}$ | `force_rendering_damping_max_current` | damping clamp (mA) |
| $\alpha_\tau$ | `torque_filter_alpha` | input torque low-pass ($1$ = off) |
