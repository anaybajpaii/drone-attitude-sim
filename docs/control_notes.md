# Control Notes

## Overview

This simulator models 3-axis (roll, pitch, yaw) drone attitude control using an independent PID controller per axis. Each axis has a constant disturbance torque causing it to drift from level flight; three PID controllers compute corrective torques, which a motor mixer converts into four individual motor thrust values. This document covers the simulation model, the control theory behind it, the mixing equations, the tuning results, and the limitations of the current approach.

## 1. Simulation Model

Each axis is modeled as a simple rotating system under a constant disturbance (a fixed angular acceleration), plus the PID correction once controllers are active:

$$\dot\omega = d + c$$

$$\omega(t+\Delta t) = \omega(t) + \dot\omega\,\Delta t$$

$$\theta(t+\Delta t) = \theta(t) + \omega(t+\Delta t)\,\Delta t$$

where $d$ is the fixed disturbance constant for that axis and $c$ is the PID correction (zero in the uncontrolled case). This is explicit Euler integration, the same approach used in `stabilization-sim`: the angular velocity is updated first using the current acceleration, and the angle is then updated using that new velocity, rather than the old one.

The three axes are currently decoupled — roll, pitch, and yaw dynamics don't interact with each other. Real drone attitude dynamics are coupled (e.g. gyroscopic effects between axes), so this is a simplification appropriate for a controls-focused simulator rather than a full flight-dynamics model.

## 2. From Proportional-Only to Full PID

The predecessor project, `stabilization-sim`, used a pure proportional controller on a single-axis pendulum — the correction was just the gain times the current error:

$$u = K_p\,e$$

Its key finding was that P-only control can prevent a system from falling, but cannot settle it to rest — once $K_p$ crosses the threshold needed to overcome the destabilizing force, the system oscillates indefinitely, since nothing in the control law dissipates energy from the oscillation.

This project uses the full PID law instead, adding two more terms on top of the proportional one:

$$u = K_p\,e + K_i\sum e\,\Delta t + K_d\,\frac{e_{now} - e_{prev}}{\Delta t}$$

- **Proportional (P)**, $K_p\,e$ — reacts to the current error. This is the only term `stabilization-sim` had.
- **Integral (I)**, $K_i\sum e\,\Delta t$ — keeps a running total of past error over time, and pushes the correction harder the longer an offset persists. This removes the small leftover steady-state error that P-only control can't fully correct.
- **Derivative (D)**, $K_d\,\frac{e_{now}-e_{prev}}{\Delta t}$ — reacts to how fast the error is changing, not just its size. This is the damping term P-only control was missing — it resists fast motion and is what breaks the overshoot-oscillate cycle seen in `stabilization-sim`.

Derivative-on-error (rather than derivative-on-measurement) was used for simplicity, since the setpoint here never changes — it's always 0 degrees, level flight. The distinction between the two mainly matters when the setpoint itself is changing often, which doesn't happen in this simulator.

## 3. Motor Mixing

The three axis corrections, plus a base throttle $T$, are converted into four individual motor thrust values using standard X-configuration quadcopter mixing, with motor order front-left (0), front-right (1), rear-left (2), rear-right (3):

$$m_0 = T + c_{roll} + c_{pitch} - c_{yaw}$$

$$m_1 = T - c_{roll} + c_{pitch} + c_{yaw}$$

$$m_2 = T + c_{roll} - c_{pitch} + c_{yaw}$$

$$m_3 = T - c_{roll} - c_{pitch} - c_{yaw}$$

This reflects how a real X-frame quadcopter achieves attitude changes: diagonal motor pairs are sped up or slowed down together to roll, pitch, or yaw the frame, rather than any single motor acting alone.

The current implementation computes these values but does not yet clamp them to a realistic thrust range (e.g. 0–100%) — see Limitations below.

## 4. Gain Comparison Results

To directly compare against the `stabilization-sim` finding, the simulator was run from a 15-degree initial roll disturbance across three values of the derivative gain ($K_d$), holding $K_p = 2.0$ and $K_i = 0.1$ fixed:

| Scenario | $K_d$ | Behavior |
|---|---|---|
| Underdamped | 0.1 | Overshoots past 0°, oscillates several times before settling — reproduces the `stabilization-sim` pattern |
| Baseline | 0.5 | Small single overshoot, settles quickly |
| Overdamped | 1.5 | No overshoot, but slower to fully settle |

See `data/scenario_comparison.png` / `docs/screenshots/scenario_comparison.png`.

This is the direct, visual confirmation that the derivative term is what separates this project's behavior from `stabilization-sim`'s: with $K_d$ too low, the system still oscillates like the pure-P pendulum did; with $K_d$ in a reasonable range, it settles.

## 5. Design Decisions

- **Explicit Euler integration** — consistent with `stabilization-sim`, and accurate enough at the 0.1s timestep used here given how simple the dynamics being modeled are.
- **Derivative-on-error** — simpler to implement than derivative-on-measurement, and appropriate since the setpoint never changes in this simulator.
- **Decoupled axes** — roll, pitch, and yaw are simulated independently; no cross-axis coupling or gyroscopic effects are modeled.
- **Shared default gains across axes** — all three axes default to the same P, I, D values for simplicity, though the CLI interface allows setting them independently per axis.
- **Fixed-order positional CLI arguments** over a config file — simpler for a project at this scale; a config file parser would add complexity without real benefit here.

## 6. Limitations

- Motor thrust outputs from the mixer are not clamped to a physically realistic range, so at large corrections the values can exceed what a real motor could produce.
- No sensor noise is modeled — the controller reads exact state values every step, unlike a real flight controller reading noisy gyroscope/accelerometer data.
- No actuator delay is modeled — real motors take time to spin up or down in response to a command change.
- Integral windup is not addressed; a sustained large error could cause the integral term to accumulate without bound.
- The three axes are dynamically decoupled, which is not physically accurate for a real quadcopter.
