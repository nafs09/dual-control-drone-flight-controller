# Flight Controller — Control Architecture & Process Reference

> **This document is standalone.** It is not connected to any source file.
> It explains the *why* and *how* of the control architecture, including
> the mathematics, physical intuition, and tuning procedure.
> Read this before touching any PID gain.

---

## 1. Why Cascaded Control?

A quadrotor has one actuator group (4 motors) and needs to control 6 degrees
of freedom (position xyz, orientation roll/pitch/yaw). It can only directly
control attitude through differential motor thrust — it cannot control position
directly. Position control is *achieved* by tilting the vehicle.

This creates a natural hierarchy:

```
What you want    → Where to be      (position,    metres)
How to get there → How fast to move (velocity,    m/s)
How to move fast → Which way to tilt(attitude,    radians)
How to tilt      → How fast to tilt (angular rate,rad/s)
How fast to tilt → Motor speeds     (DSHOT value, 0-2047)
```

Each level has a different bandwidth (how fast it can respond) and a different
physical meaning. Separating them makes the system:

- **Tunable**: each loop can be tuned independently, from inside out
- **Stable**: inner loops are always faster than outer loops
- **Understandable**: a problem in one loop doesn't mask another
- **Extensible**: you can disable outer loops without rewriting inner ones

This is standard aerospace practice from autopilot design. The same architecture
appears in commercial flight controllers (PX4, ArduCopter) and certified
aviation autopilots.

---

## 2. Full Loop Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                     CAMERA MODE (all loops)                        │
├────────────┬────────────┬────────────┬────────────┬────────────────┤
│ Loop       │ Rate       │ Input      │ Output     │ Physical unit  │
├────────────┼────────────┼────────────┼────────────┼────────────────┤
│ Position   │ 50 Hz      │ pos error  │ vel sp     │ m → m/s        │
│ Velocity   │ 100 Hz     │ vel error  │ tilt angle │ m/s → rad      │
│ Attitude   │ 3840 Hz    │ angle error│ rate sp    │ rad → rad/s    │
│ Rate       │ 3840 Hz    │ rate error │ torque     │ rad/s → [-1,1] │
│ Mixer      │ 3840 Hz    │ torque     │ DSHOT      │ [-1,1] → 0-2047│
└────────────┴────────────┴────────────┴────────────┴────────────────┘

FPV MODE: Position → Velocity → Attitude loops all bypassed.
          RC sticks feed directly to Rate loop as rate setpoints.
```

### Bandwidth Rule

Each outer loop must have bandwidth at least 3-5× lower than the inner loop
it commands. Violating this causes the loops to fight each other.

```
Rate loop:     ~100 Hz closed-loop bandwidth
Attitude loop: ~20-30 Hz (≥5× below rate loop)
Velocity loop: ~3-5 Hz  (≥5× below attitude)
Position loop: ~0.5-1 Hz (≥5× below velocity)
```

The *execution rates* (3840 Hz, 100 Hz, 50 Hz) are much higher than the
*bandwidths* — you need many samples per oscillation period to control it.

---

## 3. Rate Controller (innermost)

**Runs at:** 3840 Hz (every IMU sample)
**In both FPV and Camera mode.**

**Input:**  desired angular rate [rad/s], body frame, per axis
**Output:** normalised torque [-1..+1], per axis

Three independent PIDs (roll, pitch, yaw), one per body axis.

### Why gyro rate, not angle?

In FPV mode, controlling angle is unnecessary — the pilot provides angle
implicitly through stick position. Rate control gives the pilot direct,
proportional feel. A push of 50% on the stick gives exactly 50% of maximum
rotation rate, regardless of current attitude. This is the basis of
"full acro" flying.

### The D term

The D term (derivative) on the rate loop acts as **gyro damping**. Without it,
the drone oscillates: it overshoots the commanded rate, the P term then
pushes back, it overshoots the other way, and so on.

The D term sees: `(rate_error_now - rate_error_last) / dt`

This is mathematically equivalent to the second derivative of angle. On a drone
it damps the prop wash and frame resonance that would otherwise make the D
term fight you. A first-order low-pass filter on the D term (tau_d ≈ 2 ms at
3840 Hz) cuts high-frequency noise while preserving the damping effect.

### Anti-windup

The integral term accumulates error over time to remove steady-state offset.
Without anti-windup, if the motor saturates (full throttle applied, rate still
not achieved), the integrator keeps winding up. When the saturation ends,
there is a large stored integral that suddenly applies a huge correction.

The clamping anti-windup used here:
```
if (output_with_integral > max) AND (integral_increment > 0):
    do not update integrator     ← freeze while saturating upward
```
This bounds the integrator to [−i_max, +i_max] and prevents the spike.

---

## 4. Attitude Controller (middle loop, camera mode only)

**Runs at:** 3840 Hz
**Camera mode only.** Bypassed in FPV.

**Input:**  desired attitude quaternion q_desired, current attitude q_current
**Output:** angular rate setpoint [rad/s], body frame

### Quaternion error

The error quaternion is computed as:
```
q_error = q_desired ⊗ q_current_inverse
```

This gives a quaternion whose vector part (x, y, z) encodes the rotation axis
and whose scalar (w) encodes how close we are to the goal (w = 1 = perfect).

For small errors, `q_error.xyz ≈ δθ/2` (half the angle error vector).

### Control law

```
rate_setpoint = 2 × Kp × q_error.xyz
```

The factor of 2 converts from "half-angle" to full angle. This is a
pure-proportional attitude law. No integral is needed here because:
- Steady-state attitude errors are removed by the velocity controller's integral
- Adding integral to the attitude loop risks dual-integrator instability

### Why quaternion, not Euler angles?

Euler angles have a **gimbal lock** singularity at pitch = ±90°. At that point,
the roll and yaw axes become coincident and control breaks down. Quaternions
represent all orientations continuously without singularity.

The quaternion error law also gives geometrically correct interpolation — the
shortest rotation path between two orientations — which Euler-based controllers
do not guarantee.

---

## 5. Velocity Controller (new loop, camera mode only)

**Runs at:** 100 Hz (every 38 IMU ticks)
**Camera mode only.** Bypassed in FPV.

**Input:**  desired NED velocity [m/s], measured NED velocity [m/s] from EKF
**Output:** desired body-frame tilt [roll, pitch, rad]

This is the key loop for wind rejection and cinematic flight.

### Physical model

A hovering quadrotor tilted at angle θ from vertical produces:
- Vertical force: F_z = T·cos(θ) ≈ T  (for small θ)
- Horizontal force: F_h = T·sin(θ) ≈ T·θ  (small angle)

The horizontal equation of motion is:
```
m × a_h = T × θ  →  a_h ≈ g × θ   (at hover where T ≈ m×g)
```

Inverting: `θ_desired = a_desired / g`

The velocity PID outputs a desired horizontal acceleration. Dividing by g
gives the required tilt angle.

### Coordinate conversion

The PID operates in NED (North-East-Down) because the EKF provides velocity
in NED. The attitude controller needs tilt in body frame. The conversion uses
current yaw ψ:

```
a_fwd   =  a_N × cos(ψ) + a_E × sin(ψ)     ← forward component in body X
a_right = -a_N × sin(ψ) + a_E × cos(ψ)     ← rightward component in body Y

pitch_body = -a_fwd   / g    (negative: forward accel requires nose-DOWN)
roll_body  =  a_right / g    (positive: rightward accel requires lean-RIGHT)
```

Sign check (drone facing North, ψ=0, want to go North):
- a_N > 0, a_E = 0
- a_fwd = a_N > 0, a_right = 0
- pitch_body = -a_N/g < 0  (nose down, lean forward) ✓

Sign check (drone facing East, ψ=π/2, want to go North):
- a_N > 0, a_E = 0
- a_fwd = 0, a_right = -a_N < 0
- roll_body = -a_N/g < 0  (lean left, which is North-ward when facing East) ✓

### Stick-to-velocity mapping (no GPS hold)

When GPS position hold is not active, the RC sticks map directly to a velocity
setpoint in the body-relative frame:

```
v_forward = -pitch_stick × V_MAX    (push stick forward → positive forward vel)
v_right   =  roll_stick  × V_MAX    (push stick right   → positive rightward vel)
```

Then rotated to NED using yaw:
```
v_N = v_forward × cos(ψ) - v_right × sin(ψ)
v_E = v_forward × sin(ψ) + v_right × cos(ψ)
```

**This is "velocity mode"** — centring the stick commands zero velocity, so the
drone will decelerate and hold position (approximately, without GPS). This is
very different from angle mode where centring the stick levels the drone but
it continues to slide until drag stops it.

Velocity mode is far more cinematic because:
1. Stopping is smooth: the drone decelerates rather than snapping level
2. Speed is predictable: 50% stick = 50% of V_MAX, always
3. Wind doesn't cause drift when stick is centred

---

## 6. Wind Rejection — The Key Mechanism

This is the most important thing the velocity controller does.

### What happens without it

In angle-only mode, the outer position loop commands a tilt. If there is a
10 km/h headwind, the drone drifts. The position error grows. The position PID
increases the tilt. Eventually it finds an equilibrium tilt that balances the
wind drag. This works but is slow (limited by position loop bandwidth ~0.5 Hz)
and produces a position error oscillation visible in GPS traces.

### What happens with the velocity controller integral

The velocity controller adds an **integral term** (Ki). When there is a
persistent velocity error (wind pushing the drone off target), the integral:

```
integral += Ki × velocity_error × dt
```

This accumulates over time. After a few seconds, it has built up enough to
command a permanent tilt into the wind, exactly cancelling the aerodynamic drag.

At steady state (constant wind, constant position):
- velocity_error = 0  (drone is stationary)
- integral = wind_compensation_tilt  (non-zero, holding the lean)
- P term = 0
- Total output = integral term only

This is physically the correct answer: the drone must permanently lean into the
wind to hover in it. The integral finds this tilt automatically.

### Time constant of rejection

With Ki = 0.3 and a step wind input of 5 m/s:
- Initial P response: 1.5 × 5 = 7.5 m/s² → ~0.76 rad tilt (43°) — immediately clamps to 30°
- Integral builds at ~Ki × vel_error = 0.3 × 5 = 1.5 m/s² per second
- Steady state (drone at 5 m/s drift): integral needs ~7.5/0.3 = ~25 seconds
- With integrator anti-windup (i_max = 5 m/s²): clamped at ~3 seconds

In practice, the P term dominates the initial response (drone leans into the
wind immediately) and the I term builds the steady-state hold. The drone
re-acquires position after a wind gust in approximately 2-5 seconds.

### Anti-windup in the wind context

If the drone is flying at maximum speed (velocity setpoint = V_MAX and it's
moving at V_MAX), the integrator should not continue winding up — the output
is already saturated. The clamping anti-windup prevents the integrator from
accumulating beyond what can be produced by the actuators.

---

## 7. Vertical Velocity Controller

The vertical channel is a separate PID inside the VelocityController:

```
Input:  vel_sp.z (NED down, positive = downward) vs vel_ned.z from EKF
Output: thrust_delta — added to the base hover throttle
```

The base hover throttle is approximately 0.5 (50%) but varies with battery
voltage, payload, and temperature. The vertical PID trims around this.

Setting vel_sp.z from sticks:
```
vel_sp.z = -(throttle_stick × 2 - 1) × V_VERT_MAX
```
Neutral stick (0.5) → vel_sp.z = 0 → hold altitude
Full up stick (1.0) → vel_sp.z = -V_VERT_MAX → climb
Full down (0.0) → vel_sp.z = +V_VERT_MAX → descend

The negative sign is because NED defines down as positive, but operators
expect up-stick to mean go up.

---

## 8. Position Controller (outermost, GPS hold only)

**Runs at:** 50 Hz
**Camera mode with GPS hold active only.**

**Input:**  position setpoint [m, NED], current EKF position [m, NED]
**Output:** velocity setpoint [m/s, NED] → fed into velocity controller

The position controller is simply a P (or PID) on the position error that
outputs a commanded velocity. It does not directly command tilt — it tells
the velocity controller how fast to move.

This two-stage separation (position → velocity → tilt) gives much smoother
behaviour near the setpoint:
- Far away: large position error → large velocity setpoint → drone moves fast
- Close: small position error → small velocity setpoint → drone brakes smoothly
- At target: zero error → zero velocity setpoint → velocity controller holds still

Without this two-stage approach, a pure position-to-tilt controller would
oscillate around the setpoint (drone overshoots because it's going too fast
when it arrives).

---

## 9. Mode Transition Behaviour

### FPV → Camera

1. Rate PID gains swapped to camera preset (lower Kp/Kd)
2. Desired yaw **captured** from current attitude — no yaw snap
3. Velocity setpoint initialised to current EKF velocity — no step input
4. Velocity controller **reset** (integrators zeroed) — no carry-over
5. Attitude controller desired heading set to current heading

The integrator is reset on entry to camera mode (not FPV→camera gain swap)
because the velocity integral from FPV flight is meaningless in position hold.
The rate PID integrators are **not** reset because they contain useful
information about trim and gyro bias.

### Camera → FPV

1. Rate PID gains swapped to FPV preset (higher Kp/Kd)
2. Velocity and attitude controllers are bypassed immediately
3. Sticks now command rate directly

The transition is instantaneous from the control perspective. The physical
transition feels like the drone suddenly becomes more responsive.

### Failsafe

1. Velocity setpoint forced to {0, 0, +0.5} — hover and descend at 0.5 m/s
2. Camera-mode cascade remains active (uses last attitude estimate)
3. Velocity controller integral remains active to counteract wind
4. Land detection: vertical velocity < 0.1 m/s AND baro AGL < 0.3 m → disarm

---

## 10. EKF Integration

The velocity controller depends on good velocity estimates from the EKF.
Without good velocity measurements it degrades to approximate behaviour:

| Sensors available | Velocity quality | Controller behaviour |
|---|---|---|
| GPS + baro + mag | Excellent (±0.1 m/s) | Full wind rejection, tight hold |
| Baro + mag only  | Good (drift over time)| Short-term rejection, slow drift |
| IMU only (Mahony)| Poor (integrated noise)| Very approximate, not reliable |

The EKF's 15-state error-state design specifically provides accurate velocity
estimates even during GPS dropouts by propagating IMU measurements through the
dynamic model. The gyro and accelerometer bias states ensure that velocity
integration doesn't drift from DC offsets.

---

## 11. Tuning Procedure

Tune from the inside out. **Never touch an outer loop until the inner loop
is solid.** Each step assumes the previous step is complete.

### Step 1: Rate loop (FPV mode)

Fly in FPV mode. Ignore everything else.

1. Set all Ki = 0, Kd = 0.
2. Increase Kp until the drone oscillates on quick stick inputs. Then halve it.
3. Add Kd in small steps until oscillation stops. Too much Kd = high-frequency buzz.
4. Add Ki in small steps until slow drifts (wind, trim) are corrected.
   Watch for low-frequency oscillation (integrator windup sign).
5. Rate loop Kp is typically 0.03–0.08. Kd is typically 10-30% of Kp.

### Step 2: Attitude loop (camera mode, hover)

With rate loop tuned, switch to camera mode.

1. Set attitude Kp = 3.0 (conservative).
2. Command small roll and pitch angles by tilting the transmitter slightly.
3. Increase Kp until tracking is tight. Watch for oscillation between the
   attitude and rate loops.
4. Typical Kp range: 5–12. Stop before you see coupling oscillation.

### Step 3: Velocity loop (camera mode, light wind)

With attitude loop tuned, enable velocity mode (camera mode, sticks).

1. Set Ki = 0. Set Kp = 0.5.
2. Give a short stick input and release. Drone should decelerate and hold.
3. Increase Kp until the deceleration response feels crisp. Watch for overshoot
   (drone decelerates but keeps oscillating back and forth).
4. With Kp set, add Ki in small steps. Fly in a gentle breeze.
   Ki correct when position hold is rock-steady. Too much Ki = slow oscillation.
5. Typical Kp range: 1.0–2.5. Ki range: 0.1–0.5.

### Step 4: Position loop (GPS hold)

With velocity loop tuned, enable GPS hold.

1. Set Kp = 0.5. Hover and observe.
2. Increase Kp until the drone snaps back to position after a disturbance.
3. Watch for oscillation around the setpoint (drone is moving too fast when
   it arrives). If oscillating, reduce Kp or increase velocity loop Kd.
4. Typical Kp range: 0.5–1.5. Often P-only (no I, no D needed at this level).

---

## 12. Future Trajectory Generation

The velocity controller is the natural integration point for trajectory generation
because it already accepts a velocity setpoint from above.

A trajectory generator would:
1. Given a waypoint sequence, compute a time-parameterised path through NED space
2. At each timestep, output a (position, velocity, acceleration) triplet
3. The position feeds the position controller
4. The velocity feeds the velocity controller as a feedforward term
5. The acceleration feeds the attitude controller as a feedforward tilt (a/g)

This feedforward architecture makes trajectory tracking dramatically better than
pure feedback: the vehicle doesn't need to build up an error before it starts
responding — it's already tilted and moving before it reaches each waypoint.

The minimum snap trajectory generation approach (Mellinger & Kumar 2011) gives
smooth acceleration profiles that respect jerk limits automatically. This is
what produces the silky cinematics of high-end professional drones.

The code hooks for this already exist:
- `PositionController::captureSetpoint()` → set waypoint
- `PositionController::nudgeSetpoint()` → move setpoint along trajectory
- `VelocityController::update(vel_sp, ...)` → feedforward velocity
- `VelocityController::sticksToVelSetpoint()` → replace with trajectory velocity

For now, trajectory generation is not implemented. The architecture is ready
when you need it.

---

## 13. Key Design Decisions and Tradeoffs

### Why not a single 6-DOF controller (LQR/MPC)?

An LQR (Linear Quadratic Regulator) or MPC (Model Predictive Control) treats
all states simultaneously and theoretically produces the globally optimal
control. In practice, for a drone flight controller on an STM32:

- MPC at 3840 Hz on a Cortex-M7 is not feasible (quadratic programme per step)
- LQR requires an accurate vehicle model (mass, inertia tensor, rotor constants)
  that changes with payload, battery level, and temperature
- Cascaded PID is transparent: you can observe each loop's output independently
  and know exactly which loop to tune when something is wrong
- The bandwidth separation in cascaded PID provides inherent robustness that
  is difficult to achieve in single-loop designs on uncertain plants

LQR/MPC are candidates for a future enhancement. The architecture supports
them: the rate and attitude controllers could be replaced with LQR state-feedback
while keeping the velocity/position loops as-is.

### Why 3840 Hz IMU rate?

The LSM6DSV can produce data at 3840 Hz. Running the rate loop at IMU rate
eliminates aliasing in the gyro signal and allows the D-term low-pass filter
to be set at 400+ Hz, preserving damping while rejecting noise.

At lower IMU rates (1000 Hz, common in older controllers), the D term must
be filtered more aggressively, which introduces phase lag and reduces the
maximum stable D gain.

### Why Mahony AHRS + EKF, not just EKF?

The EKF at 480 Hz is not fast enough to propagate the attitude estimate for the
3840 Hz rate loop. The Mahony filter (computationally trivial at 3840 Hz) provides
the fast attitude estimate. The EKF runs slower but provides accurate position,
velocity, and bias estimates. The Mahony gyro bias is continuously corrected by
the EKF bias output.

This combination is the same architecture used in the open-source InvenSense ICM
fusion library and similar to PX4's attitude estimator design.

---

## Reference: System State Variables

| Variable | Symbol | Frame | Source | Updated at |
|---|---|---|---|---|
| Position | p | NED, m | EKF | 480 Hz |
| Velocity | v | NED, m/s | EKF | 480 Hz |
| Attitude | q | quat (body→NED) | Mahony | 3840 Hz |
| Euler angles | φθψ | ZYX, rad | From q | 3840 Hz |
| Angular rate | ω | body FRD, rad/s | IMU (bias corrected) | 3840 Hz |
| Gyro bias | b_g | body, rad/s | EKF | 480 Hz |
| Accel bias | b_a | body, m/s² | EKF | 480 Hz |
| Baro altitude | h | MSL, m | MS5611 | 50 Hz |
| GPS position | p_gps | WGS84 | USART1 | 10 Hz |
| Mag field | m | body, norm | QMC5883L | 100 Hz |
| Motor eRPM | ω_i | elec, RPM | DSHOT bidir | 3840 Hz |
| Motor thrust | F_i | body, N | kF × RPM² | 3840 Hz |

## Reference: Frame Conventions

| Frame | Name | Definition |
|---|---|---|
| World | NED | North=+X, East=+Y, Down=+Z |
| Body | FRD | Forward=+X, Right=+Y, Down=+Z |
| Sensor | chip | Remapped to FRD in driver |

Gravity in NED: **g = [0, 0, +9.80665] m/s²**  (positive down)

Quaternion convention: **q rotates vectors from body FRD to world NED.**
`v_NED = q ⊗ [0, v_body] ⊗ q*`
