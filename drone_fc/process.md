# Control, estimator, and frame reference

This document describes the v1.1 source-level design. It records the intended final architecture and mathematical conventions, while marking which measurement sources still depend on hardware integration. Source code alone is not a target validation report.

## 1. Frames, units, and quaternion convention

- World navigation axes are NED: north +X, east +Y, down +Z.
- Body axes are FRD: forward +X, right +Y, down +Z.
- The nominal quaternion maps a body vector into NED.
- Position is metres, velocity is m/s, specific force is m/s², and angular rate is rad/s. Quaternion and controller angles are radians internally.
- GPS position/velocity enter the filter in local NED. The first accepted GPS fix establishes the current local origin.
- Barometric samples are relative altitude, positive up, in metres.
- Magnetometer samples are hard/soft-iron calibrated vectors in body FRD, in microtesla; the reference field is NED and uses the same units.

Sensor sign, range, mounting rotation, timing, and calibration remain target integration checks. In particular, verify accelerometer polarity at rest: the estimator adds positive NED-down gravity to the rotated specific force, which should produce near-zero translational acceleration when stationary and level.

## 2. Active estimator architecture

### Mahony high-rate attitude pre-computation

Mahony is updated for every IMU sample in the IMU-paced control task. It integrates gyro rate after bias correction and uses the accelerometer direction as a smooth, confidence-weighted tilt observation. Valid magnetometer samples contribute a full-vector correction. Its quaternion is the low-latency attitude used by the attitude controller.

### InvEKF navigation and calibration filter

InvEKF consumes each accelerometer and gyroscope sample together with the corresponding Mahony body-to-NED quaternion. It follows the relative quaternion increment rather than replacing the AHRS with a second IMU integrator. It propagates NED position/velocity nominal states per IMU sample and predicts the covariance every eight samples (nominally 480 Hz from a 3840 Hz IMU/control target). The Mahony quaternion remains the fast control attitude; InvEKF position and velocity feed the outer loops.

Accelerometer gravity correction remains inside Mahony. The InvEKF does not apply another gravity measurement from the same sample, which would incorrectly treat one sensor observation as two independent measurements. Slower GPS, barometer, and magnetometer data update InvEKF when valid. GPS fusion is active in the current task path. Barometer and magnetometer acquisition still depend on drivers that are not included.

## 3. The 24 InvEKF error coordinates

| Index range | Size | Symbol / state | Coordinates |
|---|---:|---|---|
| 0–2 | 3 | Attitude error, δθ | Left-invariant tangent, body frame |
| 3–5 | 3 | Velocity error, δv | Left-invariant tangent, body frame |
| 6–8 | 3 | Position error, δp | Left-invariant tangent, body frame |
| 9–11 | 3 | Gyro bias, b_g | Additive body-frame rad/s |
| 12–14 | 3 | Accelerometer bias, b_a | Additive body-frame m/s² |
| 15 | 1 | Motor time constant, τ_m | Scalar seconds |
| 16–18 | 3 | Magnetometer bias, b_m | Additive body-frame µT |
| 19 | 1 | Barometer bias, b_h | Altitude offset, metres |
| 20 | 1 | Barometer scale, s_h | Fractional scale error |
| 21–23 | 3 | Wind, w_NED | Additive NED m/s |
| **Total** | **24** | | |

The first nine errors use the body-frame error Xhat inverse times X, called a left-invariant SE_2(3) error in the convention used here. Nominal position/velocity remain stored in NED. During correction injection:

\[
q^+ = q\,\operatorname{Exp}(\delta\theta/2),\qquad
v^+ = v + R(q)\,\delta v,\qquad
p^+ = p + R(q)\,\delta p.
\]

Gyro, accelerometer, and magnetic biases are additive in body coordinates. Wind is additive in NED. Motor time constant is constrained to 1 ms–1 s and barometer scale to ±20%. Bias, wind, barometer, and actuator calibration states have configurable Gauss–Markov process terms; covariance propagation uses a third-order transition approximation and two-node Gauss–Legendre integration of the continuous process noise.

## 4. Nominal inertial propagation and covariance

At each IMU tick, attitude is advanced by the shortest signed increment between the previous and current Mahony quaternions. With the midpoint attitude \(q_{k+1/2}\), body specific force \(f_b = a_m-b_a\), NED velocity \(v\), wind estimate \(w\), and configured drag coefficient \(k_d\), the source model is:

\[
a_{NED}=R(q_{k+1/2})f_b + [0,0,g]^T - k_d(v-w),
\]
\[
p_{k+1}=p_k+v_k\Delta t+\tfrac12a_{NED}\Delta t^2,\qquad
v_{k+1}=v_k+a_{NED}\Delta t.
\]

The filter accumulates mean bias-corrected force and angular rate over each covariance interval. The continuous error Jacobian contains attitude/rate coupling, force/attitude coupling, gyro/accelerometer bias coupling, position/velocity coupling, linear drag, and wind sensitivity. Process noise includes gyro and accelerometer white noise, bias random walks, actuator/magnetic/barometer parameter random walks, and horizontal/vertical wind noise.

The covariance update is \(P^- = \Phi P \Phi^T + Q_d\). Measurement updates use innovation covariance inversion with pivoting, NIS gating, a Joseph covariance form, symmetry stabilization, and an invariant error reset Jacobian. All estimator arrays are fixed-size; the filter does not allocate on the heap.

## 5. Measurement models and gates

### GPS position and velocity

The task converts latitude/longitude/altitude to local NED and passes receiver-reported `hAcc`, `vAcc`, and `sAcc` to InvEKF. A fix must have a valid 3D NAV-PVT solution, at least six satellites, and position DOP in the range (0, 5] by default; these thresholds are configurable. Receiver accuracy values already reflect its geometry estimate, so position DOP is a quality gate and is not multiplied into the measurement variance. Configured variance floors protect the update from zero or overconfident receiver values.

For position, the innovation is \(p_{GPS}-p\) in NED and the position-error block of the observation Jacobian is \(R(q)\). The velocity update likewise uses \(v_{GPS}-v\) in NED with \(R(q)\) in its velocity-error block. This preserves the receiver's axis-dependent NED covariance while the filter error coordinates remain body-frame. Position and velocity have separate NIS gates.

### Barometric altitude

For relative altitude \(h=-p_D\), scale \(s_h\), and bias \(b_h\), predicted barometer altitude is:

\[
\hat h_{baro}=h(1+s_h)+b_h.
\]

The Jacobian includes the body-frame projection of NED down-position, plus direct bias and scale terms. The caller can increase measurement uncertainty using vibration RMS. The runtime task currently uses a fixed uncertainty and waits for a valid shared-state sample; the physical MS5611 driver is pending.

### GPS relative altitude

An optional `updateGPSAltitude` method performs a scalar relative-altitude observation with the GPS position variance floor. It is available to callers but the current NAV-PVT task uses the coupled 3D position update instead.

### Magnetometer field

For calibrated body measurement \(m_b\), NED reference \(m_N\), and body bias \(b_m\), the predicted field is:

\[
\hat m_b=R(q)^Tm_N+b_m.
\]

The update estimates both attitude and three-axis magnetic bias. Its variance grows when the measured field magnitude departs from the configured local reference strength, reducing trust during interference. Mahony normalizes its own copy of the calibrated vector; InvEKF retains the field units so its bias remains in microtesla. A QMC5883L acquisition/calibration driver and the local magnetic reference are not yet part of the source package.

### Motor time constant and wind prior

`updateMotor` estimates the actuator lag from command speed, measured shaft speed, previous measured speed, and elapsed time. This update must not be driven by raw DShot command values: the caller needs a calibrated mapping from command to shaft angular speed and a valid telemetry conversion first. No runtime task currently invokes it.

`setWindPrior` accepts a NED wind estimate and uncertainty. It is an explicit optional observation, not an automatic estimate from an unvalidated drag model. The current runtime has no wind sensor or independent wind source, so no wind prior is currently applied.

## 6. Runtime schedule and data ownership

| Loop or task | Nominal rate | State/data path |
|---|---:|---|
| IMU and Mahony | 3840 Hz | DRDY paced; high-rate attitude output |
| InvEKF nominal propagation | each sample | NED position/velocity propagated from IMU and Mahony quaternion |
| InvEKF covariance prediction | 480 Hz | Every eight IMU samples |
| Attitude and body-rate control | IMU rate | Mahony attitude and bias-corrected gyro feed fast loops |
| Camera velocity loop | ~101 Hz | NED velocity to desired tilt/thrust |
| Camera position loop | ~49.9 Hz | GPS/hold-gated NED position hold |
| GPS | 10 Hz task | UBX NAV-PVT parsing, origin, fusion, stale-fix handling |
| Barometer / magnetometer | 50 / 100 Hz tasks | Measurement consumers; physical drivers pending |
| Safety | 100 Hz | Receiver, IMU, tilt, battery, geofence, motor telemetry, watchdog |

The source protects shared sensor and navigation snapshots with CMSIS-RTOS mutexes. The high-priority control loop updates the AHRS and estimator, then publishes attitude, corrected body rate, and navigation state. Measurement tasks copy input samples under their sensor mutex, release it, then update InvEKF under the navigation mutex. The AHRS gyro-bias estimate is synchronized from InvEKF after covariance and measurement updates.

## 7. Cascaded controller

### FPV / Acro

Roll, pitch, and yaw stick channels map to body angular-rate setpoints. The rate controller compares these with the bias-corrected body gyro rates and generates torque commands for the Quad-X mixer. Position, velocity, and attitude setpoint loops are bypassed.

### Camera / stabilized mode

1. **Position loop (~50 Hz):** when position hold is requested and GPS is valid, position error plus velocity damping produces a NED velocity setpoint.
2. **Velocity loop (~100 Hz):** NED velocity error produces horizontal acceleration and vertical thrust corrections.
3. **Attitude loop (IMU rate):** desired tilt and captured yaw form a desired quaternion; the shortest quaternion error becomes a body-rate setpoint.
4. **Rate loop and mixer (IMU rate):** rate PID torque and collective go through the Quad-X mixer and output limits.

If hold is not active, horizontal sticks become heading-relative velocity targets. Throttle sets the NED-down velocity target. The source RC field is named `altHold` even though it gates horizontal and vertical position hold.

### Velocity-to-tilt conversion

Given horizontal desired NED accelerations (a_N,a_E) and current yaw ψ:

\[
a_F=a_N\cos\psi+a_E\sin\psi,\qquad
a_R=-a_N\sin\psi+a_E\cos\psi,
\]
\[
\theta_{desired}=-a_F/g,\qquad
\phi_{desired}=a_R/g.
\]

The velocity controller clamps requested tilt near ±0.524 rad (30°), with an additional configured command limit in the mode path. Validate signs and axes with props removed and then on a restrained rig.

### NED-down thrust sign

NED down is positive. A positive vertical-velocity PID result means the requested downward speed exceeds the measured speed; this should reduce thrust. The implementation negates the PID result before applying it as the throttle correction. The mixer clamps output and desaturates torque, but actual thrust response depends on battery, ESC, motor, propeller, and thrust mapping.

## 8. Safety, failsafe, and landing

On failsafe entry, the FSM switches to camera-style control, captures yaw, clears selected controller state, and ramps a positive NED-down velocity target. Landing transition requires valid barometer input, low AGL altitude, and low vertical speed. Battery, geofence, receiver, and motor/IMU fault paths can request failsafe or kill outputs.

This describes source behavior, not a verified safety guarantee. Validate every trigger, reset, stale-data case, altitude reference, transition, and motor response in simulation and on a secured bench. There is no return-to-home implementation.

## 9. Future NMPC-RTI interface

`src/control/future/nmpc_rti.hpp` defines a fixed-size 20-interval horizon and separate preparation and feedback calls around a caller-supplied backend. Inputs use SI units and the frames above. The source provides no dynamics model, optimizer, QP solver, constraints, warm start, mixer connection, FSM mode, deadline monitor, or fallback behavior. A successful callback status does not establish feasibility or safety. The PID cascade remains active until an optimizer is target-bounded and validated.

## 10. Tuning and validation sequence

1. Verify IMU identity, configured scale, axes, polarity, DRDY timing, and calibration with motors disconnected.
2. Check Mahony attitude and gyro bias with stationary and controlled rotations; verify quaternion/frame conventions.
3. Check InvEKF propagation and 24 covariance diagonal values with recorded or simulated motion before enabling GPS/barometer/magnetometer updates.
4. Verify CRSF channels, switch polarity, arming, timeout, failsafe, and kill behavior.
5. Verify DShot packet timing, output order/direction, ESC compatibility, telemetry decode, freshness, and emergency output behavior with props removed.
6. Tune the rate controller, then attitude, velocity, and position loops. Confirm thrust sign, saturation, and motor response on a restrained setup.
7. Validate GPS origin/fix timeout, barometer reference, magnetic calibration, battery thresholds, geofence, failsafe, and landing behavior.
8. Tune estimator noise and state time constants using measured sensor/vehicle data. Only feed motor-lag or wind updates from calibrated, valid observations.

The package contains no complete target build system, automated estimator suite, or flight logs. Reproducible host/target build and closed-loop validation remain required.
