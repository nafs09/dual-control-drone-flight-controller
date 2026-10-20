# Control, estimator, and frame reference

This document describes the v1.1 source as implemented, and distinguishes active paths from stubs. It is not a control-system validation report.

## 1. Frames and units

- World navigation is NED: north +X, east +Y, down +Z.
- Body axes are FRD: forward +X, right +Y, down +Z.
- The nominal quaternion rotates body vectors into NED.
- Position is metres, velocity m/s, acceleration m/s², angles radians internally, and angular rate rad/s.
- Specific force from the accelerometer is bias-corrected and combined with the gravity convention in the estimator. Check the sensor driver’s configured scale and axis mapping against the physical board before use.

The attitude error sent to the body-rate loop is q_current conjugate multiplied by q_desired. That makes the small rotation error and gyro rate live in the same body frame.

## 2. Active loop schedule

| Loop | Nominal rate | Implementation |
|---|---:|---|
| IMU and FSM | 3840 Hz | Control task waits on IMU DRDY |
| InvEKF nominal propagation | each IMU sample | Integrates corrected gyro and specific force |
| InvEKF covariance prediction | about 480 Hz | Prediction every eight IMU samples |
| Gravity observation | about 101 Hz | Decimated accelerometer update |
| Camera velocity | about 101 Hz | Every 38 IMU ticks |
| Camera position | about 49.9 Hz | Every 77 IMU ticks |
| GPS update | 10 Hz task | UBX NAV-PVT parser; local NED origin is first valid fix |
| Barometer / magnetometer consumers | 50 / 100 Hz tasks | Read shared-state samples only; physical sensor drivers are incomplete |
| Safety | 100 Hz | Fault checks and watchdog feed |

These are nominal rates from the source. They require confirmation on the configured MCU project and hardware.

## 3. Estimator paths

### InvEKF (v1.1 active path)

The right-invariant estimator uses 15 error coordinates: attitude, velocity, position, gyro bias, and accelerometer bias (three each). The nominal state propagates at IMU rate. Covariance propagation is decimated to about 480 Hz, and gravity, GPS, barometer, and magnetometer updates are exposed with innovation gating and covariance reset.

The v1.1 system loop selects InvEKF. GPS updates arrive from the UBX NAV-PVT path. Barometer and magnetometer update methods are called only when their shared-state input is marked valid; this source tree does not contain the sensor drivers that would provide those physical samples.

### ES-EKF

The corrected 24-state ES-EKF remains available as a distinct estimator implementation. It is not selected by the v1.1 system loop. Its preintegration, covariance propagation and update math are separate from InvEKF.

## 4. Control cascade

### FPV

Receiver roll, pitch, and yaw channels map to body-rate setpoints. The rate controller compares those setpoints with bias-corrected body gyro rates and produces torque commands for the mixer. Attitude, velocity, and position loops are bypassed.

### Camera mode

The active cascade is:

1. Position/vertical-position hold, about 49.9 Hz when hold is selected and GPS is valid: position error plus measured-velocity damping produces an NED velocity setpoint.
2. Velocity loop, about 101 Hz: NED velocity error produces horizontal acceleration commands and a vertical thrust correction.
3. Attitude loop, each IMU tick: the desired body tilt and captured yaw produce a body-frame quaternion error and a body-rate setpoint.
4. Rate loop and mixer, each IMU tick: body-rate error produces torque, then the X-frame mixer allocates collective and torque to four DShot outputs.

If GPS is invalid or position hold is not selected, horizontal stick commands map to a heading-relative NED velocity setpoint. The throttle channel maps to vertical velocity.

## 5. Velocity-to-tilt conversion

The horizontal velocity PID produces desired NED acceleration a_N and a_E. With current yaw ψ, code rotates that acceleration into body forward/right components:

- a_forward = a_N cos(ψ) + a_E sin(ψ)
- a_right = -a_N sin(ψ) + a_E cos(ψ)
- pitch_desired = -a_forward / g
- roll_desired = a_right / g

The desired tilt is clamped to approximately ±0.524 rad (30°) in this controller. The flight-mode command path also applies configured angle limits. Gains, signs, and axis directions require validation with props removed and then on a restrained test setup.

## 6. NED vertical velocity and thrust sign

NED down is positive. The vertical PID error is setpoint velocity minus measured velocity. A positive PID result means the requested downward velocity is greater than measured; that requires less thrust. Conversely, if the aircraft is descending faster than requested, the error is negative and thrust must increase.

The v1.1 implementation negates the PID result before applying it as a throttle/thrust delta. The mixer then applies collective limits and torque desaturation. The actual thrust response depends on the ESC, motor, propeller, battery, and mixer calibration.

## 7. Position hold

The position loop runs only in camera mode when the receiver requests hold and a valid GPS fix is present. It captures the current position when hold is engaged. Its outputs become NED velocity setpoints for the velocity loop. GPS accuracy, latency, antenna placement, and local origin quality determine the practical hold performance; there is no claim of precision positioning from this code alone.

The RC field is named altHold in source even though the hold path includes horizontal position as well as the vertical coordinate.

## 8. Failsafe and landing

On entry to failsafe the FSM switches to camera-style gains, captures current yaw, clears controller state, and ramps a positive NED-down velocity command to a configured limit. The landing transition requires a valid barometer sample, low AGL altitude, and low vertical speed. Battery and geofence safety events can request failsafe asynchronously.

This is an implementation path, not a validated flight safety guarantee. Test each trigger, state transition, altitude reference, and motor response in simulation and on a secured bench. There is no return-to-home implementation.

## 9. NMPC-RTI future interface

src/control/future/nmpc_rti.hpp defines a 20-step reference horizon and a caller-supplied prepare/feedback backend. State, reference, and model inputs use SI units and the frames above. No optimizer/QP backend is supplied, and the interface is not connected to the FSM, mixer, or motor commands. Until a fixed-memory solver has bounded execution time on the target and has been validated, the active control path remains the PID cascade.

## 10. Tuning and validation order

1. Verify IMU identity, axis mapping, scale, bias, and DRDY rate with motors disconnected.
2. Verify quaternion/body/world conventions and estimator behavior using recorded or simulated motion.
3. Verify CRSF channel mapping, switch polarity, timeout, arming timer, and failsafe transition.
4. Verify DShot packet timing, motor order, spin direction, telemetry response, and kill behavior with propellers removed.
5. Tune the rate loop before camera attitude and velocity loops. Confirm thrust signs and saturation on a restrained rig.
6. Validate GPS origin, fix timeout, position hold, barometer reference, battery thresholds, geofence, and landing transition.
7. Tune estimator noise/bias parameters and thrust/motor model from measured data.

There is no included target project, automated test suite, or flight log data in this source package. Do not infer successful target execution from the source documentation.

## 11. Integration status

Implemented in source: IMU driver, CRSF input parser, UBX NAV-PVT parser and GPS update path, InvEKF and ES-EKF libraries, cascaded PID control, FSM/safety hooks, mixer, DShot output interface, and HIL/SIL state hooks.

Incomplete or awaiting hardware integration: CubeMX/HAL project, MS5611 and QMC5883L drivers, sensor calibration, MAVLink messages, FatFS logging, current measurement, startup calibration call site, validated DShot telemetry conversion, return-to-home, and target-specific tuning.
