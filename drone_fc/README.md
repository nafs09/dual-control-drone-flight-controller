# Firmware architecture and setup

This folder contains the Drone OS v1.1 flight-controller source for an STM32F722RET6 target. It is a source package, not a complete CubeMX firmware project: generated HAL files, startup and linker files, peripheral initialization, and a reproducible target build are not included.

The planned runtime keeps Mahony as the high-rate attitude pre-computation path and uses the 24-state left-invariant InvEKF for navigation and calibration. Mahony runs on each IMU-paced control tick and its quaternion increments are passed to InvEKF. The same Mahony attitude supplies the fast attitude controller. InvEKF fuses GPS and any valid barometer or magnetometer samples and provides NED position and velocity to the outer loops.

## Runtime paths

| Path | Nominal rate | Behavior in source |
|---|---:|---|
| IMU, Mahony, attitude/rate control | 3840 Hz | Control task waits on IMU data-ready and updates Mahony each sample |
| InvEKF nominal propagation | each IMU sample | Uses IMU data and Mahony quaternion increments |
| InvEKF covariance prediction | 480 Hz | Runs every eight nominal IMU samples |
| Camera velocity loop | ~101 Hz | Converts NED velocity error into bounded tilt and thrust requests |
| Camera position loop | ~49.9 Hz | GPS/hold-gated position setpoint update |
| GPS task | 10 Hz | Parses UBX NAV-PVT, requires a 3D fix, at least six satellites and pDOP no greater than 5 by default, sets local origin, rejects stale fixes after 500 ms |
| Barometer task | 50 Hz | Consumes relative-altitude samples; MS5611 hardware driver is pending |
| Magnetometer task | 100 Hz | Consumes calibrated field and NED reference; QMC5883L driver is pending |
| Safety monitor | 100 Hz | Checks receiver, IMU, tilt, battery, geofence, motor telemetry, and watchdog path |
| CRSF task | UART/polling | Receives RC data and applies receiver timeout |
| MAVLink and logger | task placeholders | USB CDC message generation and FatFS record writes remain unimplemented |

These are source scheduling targets. The configured MCU clock, RTOS tick, sensor ODR, interrupt priorities, DMA setup, and end-to-end latency must be measured on the target.

## Coordinate frames and measurement contracts

- Navigation/world frame is NED: north +X, east +Y, down +Z.
- Body frame is FRD: forward +X, right +Y, down +Z.
- The quaternion rotates body vectors into NED.
- Position is metres; velocity is m/s; acceleration is m/s²; angular rate is rad/s; internally angles are radians.
- The IMU accelerometer supplies specific force. Confirm polarity, scale, and mounting transform against the physical board before using navigation results.
- GPS position and velocity passed to InvEKF are local NED coordinates. The current GPS task uses the first accepted fix as origin.
- Barometer samples are altitude relative to the same navigation origin, positive up, in metres.
- Magnetometer data passed to InvEKF is a hard/soft-iron calibrated body vector in microtesla. Its NED reference must use the same units. Mahony normalizes its copy for attitude correction.

## Estimation state and update path

The InvEKF retains 24 error coordinates:

| State group | Size | Meaning |
|---|---:|---|
| Attitude / velocity / position | 9 | Left-invariant body-frame errors on SE_2(3) |
| Gyroscope bias | 3 | Body-frame rad/s |
| Accelerometer bias | 3 | Body-frame m/s² |
| Motor time constant | 1 | Scalar seconds |
| Magnetometer bias | 3 | Body-frame microtesla |
| Barometer bias | 1 | Altitude offset in metres |
| Barometer scale | 1 | Fractional scale error |
| Wind | 3 | NED m/s |
| **Total** | **24** | |

The controller uses the high-rate Mahony quaternion. InvEKF follows the quaternion’s successive increments while it propagates navigation and covariance. Accelerometer gravity correction stays in Mahony; the navigation filter does not add a second gravity observation from the same sample. GPS position/velocity, relative barometric altitude, and calibrated magnetic field updates are innovation-gated. Motor-time-constant and wind-prior update APIs are available, but the runtime does not create motor-speed observations from raw DShot values or invent wind measurements.

For equations and state/error conventions see [process.md](process.md). The complete runtime overview and colored diagrams are in [../ARCHITECTURE.md](../ARCHITECTURE.md).

## Control modes

- **FPV / Acro:** pilot sticks set body-rate targets; attitude, velocity, and position loops are bypassed.
- **Camera / stabilized:** sticks set heading-relative NED velocity targets. When hold is requested and GPS is valid, position error supplies the velocity setpoint.
- **Failsafe:** receiver or safety faults can request a bounded NED-down velocity command. Landing transition depends on valid barometer data, low altitude, and low vertical speed.
- **Landing:** the FSM reduces collective before disarming. Verify this transition and all motor outputs in simulation and on a secured bench before any flight evaluation.

The active controller is the cascaded PID path. [src/control/future/](src/control/future/README.md) contains an NMPC-RTI interface scaffold only, without a solver or motor-output connection.

## Target allocation

| Function | Intended interface |
|---|---|
| LSM6DSV IMU | SPI1 PA5/PA6/PA7, CS PA4, DRDY PC4 |
| MS5611 barometer | SPI1, CS PC5; driver pending |
| QMC5883L magnetometer | I2C1 PB6/PB7; driver and calibration pending |
| UBX GPS | USART1 PA9/PA10 |
| CRSF / ELRS | USART2 PA2/PA3 at the configured 420 kbaud target |
| Motors 1–4 | UART4 PA0, UART5 PC12, USART3 PB10, USART6 PC6 |
| USB CDC | USB OTG FS PA11/PA12; MAVLink message handling pending |
| microSD | SPI2 PB13/PB14/PB15, CS PC1; logger pending |
| Battery voltage | ADC1 channel 1, PA1; divider calibration pending |
| Current signal | ADC1 channel 10, PC0; sampling path pending |

This is a firmware allocation proposal. Resolve the conflicts in [src/pins.md](src/pins.md) against the PCB schematic before creating the CubeMX project or wiring hardware.

## Build and bring-up

1. Integrate these sources with the matching STM32Cube-generated startup, HAL, CMSIS-RTOS v2, USB CDC, and linker configuration.
2. Confirm the actual clock tree and USB clock, GPIO alternate functions, SPI/I2C modes and rates, UART baud and half-duplex behavior, DMA routing, interrupt priorities, and RTOS-safe ISR calls.
3. With propellers removed, verify IMU identity, scale, axes, data-ready timing, Mahony orientation, InvEKF convergence, GPS origin and timeout, CRSF channel mapping, failsafe transitions, DShot output, motor order, and telemetry.
4. Integrate calibrated barometer/magnetometer inputs and test altitude and field update models with known units and references.
5. Tune estimator noise, controller gains, thrust mapping, motor response, and safety limits from target measurements; then validate simulation/HIL and restrained bench behavior.

No successful target build or flight validation should be inferred from source presence alone.

## Remaining implementation work

- Complete the CubeMX/HAL/RTOS target project and verify all pin assignments against the board.
- Implement and calibrate MS5611 and QMC5883L drivers; supply valid relative altitude and calibrated magnetic vectors in the documented frames and units.
- Integrate the bounded StartupCalibration helper into startup, define whether its bias seeds the driver or estimator states, and propagate calibration timeout/failure into an explicit arm inhibit.
- Generate and send the chosen MAVLink messages over USB CDC.
- Define a log record schema, queue records, and implement FatFS open/write/flush/error handling.
- Sample battery current and calibrate voltage/current conversion.
- Validate bidirectional DShot decode timing, eRPM conversion, freshness, ESC settings, DMA behavior, and motor order.
- Supply a calibrated command-to-shaft-speed model before calling InvEKF motor-time-constant updates.
- Tune parameters and validate failsafe, landing, and safety thresholds on the configured target.
- Measure worst-case timing/memory and validate closed-loop behavior before connecting any NMPC solver.

The project’s license is in [../LICENSE.txt](../LICENSE.txt).
