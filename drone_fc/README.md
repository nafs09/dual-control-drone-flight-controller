# Flight controller architecture and setup

## Scope and status

This folder contains the Drone OS v1.1 flight-controller sources for the STM32F722RET6. The source package does not include the generated STM32Cube project, HAL/CMSIS-RTOS implementation, linker script, startup code, or a build system. Peripheral pin macros describe the intended board allocation; they do not configure the hardware by themselves.

The primary estimator is the 15-state right-invariant navigation EKF in src/estimation/invariant_ekf.*. The corrected 24-state ES-EKF in src/estimation/ekf.* remains a separate implementation and is not selected by the v1.1 control loop. Mahony AHRS source is retained but is not the active v1.1 attitude source.

This is a development source tree, not flight-ready firmware. The barometer and magnetometer hardware drivers, MAVLink serialization, SD/FatFS logging, current measurement, startup-calibration call site, DShot telemetry scale validation, and board-level configuration remain unfinished.

## Runtime architecture

| Path | Nominal rate | Current behavior |
|---|---:|---|
| IMU/control loop | 3840 Hz | Waits for IMU data-ready; samples IMU, propagates nominal InvEKF state, runs FSM and control, sends DShot commands |
| InvEKF covariance | 480 Hz | Covariance prediction every 8 IMU samples |
| Gravity correction | about 101 Hz | Accelerometer gravity observation when the sample is usable |
| Camera velocity loop | about 101 Hz | Converts NED velocity error to bounded roll, pitch, and thrust correction |
| Camera position loop | about 49.9 Hz | GPS position/vertical-position hold setpoint update when hold is requested and GPS fix is valid |
| GPS | 10 Hz task | Parses UBX NAV-PVT, establishes a local NED origin, updates position/velocity, and marks fixes stale after 500 ms |
| Barometer | 50 Hz task | Update consumer exists; MS5611 read/driver is not implemented |
| Magnetometer | 100 Hz task | Update consumer exists; QMC5883L read and calibration are not implemented |
| Safety | 100 Hz | Checks IMU, RC, tilt, battery voltage, geofence, and motor telemetry |
| CRSF | task polling | Reads USART2 input and applies receiver timeout |
| MAVLink | 10 Hz task | Task placeholder; messages are not built or transmitted |
| Logger | queue task | Task placeholder; FatFS writes are not implemented |

The nominal rates describe software scheduling. Actual rates and latency depend on the external project’s clock tree, sensor configuration, interrupt setup, and RTOS tick.

## Estimator and frames

The estimator uses body FRD axes (forward, right, down) and world NED axes (north, east, down). Its quaternion rotates body vectors into NED. Gravity is positive on the NED down axis.

InvEKF propagates attitude, velocity, and position for each IMU sample, accumulates samples between covariance predictions, and applies available measurement updates. The current sensor integration is mixed: the UBX GPS path is implemented, while barometer and magnetometer tasks only consume valid data supplied through shared state or simulation. The GPS local origin is the first accepted fix.

See [process.md](process.md) for loop equations, timing, and estimator update rates.

## Controller modes

- **FPV:** receiver sticks command body-rate setpoints; the rate controller drives the mixer.
- **Camera:** receiver sticks command NED velocity. The velocity controller commands tilt and vertical thrust. When position hold is requested and GPS is valid, the position loop supplies horizontal and vertical setpoints.
- **Failsafe:** the FSM initializes a camera-style controller state, ramps a positive NED-down velocity setpoint, and only enters landing when barometer validity, low altitude, and low vertical speed agree.
- **Landing:** the current code ramps collective down before disarming; this behavior still requires controlled bench validation.

The NMPC-RTI code in src/control/future/ is an interface scaffold only. The active controller remains the cascaded PID path.

## Intended hardware allocation

The allocation below follows inc/fc_config.h and is a firmware-side plan. Confirm it against the board schematic and generated CubeMX configuration. See [src/pins.md](src/pins.md) for conflicting legacy assignments and details.

| Function | Interface and pins |
|---|---|
| IMU | LSM6DSV, SPI1 PA5/PA6/PA7, CS PA4, DRDY PC4 |
| Barometer | MS5611, shared SPI1, CS PC5; driver absent |
| Magnetometer | QMC5883L, I2C1 PB6/PB7; driver/calibration absent |
| GPS | USART1 PA9/PA10; UBX NAV-PVT parser present |
| CRSF | USART2 PA2/PA3 at 420 kbaud |
| Motors 1–4 | UART4 PA0, UART5 PC12, USART3 PB10, USART6 PC6 |
| USB CDC | USB OTG FS PA11/PA12; MAVLink messages absent |
| SD | SPI2 PB13/PB14/PB15, CS PC1; logger absent |
| Battery voltage | ADC1 channel 1, PA1; conversion currently assumes a fixed divider |
| Current signal | ADC1 channel 10, PC0; not currently sampled into battery state |
| Status/error LEDs | PB0 / PB12 |
| Buzzer | PC7; optional output, not integrated |

## Build and bring-up

1. Add these sources to the matching STM32Cube project and supply the generated HAL handles, USB CDC, CMSIS-RTOS v2, interrupt vector integration, and linker/startup files.
2. Resolve the pin conflicts documented in src/pins.md before generating or wiring the board project.
3. Configure and verify the clock tree, SPI mode and rates, USART receive/transmit modes, DMA/interrupt priorities, ADC channels, USB 48 MHz clock, RTOS tick, and IWDG timing against the actual MCU and board.
4. Keep propellers removed. Validate sensor identity/scales, frame signs, RC channels and timeout, estimator convergence, failsafe transitions, motor order/direction, and DShot response with logs or a simulator.
5. Tune PID, noise, bias time constants, thrust mapping, and safety thresholds on the target hardware before any flight evaluation.

There is no complete build configuration in this folder, so a firmware build cannot be reproduced from this source package alone.

## Known remaining work

- Implement and calibrate MS5611 and QMC5883L drivers.
- Complete MAVLink serialization and transport handling.
- Implement the FatFS logger and define the log record schema.
- Sample battery current and calibrate voltage/current conversion.
- Call startup gyro calibration with timeout/failure handling.
- Decode and validate bidirectional DShot telemetry timing; check freshness and ESC compatibility.
- Confirm DShot DMA/HAL behavior, peripheral setup, interrupt priorities, and timing on the actual board.
- Finish hardware calibration, motor/thrust characterization, magnetic reference/declination, PID and EKF tuning.
- Add a tested return-to-home strategy and any chosen rangefinder integration.
- Supply a target build project and run simulation, bench, and hardware-in-loop validation.
