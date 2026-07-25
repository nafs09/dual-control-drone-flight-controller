# Drone Flight Controller — Architecture & Setup Guide

## Overview

High-end dual-mode quadrotor flight controller for the **STM32F722RET6** (216 MHz Cortex-M7 + FPU).
Supports seamless switching between **FPV/Acro** and **Cinematic Camera** flight modes.

For full control architecture explanation including the velocity controller, wind
rejection math, and tuning procedure, see **[process.md](process.md)**.

---

## Hardware

| Component | Part | Interface | Pins |
|---|---|---|---|
| MCU | STM32F722RET6 | — | LQFP64 |
| IMU | LSM6DSV320XTR | SPI1 | PA5/6/7, CS=PA4, DRDY=PC4 |
| Barometer | MS5611 | SPI1 | shared SCK/MISO/MOSI, CS=PC5 |
| Magnetometer | QMC5883L | I2C1 | PB6 SCL, PB7 SDA, INT=PD2 |
| GPS | Any NMEA/UBX | USART1 | PA9 TX, PA10 RX |
| RC Receiver | CRSF/ELRS | USART2 @ 420 kbaud | PA2 TX, PA3 RX |
| Telemetry | MAVLink via **USB CDC** | USB OTG FS | PA11 D−, PA12 D+ |
| Motor 1 | Bidir DSHOT300 | UART4 half-duplex | PA0 |
| Motor 2 | Bidir DSHOT300 | UART5 half-duplex | PC12 |
| Motor 3 | Bidir DSHOT300 | USART3 half-duplex | PB10 |
| Motor 4 | Bidir DSHOT300 | USART6 half-duplex | PC6 |
| SD Card | SPI2 | SPI2 | PB13/14/15, CS=PC1 |
| Battery V | Resistor divider | ADC1 IN1 | PA1 |
| Current | Hall sensor | ADC1 IN10 | PC0 |
| Status LED | Green | GPIO | PB0 |
| Error LED | Red | GPIO | PB12 |
| Buzzer | PWM or GPIO | PC7 | TIM8_CH2 optional |

> **UART count:** the LQFP64 package exposes exactly 6 UARTs
> (USART1,2,3,6 + UART4,5). MAVLink uses USB CDC to free USART3 for Motor 3.
> UART7/8 exist in silicon but their pins (PE/PF) are not bonded in this package.

---

## Complete LQFP64 Pin Table

Pin 1 = bottom-left (notch). Numbering goes counterclockwise.

| Pin | GPIO | Function | Notes |
|---|---|---|---|
| 1 | VBAT | Battery backup | 100 nF to GND |
| 2 | PC13 | Free GPIO | RTC wake-up |
| 3 | PC14 | OSC32_IN | 32.768 kHz crystal |
| 4 | PC15 | OSC32_OUT | |
| 5 | PH0 | OSC_IN | 8 MHz HSE crystal |
| 6 | PH1 | OSC_OUT | |
| 7 | NRST | Reset | 100 nF to GND |
| 8 | PC0 | ADC1_IN10 | Current sensor |
| 9 | PC1 | GPIO out | SD card /CS |
| 10 | PC2 | Free GPIO | |
| 11 | PC3 | Free GPIO | |
| 12 | VSSA | Analogue GND | Star GND to pin 13 |
| 13 | VDDA | Analogue VDD | 1 µF + 10 nF to VSSA |
| 14 | **PA0** | **UART4 TX AF8** | **Motor 1 half-duplex** |
| 15 | **PA1** | **ADC1_IN1** | **Battery voltage** |
| 16 | **PA2** | **USART2 TX AF7** | **CRSF TX** |
| 17 | **PA3** | **USART2 RX AF7** | **CRSF RX** |
| 18 | VSS | Ground | |
| 19 | VDD | 3.3 V | |
| 20 | **PA4** | **GPIO out** | **IMU /CS (SPI1)** |
| 21 | **PA5** | **SPI1 SCK AF5** | **IMU + Baro** |
| 22 | **PA6** | **SPI1 MISO AF5** | |
| 23 | **PA7** | **SPI1 MOSI AF5** | |
| 24 | **PC4** | **EXTI4 input** | **IMU DRDY (INT1)** |
| 25 | **PC5** | **GPIO out** | **Baro /CS (SPI1)** |
| 26 | **PB0** | **GPIO out** | **Status LED (GREEN)** |
| 27 | PB1 | ADC1_IN9 | RSSI / spare |
| 28 | PB2 | BOOT1 | Tie to GND |
| 29 | **PB10** | **USART3 TX AF7** | **Motor 3 half-duplex** |
| 30 | VCAP_1 | Internal reg cap | **4.7 µF ESR<0.2 Ω to GND — NOT GPIO** |
| 31 | VSS | Ground | |
| 32 | VDD | 3.3 V | |
| 33 | **PB12** | **GPIO out** | **Error LED (RED)** |
| 34 | **PB13** | **SPI2 SCK AF5** | **SD card** |
| 35 | **PB14** | **SPI2 MISO AF5** | **SD card** |
| 36 | **PB15** | **SPI2 MOSI AF5** | **SD card** |
| 37 | **PC6** | **USART6 TX AF8** | **Motor 4 half-duplex** |
| 38 | PC7 | GPIO / TIM8_CH2 | Buzzer |
| 39 | PC8 | Free GPIO | Camera trigger (TIM8_CH3) |
| 40 | PC9 | Free GPIO | |
| 41 | PA8 | Free / TIM1_CH1 | Camera shutter |
| 42 | **PA9** | **USART1 TX AF7** | **GPS TX** |
| 43 | **PA10** | **USART1 RX AF7** | **GPS RX** |
| 44 | **PA11** | **USB OTG FS D−** | **MAVLink CDC** |
| 45 | **PA12** | **USB OTG FS D+** | **MAVLink CDC** |
| 46 | PA13 | SWDIO | Debug — keep accessible |
| 47 | VCAP_2 | — | 100 nF to VDD |
| 48 | VDD | 3.3 V | |
| 49 | PA14 | SWCLK | Debug |
| 50 | PA15 | Free GPIO | (JTDI, remap after debug) |
| 51 | PC10 | Free GPIO | |
| 52 | PC11 | Free GPIO | |
| 53 | **PC12** | **UART5 TX AF8** | **Motor 2 half-duplex** |
| 54 | **PD2** | **EXTI2 input** | **Mag data-ready** |
| 55 | PB3 | Free GPIO | (JTDO) |
| 56 | PB4 | Free GPIO | |
| 57 | PB5 | Free GPIO | |
| 58 | **PB6** | **I2C1 SCL AF4** | **QMC5883L** |
| 59 | **PB7** | **I2C1 SDA AF4** | **QMC5883L** |
| 60 | BOOT0 | Boot mode | Tie GND, add button for DFU |
| 61 | PB8 | Free GPIO | I2C1 SCL alt |
| 62 | PB9 | Free GPIO | I2C1 SDA alt |
| 63 | VSS | Ground | |
| 64 | VDD | 3.3 V | |

---

## Software Architecture

```
/src
├── core/
│   ├── shared_state.hpp     Central data bus + motor telemetry fields
│   └── system_init.cpp      All tasks + ISRs (USB CDC for MAVLink)
├── math/
│   ├── vector3.hpp          3D vector
│   └── quaternion.hpp       Hamilton quaternion + exp-map
├── estimation/
│   ├── mahony_ahrs.hpp/cpp  AHRS @ 3840 Hz (improvements #5,8,9,10)
│   └── ekf.hpp/cpp          15-state ES-EKF (improvements #2-7,9,11-13)
├── drivers/
│   ├── lsm6dsv.hpp/cpp      LSM6DSV320XTR IMU
│   └── startup_calibration.hpp  Pre-arm gyro bias
├── control/
│   ├── pid.hpp              Generic PID with anti-windup
│   ├── rate_controller.hpp  Inner loop @ 3840 Hz
│   ├── attitude_controller.hpp  Quaternion attitude @ 3840 Hz
│   ├── velocity_controller.hpp  Velocity + wind rejection @ 100 Hz  ← NEW
│   └── position_controller.hpp  GPS position hold @ 50 Hz
├── actuation/
│   ├── dshot.hpp/cpp        Bidir DSHOT300 UART half-duplex
│   └── mixer.hpp            X-frame mixer + thrust linearisation
├── flight_modes/
│   └── flight_mode_fsm.hpp/cpp  FSM with full cascaded camera path
├── communication/
│   └── crsf.hpp/cpp         CRSF/ELRS parser
├── safety/
│   └── safety_monitor.hpp/cpp   100 Hz watchdog
└── simulation/
    └── sim_interface.hpp    HIL/SIL hooks
```

---

## Control Loop Architecture

```
CAMERA MODE:

  GPS pos error [m]
        ↓  50 Hz  PositionController
  vel setpoint [m/s, NED]
        ↓  100 Hz VelocityController   ← wind rejection, smooth braking
  tilt [roll, pitch, rad]
        ↓  3840 Hz AttitudeController  (q_err = q_des ⊗ q⁻¹)
  rate setpoint [rad/s]
        ↓  3840 Hz RateController      (3× independent PIDs)
  torque [-1..+1]
        ↓  Mixer  (X-frame matrix + desaturation)
  DSHOT[0..2047] × 4

FPV MODE:  RC sticks → rate setpoint → RateController → Mixer
           (all outer loops bypassed)
```

---

## Mode Differences

| Parameter | FPV / Acro | Camera / Stable |
|---|---|---|
| Position loop | Off | GPS hold (50 Hz) |
| **Velocity loop** | **Off** | **On (100 Hz) — wind rejection** |
| Attitude loop | Off — full acro | On (3840 Hz) |
| Rate PIDs | High Kp/Kd | Smooth Kp/Kd |
| Max roll rate | 800 deg/s | 220 deg/s |
| Max tilt | Unlimited | ±40° |
| Yaw | Rate control | Heading hold |
| Motor idle | 150 | 80 |
| Expo (roll) | 40% cubic | 15% cubic |
| Stick mapping | Sticks → rates | Sticks → velocity |

---

## Bug Fixes vs Original Codebase

| # | Bug | File | Marker |
|---|---|---|---|
| 1 | `0.07FF` float literal used as bitmask — all CH2-CH16 garbage | `communication/crsf.cpp` → `unpackChannels()` | `/* BUG FIX #1 */` |
| 2 | No CRC validation on CRSF frames — corrupted frames accepted | `communication/crsf.cpp` → `feedByte()` | `/* BUG FIX #2 */` |
| 3 | No RC failsafe timeout — stale values persisted indefinitely | `communication/crsf.cpp` → `tick1ms()` | `/* BUG FIX #3 */` |
| 4 | Raw [172-1811] RC values reached PID without normalisation | `communication/crsf.cpp` → `processFrame()` | `/* BUG FIX #4 */` |
| 5 | `System_Init` truncated — mutexes never created, scheduler never started | `core/system_init.cpp` → `FC_SystemInit()` | `/* BUG FIX #5 */` |
| 6 | `mavTxMutex` shared with debug UART in `safety.c` — race condition | `core/system_init.cpp` → `MAVLinkTask()` | `/* BUG FIX #6 */` |
| 7 | SD card opened/closed per log write — ~100 ms stall per entry | `core/system_init.cpp` → `LoggerTask()` | `/* BUG FIX #7 */` |
| 8 | Single-loop PID on angle only, no inner rate loop | `control/rate_controller.hpp`, `control/attitude_controller.hpp` | `/* BUG FIX #8 */` |
| 9 | No IMU driver — `ax/gx` were uninitialised globals | `drivers/lsm6dsv.hpp/.cpp` | `/* BUG FIX #9 */` |
| 10 | GPS fence used raw lat/lon deltas, not real distance | `safety/safety_monitor.cpp` → `checkGeofence()` | `/* BUG FIX #10 */` |
| 11 | `safety.c` `while(1)` on fault with no recovery | `safety/safety_monitor.cpp` → `triggerMotorKill()` | `/* BUG FIX #11 */` |
| PINS-A | `STATUS_LED` on PA5 — conflicts with SPI1_SCK | `Inc/fc_config.h` | Moved to PB0 |
| PINS-B | Motor 4 on UART7 — not bonded in LQFP64 package | `actuation/dshot.hpp`, `core/system_init.cpp` | Moved to USART6/PC6 |

---

## Estimator Improvements

| # | What changed | File | Marker |
|---|---|---|---|
| 2 | Exact Φ=expm(F·dt) via Taylor expansion | `estimation/ekf.cpp` → `vanLoan()` | `/* Improvement #2 */` |
| 3 | Exact Q_d via Gauss-Legendre quadrature | `estimation/ekf.cpp` → `vanLoan()` | `/* Improvement #3 */` |
| 4 | Quaternion injection via exp-map (not small-angle) | `estimation/ekf.cpp` → `injectCorrection()` | `/* Improvement #4 */` |
| 5 | Full δθ̇ = −\[ω\]×δθ − δb_g in F matrix | `estimation/ekf.cpp` → `buildF()` | `/* Improvement #5 */` |
| 6 | Gauss-Markov bias model (τ_g, τ_a decay) | `estimation/ekf.cpp` → `buildF()` | `/* Improvement #6 */` |
| 7 | IMU preintegration (ΔR,Δv,Δp) between EKF steps | `estimation/ekf.cpp` → `accumulateIMU()` | `/* Improvement #7 */` |
| 8 | Exact quaternion integration via exp-map | `math/quaternion.hpp` → `integrateBodyRate()` | `/* Improvement #8 */` |
| 9 | 3D vector mag update (cross-product innovation) | `estimation/mahony_ahrs.cpp`, `estimation/ekf.cpp` | `/* Improvement #9 */` |
| 10 | Continuous accel weighting w=exp(−(‖a‖−g)²/σ²) | `estimation/mahony_ahrs.cpp` → `updateIMU()` | `/* Improvement #10 */` |
| 11 | NIS innovation gating (χ² test) | `estimation/ekf.cpp` → `kalmanUpdate()` | `/* Improvement #11 */` |
| 12 | Adaptive R (GPS: hDOP+sats; baro: vibe; mag: field) | `estimation/ekf.cpp` → update methods | `/* Improvement #12 */` |
| 13 | Post-injection covariance reset P←G·P·Gᵀ | `estimation/ekf.cpp` → `resetCovariance()` | `/* Improvement #13 */` |

---

## CubeMX Configuration Checklist

### Serial peripherals

- [ ] **USART1** 115200 baud, async, PA9 TX, PA10 RX — GPS
- [ ] **USART2** 420000 baud, async, PA2 TX, PA3 RX, RX interrupt — CRSF
- [ ] **USART3** 3000000 baud, half-duplex, PB10, TX+RX inverted, DMA TX+RX — Motor 3
- [ ] **UART4**  3000000 baud, half-duplex, PA0,  TX+RX inverted, DMA TX+RX — Motor 1
- [ ] **UART5**  3000000 baud, half-duplex, PC12, TX+RX inverted, DMA TX+RX — Motor 2
- [ ] **USART6** 3000000 baud, half-duplex, PC6,  TX+RX inverted, DMA TX+RX — Motor 4

### SPI / I2C

- [ ] **SPI1** Mode 3 (CPOL=1 CPHA=1), Master, 8-bit, ≤10 MHz, DMA
- [ ] **SPI2** Mode 0 (CPOL=0 CPHA=0), Master, 8-bit, ≤25 MHz, DMA — SD card
- [ ] **I2C1** 400 kHz fast mode, PB6 SCL, PB7 SDA

### USB

- [ ] **USB_OTG_FS** Device only, CDC class — MAVLink telemetry
- [ ] PLL USB clock: 48 MHz (requires PLLQ divider set correctly)

### ADC / GPIO

- [ ] **ADC1** IN1 (PA1) + IN10 (PC0), 12-bit, single conversion
- [ ] **EXTI4** PC4, rising edge, priority 5 — IMU DRDY
- [ ] **EXTI2** PD2, rising edge — Mag data-ready

### Timers

- [ ] **TIM1** can be used for camera shutter/trigger (PA8 TIM1_CH1 AF1) — optional
- [ ] **TIM8 CH2** on PC7 AF3 for buzzer PWM — optional

### System

- [ ] **IWDG** prescaler for ~32 ms timeout
- [ ] **FreeRTOS** CMSIS-OS v2, heap_4, 1 ms tick
- [ ] **DWT** cycle counter enabled (done in FC_SystemInit via dwt_init())

---

## ESC Setup

For each ESC (BLHeli_32 or AM32):
1. Connect via ESC configurator passthrough
2. Set protocol: **DSHOT300**
3. Enable **Bidirectional DSHOT** (checkbox in BLHeliSuite32 / AM32 Configurator)
4. Motor direction: verify CW/CCW matches mixer layout (see mixer.hpp)
5. Motor timing: Auto or Low for camera, Medium for FPV

---

## First Flight Checklist

1. Flash firmware with motors **disconnected**
2. Connect GCS via USB (MAVLink CDC) — verify attitude display
3. Verify CRSF link: arm switch low, mode switch low (FPV)
4. Check RC channel mapping in `fc_config.h` (RC_CH_ROLL etc.)
5. Connect motors — verify rotation directions (X-frame CW/CCW)
6. Arm: throttle low + arm switch high, hold 500 ms
7. First flight in **Camera mode** — hover, check attitude hold
8. Verify velocity mode: push stick forward, release — should decelerate and stop
9. Switch to **FPV** — verify rate response
10. Tune rate PIDs per `process.md` §11

---

## TODO (Next Steps)

- [ ] MS5611 barometer driver (`drivers/ms5611.hpp/.cpp`)
- [ ] QMC5883L magnetometer driver (`drivers/qmc5883l.hpp/.cpp`)
- [ ] GPS UBX parser — NAV-PVT (`communication/ubx_parser.hpp/.cpp`)
- [ ] MAVLink frame builder (`communication/mavlink_task.cpp` → `CDC_Transmit_FS`)
- [ ] SD card logger via FatFS (`logging/sd_logger.hpp/.cpp`)
- [ ] USB-CDC CLI for in-field PID tuning
- [ ] LIDAR altitude driver — TFmini-S or VL53L1X
- [ ] Return-to-home on geofence breach
- [ ] DSHOT direction command on first boot (repeat 10×)
- [ ] Thrust linearisation table from test bench (kF, RPM_max)
- [ ] Velocity controller Kp/Ki tuning from flight logs
- [ ] EKF gyro/accel correlation time (τ) tuning
- [ ] Mag declination update from GPS position
- [ ] Improvement #1: Euler-Lagrange rigid-body process model (needs I_xx/yy/zz from bench)
