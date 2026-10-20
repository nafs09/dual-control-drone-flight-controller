# Pin allocation and board integration notes

This guide consolidates the intended pin allocation from inc/fc_config.h. It is a firmware-side plan, not a verified board schematic or generated STM32Cube configuration. Resolve the conflicts below against the actual PCB before wiring, CubeMX setup, or motor tests.

## MCU

- STM32F722RET6, LQFP64, nominal 216 MHz.
- Firmware comments assume an 8 MHz HSE and no fitted LSE crystal.
- The project contains no .ioc file or generated peripheral initialization; pin macros alone do not configure GPIO, AFs, DMA, or clocks.

## Intended allocation

| Function | Peripheral | Pins |
|---|---|---|
| LSM6DSV IMU | SPI1 | SCK PA5, MISO PA6, MOSI PA7, CS PA4, DRDY PC4 / EXTI4 |
| MS5611 barometer | SPI1 | Shared bus, CS PC5; hardware driver not implemented |
| QMC5883L magnetometer | I2C1 | SCL PB6, SDA PB7; hardware driver/calibration not implemented |
| GPS | USART1 | TX PA9, RX PA10; UBX NAV-PVT parser is present |
| CRSF / ELRS | USART2 | TX PA2, RX PA3; configured target baud 420 kbaud |
| Motor 1 | UART4 | PA0 |
| Motor 2 | UART5 | PC12 |
| Motor 3 | USART3 | PB10 |
| Motor 4 | USART6 | PC6 |
| USB CDC | USB OTG FS | DM PA11, DP PA12; MAVLink serialization is not implemented |
| microSD | SPI2 | SCK PB13, MISO PB14, MOSI PB15, CS PC1 per fc_config.h; logger is not implemented |
| Battery voltage | ADC1 channel 1 | PA1; software currently assumes a fixed divider |
| Current sensor signal | ADC1 channel 10 | PC0; not currently sampled into battery state |
| Status LED | GPIO | PB0 |
| Error LED | GPIO | PB12 |
| Buzzer | GPIO / optional timer | PC7; not integrated |

Other pins described in the legacy board notes (INA226 alerts, SmartAudio, GPS timepulse, camera trigger/shutter, USB VBUS detect, and rail-voltage ADCs) are not active allocations in the current firmware source. Treat them as unassigned until the schematic and CubeMX project are reconciled.

## Conflicts found in the legacy pin guide

1. **SD chip select:** the firmware config assigns PC1. The older pin guide assigned PC3 and also used PC1 for ADC1_IN11. PC1 cannot serve both functions. This document follows fc_config.h: PC1 is reserved for SD CS; remove or move the old ADC1_IN11 assignment. Do not wire SD CS to PC3 unless the firmware config is deliberately changed.
2. **PC14:** the firmware pin comments associate PC14/PC15 with the LSE oscillator footprint, while the legacy guide also assigned PC14 as USB VBUS detect. Confirm whether an LSE crystal is fitted and choose one function; the current code does not define or read a VBUS-detect pin.
3. **PC0:** current firmware config labels PC0 as the current-sensor ADC input. The safety task currently samples battery voltage only; current sensing is not implemented.
4. **Peripheral details:** alternate-function numbers, SPI modes, DShot inversion, DMA channels, EXTI priorities, USB clocking, and analogue scaling must be checked against the MCU datasheet, board schematic, and the generated project. The old long guide contained conflicting/obsolete duplicated macro tables; use inc/fc_config.h as the current software allocation reference, not those historical copies.

## Required project integration

Before a build or hardware test, provide the STM32Cube-generated startup/HAL project and confirm:

- System clock and 48 MHz USB clock.
- SPI1/SPI2 and I2C1 configuration, chip-select GPIO defaults, and bus speeds.
- USART/UART alternate functions, half-duplex behavior, baud rates, DMA/interrupt routing, and callback ownership.
- ADC channel assignments, sampling times, voltage divider and current-sensor calibration.
- IMU EXTI4 and GPS/CRSF receive behavior, priorities, and RTOS-safe ISR calls.
- USB CDC, FreeRTOS/CMSIS-RTOS v2, IWDG, and DWT setup.

No hardware assignment in this document should be taken as electrically verified until it matches the PCB schematic and passes continuity and peripheral bring-up checks.
