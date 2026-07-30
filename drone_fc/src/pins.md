
# Pin Mappings and Functions

## STM32F722RET6 (LQFP64)

This document summarises the pin assignments for the flight controller hardware.
Pin numbering follows the STM32F722RET6 LQFP64 package, numbered counter-clockwise from the bottom-left corner.

---

## Pinout Summary

| Pin | GPIO  | Function                      | Notes                                            |
|-----|-------|-------------------------------|--------------------------------------------------|
| 1   | VBAT  | Battery backup                | 100 nF to GND. Tie to 3.3 V if no RTC backup.    |
| 2   | PC13  | Mode Select 1                 | Internal pull-up, active-low switch to GND.      |
| 3   | PC14  | Free GPIO                     | LSE crystal not fitted.                          |
| 4   | PC15  | Free GPIO                     | LSE crystal not fitted.                          |
| 5   | PH0   | HSE OSC_IN                    | 8 MHz crystal input.                             |
| 6   | PH1   | HSE OSC_OUT                   | 8 MHz crystal output.                            |
| 7   | NRST  | Reset                         | 100 nF to GND.                                   |
| 8   | PC0   | ADC1_IN10                     | 12 V bus voltage monitor.                        |
| 9   | PC1   | ADC1_IN11                     | 5 V bus voltage monitor.                         |
| 10  | PC2   | SmartAudio TX                 | TIM3 software UART to VTX (4800 baud).           |
| 11  | PC3   | SD Card CS                    | SPI2 chip-select.                                |
| 12  | VSSA  | Analogue Ground               | Star-connected to VDDA.                          |
| 13  | VDDA  | Analogue Supply               | 1 uF + 10 nF decoupling to VSSA.                 |
| 14  | PA0   | UART4 TX                      | Motor 1 bidirectional DSHOT300 (half-duplex).    |
| 15  | PA1   | ADC1_IN1                      | Battery voltage divider.                         |
| 16  | PA2   | USART2 TX                     | ELRS / CRSF telemetry to receiver.               |
| 17  | PA3   | USART2 RX                     | ELRS / CRSF receiver input.                      |
| 18  | VSS   | Ground                        | Digital ground.                                  |
| 19  | VDD   | 3.3 V                         | Digital supply.                                  |
| 20  | PA4   | GPIO Output                   | IMU chip select (SPI1).                          |
| 21  | PA5   | SPI1 SCK                      | Shared IMU and barometer clock.                  |
| 22  | PA6   | SPI1 MISO                     | Shared IMU and barometer MISO.                   |
| 23  | PA7   | SPI1 MOSI                     | Shared IMU and barometer MOSI.                   |
| 24  | PC4   | EXTI4 Input                   | IMU data-ready interrupt (LSM6DSV INT1).         |
| 25  | PC5   | GPIO Output                   | Barometer chip select.                           |
| 26  | PB0   | GPIO Output                   | Status LED (Green).                              |
| 27  | PB1   | ADC1_IN9                      | 3.3 V bus voltage monitor.                       |
| 28  | PB2   | BOOT1                         | Tie to GND.                                      |
| 29  | PB10  | USART3 TX                     | Motor 3 bidirectional DSHOT300.                  |
| 30  | VCAP1 | Internal regulator capacitor  | 4.7 uF (ESR < 0.2 ohm). Not GPIO.                |
| 31  | VSS   | Ground                        | Digital ground.                                  |
| 32  | VDD   | 3.3 V                         | Digital supply.                                  |
| 33  | PB12  | GPIO Output                   | Error LED (Red).                                 |
| 34  | PB13  | SPI2 SCK                      | SD card clock.                                   |
| 35  | PB14  | SPI2 MISO                     | SD card D0 (DO).                                 |
| 36  | PB15  | SPI2 MOSI                     | SD card CMD (DI).                                |
| 37  | PC6   | USART6 TX                     | Motor 4 bidirectional DSHOT300.                  |
| 38  | PC7   | TIM8_CH2 / GPIO               | Buzzer output.                                   |
| 39  | PC8   | GPIO Output                   | Camera select / VTX enable.                      |
| 40  | PC9   | GPIO Input                    | SD card detect (active-low).                     |
| 41  | PA8   | TIM1_CH1 / GPIO               | Armed LED (Blue, PWM capable).                   |
| 42  | PA9   | USART1 TX                     | GPS transmit.                                    |
| 43  | PA10  | USART1 RX                     | GPS receive.                                     |
| 44  | PA11  | USB OTG FS DM                 | MAVLink CDC (USB D-).                            |
| 45  | PA12  | USB OTG FS DP                 | MAVLink CDC (USB D+).                            |
| 46  | PA13  | SWDIO                         | SWD programming/debug.                           |
| 47  | VDD   | 3.3 V                         | Digital supply.                                  |
| 48  | VDD   | 3.3 V                         | Digital supply.                                  |
| 49  | VDD   | 3.3 V                         | Digital supply.                                  |
| 50  | PA14  | SWCLK                         | SWD clock.                                       |
| 51  | PA15  | Free GPIO                     | JTDI function available if required.             |
| 52  | PC10  | Free GPIO                     | Expansion.                                       |
| 53  | PC11  | Free GPIO                     | Expansion.                                       |
| 54  | PC12  | UART5 TX                      | Motor 2 bidirectional DSHOT300.                  |
| 55  | PD2   | Free GPIO                     | Reserved (formerly magnetometer interrupt).      |
| 56  | PB3   | Free GPIO                     | JTDO function.                                   |
| 57  | PB4   | GPIO Input                    | Mode Select 1 (active-low).                      |
| 58  | PB5   | GPIO Input                    | Mode Select 2 (active-low).                      |
| 59  | PB6   | I2C1 SCL                      | Magnetometer and INA226 sensors.                 |
| 60  | PB7   | I2C1 SDA                      | Magnetometer and INA226 sensors.                 |
| 61  | BOOT0 | Boot configuration            | Tie low; button to 3.3 V for DFU mode.           |
| 62  | PB8   | Free GPIO                     | Expansion.                                       |
| 63  | PB9   | Free GPIO                     | Expansion.                                       |
| 64  | VSS   | Ground                        | Digital ground.                                  |

---

# Peripheral Mapping

| Peripheral  | Pins                         | Purpose                                      |
|-------------|------------------------------|----------------------------------------------|
| SPI1        | PA5 / PA6 / PA7              | LSM6DSV320XTR IMU + MS5611 barometer         |
| SPI2        | PB13 / PB14 / PB15 / PC3     | SD card (SPI mode)                           |
| I2C1        | PB6 / PB7                    | QMC5883L magnetometer + 3x INA226 monitors   |
| USART1      | PA9 / PA10                   | GPS                                          |
| USART2      | PA2 / PA3                    | ELRS / CRSF receiver                         |
| USART3      | PB10                         | Motor 3 bidirectional DSHOT300               |
| UART4       | PA0                          | Motor 1 bidirectional DSHOT300               |
| UART5       | PC12                         | Motor 2 bidirectional DSHOT300               |
| USART6      | PC6                          | Motor 4 bidirectional DSHOT300               |
| USB OTG FS  | PA11 / PA12                  | MAVLink USB CDC                              |
| TIM3 + GPIO | PC2                          | SmartAudio software UART                     |
| TIM1_CH1    | PA8                          | Armed LED PWM                                |
| TIM8_CH2    | PC7                          | Buzzer output                                |

---

# ADC Channel Mapping

| ADC Channel | GPIO | Measurement              |
|-------------|------|--------------------------|
| ADC1_IN1    | PA1  | Battery pack voltage     |
| ADC1_IN9    | PB1  | 3.3 V bus voltage        |
| ADC1_IN10   | PC0  | 12 V bus voltage         |
| ADC1_IN11   | PC1  | 5 V bus voltage          |

---

# GPIO Outputs

| GPIO | Function                         |
|------|----------------------------------|
| PB0  | Status LED (Green)               |
| PB12 | Error LED (Red)                  |
| PA8  | Armed LED (Blue, PWM breathing)  |
| PC7  | Buzzer                           |
| PC8  | Camera select / VTX enable       |
| PC2  | SmartAudio output                |
| PA4  | IMU chip select                  |
| PC5  | Barometer chip select            |
| PC3  | SD card chip select              |

---

# GPIO Inputs

| GPIO | Function                  |
|------|---------------------------|
| PB4  | Mode Select 1             |
| PB5  | Mode Select 2             |
| PC4  | IMU Data Ready (EXTI4)    |
| PC9  | SD card detect            |
| PA3  | CRSF receiver input       |

---

# Motor Interfaces

| Motor   | UART    | GPIO | Protocol                  |
|---------|---------|------|---------------------------|
| Motor 1 | UART4   | PA0  | Bidirectional DSHOT300    |
| Motor 2 | UART5   | PC12 | Bidirectional DSHOT300    |
| Motor 3 | USART3  | PB10 | Bidirectional DSHOT300    |
| Motor 4 | USART6  | PC6  | Bidirectional DSHOT300    |

All four motors use bidirectional DSHOT300 in half-duplex mode for throttle commands and ESC telemetry.
