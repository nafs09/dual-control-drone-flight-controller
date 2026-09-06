# Drone Flight Controller - Master Configuration


## MCU Configuration


| Parameter              | Value                  |
|------------------------|------------------------|
| MCU                    | STM32F722RET6          |
| Core                   | ARM Cortex-M7          |
| Clock Frequency        | 216 MHz                |
| Package                | LQFP64                 |
| HSE                    | 8 MHz Crystal          |
| LSE                    | Not Fitted             |
| Available GPIO         | PC14 / PC15            |


---

# Peripheral Map


## I2C1


| Pin  | Function |
|------|----------|
| PB6  | SCL      |
| PB7  | SDA      |


### Connected Devices


| Device        | Address | Notes                                      |
|---------------|---------|--------------------------------------------|
| QMC5883L      | 0x0D    | Magnetometer inside Matek M10Q-5883 module |
| INA226 Bus 1  | 0x40    | 12 V rail current monitor                  |
| INA226 Bus 2  | 0x41    | 5 V rail current monitor                   |
| INA226 Bus 3  | 0x44    | 3.3 V rail current monitor                 |


### Notes


- The QMC5883L magnetometer is integrated into the Matek M10Q-5883 GPS module.
- No separate magnetometer is mounted on the flight controller PCB.
- The I2C bus is routed directly to the GPS connector.


---

# SPI1


| Pin  | Function |
|------|----------|
| PA5  | SCK      |
| PA6  | MISO     |
| PA7  | MOSI     |


### Connected Devices


| Device              | Chip Select | Interrupt |
|---------------------|-------------|-----------|
| LSM6DSV320XTR IMU   | PA4         | PC4       |
| MS5611 Barometer    | PC5         | None      |


---

# SPI2


| Pin  | Function |
|------|----------|
| PB13 | SCK      |
| PB14 | MISO     |
| PB15 | MOSI     |


### Connected Device


| Device       | Chip Select |
|--------------|-------------|
| microSD Card | PC3         |


### SD Card Wiring


| SD Signal | MCU Pin |
|-----------|---------|
| D3 / CS   | PC3     |
| CLK       | PB13    |
| CMD / MOSI| PB15    |
| D0 / MISO | PB14    |
| D1        | 47 k? Pull-up to 3.3 V |
| D2        | 47 k? Pull-up to 3.3 V |
| CD        | PC9     |


### Notes


- SD card operates in SPI mode only.
- 4-bit SDIO mode is intentionally unused.
- SDMMC1_CLK (PC12) is reserved for Motor 2 DSHOT output.


---

# USART Allocation


| Peripheral | Pins          | Connected Device                  |
|------------|---------------|-----------------------------------|
| USART1     | PA9 / PA10    | Matek M10Q-5883 GPS               |
| USART2     | PA2 / PA3     | RadioMaster RP3 V2 CRSF / ELRS    |
| USART3     | PB10          | Motor 3                           |
| UART4      | PA0           | Motor 1                           |
| UART5      | PC12          | Motor 2                           |
| USART6     | PC6           | Motor 4                           |


---

# USART2 CRSF Configuration


| Parameter              | Value      |
|------------------------|------------|
| Protocol               | CRSF / ELRS |
| Baud Rate              | 420000     |
| PA2                    | Receiver telemetry |
| PA3                    | CRSF input |


---

# Motor Outputs


## Bidirectional DSHOT300


All motors use:

| Parameter              | Value                  |
|------------------------|------------------------|
| Protocol               | Bidirectional DSHOT300 |
| Mode                   | Half Duplex            |
| Baud Rate              | 3 MHz                  |


### Motor Mapping


| Motor   | UART    | GPIO |
|---------|---------|------|
| Motor 1 | UART4   | PA0  |
| Motor 2 | UART5   | PC12 |
| Motor 3 | USART3  | PB10 |
| Motor 4 | USART6  | PC6  |


---

# USB OTG FS


| Pin  | Function |
|------|----------|
| PA11 | USB D-  |
| PA12 | USB D+  |


## USB-C Configuration


| Signal | Connection |
|--------|------------|
| CC1    | 5.1 k? ? GND |
| CC2    | 5.1 k? ? GND |
| VBUS   | Protected 5 V rail |
| D-     | USBLC6-2SC6 ? PA11 |
| D+     | USBLC6-2SC6 ? PA12 |


## USB Detection


| Parameter | Value |
|-----------|-------|
| GPIO      | PC14  |
| Circuit   | 100 k? / 100 k? divider |


### Logic


| State | Meaning |
|-------|---------|
| HIGH  | USB Connected |
| LOW   | Battery Only |


---

# SmartAudio


| Parameter              | Value          |
|------------------------|----------------|
| GPIO                   | PC2            |
| Protocol               | SmartAudio V2  |
| Baud Rate              | 4800           |
| Format                 | 8N1            |
| Mode                   | Half Duplex    |


### VTX Wiring


| VTX Pin       | Connection          |
|---------------|---------------------|
| GND           | GND                 |
| Video         | Camera CVBS         |
| +5 V          | 5 V Rail            |
| SmartAudio    | PC2 through 100 ?   |
| +VBAT         | 12 V Rail           |

---

# ADC1 Power Monitoring


## ADC DMA Scan Configuration


| ADC Channel | GPIO | Measurement       |
|-------------|------|-------------------|
| ADC1_IN1    | PA1  | Battery Voltage   |
| ADC1_IN10   | PC0  | 12 V Rail          |
| ADC1_IN11   | PC1  | 5 V Rail           |
| ADC1_IN9    | PB1  | 3.3 V Rail         |


---

## Voltage Divider Configuration


| Rail                  | Divider        | Ratio | Maximum ADC Voltage |
|-----------------------|----------------|-------|---------------------|
| Battery (16.8 V max)  | 100 k? / 10 k? | 11.0  | 1.53 V              |
| 12 V Rail             | 100 k? / 33 k? | 4.03  | 2.98 V              |
| 5 V Rail              | 47 k? / 47 k?  | 2.00  | 2.50 V              |
| 3.3 V Rail            | Direct         | 1.00  | 3.30 V              |


---

# INA226 Alert Inputs


All INA226 ALERT outputs are:

- Open-drain
- Active-low
- Connected to EXTI capable GPIO
- Require external pull-up resistors


## Alert Mapping


| Rail       | GPIO | EXTI Line | Pull-up            |
|------------|------|-----------|--------------------|
| 12 V Bus   | PC10 | EXTI10    | 4.7 k? ? 3.3 V     |
| 5 V Bus    | PC11 | EXTI11    | 4.7 k? ? 3.3 V     |
| 3.3 V Bus  | PD2  | EXTI2     | 4.7 k? ? 3.3 V     |


---

# GPS Timepulse


## Configuration


| Parameter        | Value             |
|------------------|-------------------|
| GPIO             | PC13              |
| Interrupt        | EXTI13            |
| Signal           | 1 PPS             |
| Pulse Width      | 100 ms            |
| Period           | 1 second          |


### Notes


- Optional depending on GPS cable.
- Leave unconnected if TIMEPULSE is not exposed.


---

# Mode Selection


Inputs use STM32 internal pull-ups.

Active-low configuration.


| GPIO | Function          |
|------|-------------------|
| PB4  | Mode Select 1     |
| PB5  | Mode Select 2     |


---

# LED Configuration


| GPIO | Function                 | Colour |
|------|--------------------------|--------|
| PB0  | Status / Success LED     | Blue   |
| PB12 | Error LED                | Orange |
| PA8  | Armed LED PWM            | Yellow |


---

# Buzzer Configuration


| GPIO | Timer     | Function              |
|------|-----------|-----------------------|
| PC7  | TIM8_CH2  | Passive Buzzer PWM    |


---

# Camera Control


| GPIO | Function                       |
|------|--------------------------------|
| PC8  | Camera Enable / Video Select   |


---

# SD Card Detection


| GPIO | Function |
|------|----------|
| PC9  | Card Detect |


## Logic


| State | Meaning              |
|-------|----------------------|
| LOW  | Card Inserted        |
| HIGH | No Card              |


---

# Expansion GPIO


## Available Pins


| GPIO | Function / Notes             |
|------|------------------------------|
| PC15 | Free GPIO                    |
| PA15 | Free GPIO / GPS TP Alternative |
| PB3  | Free GPIO (JTDO)             |
| PB8  | Free GPIO                    |
| PB9  | Free GPIO                    |


---

## Reserved GPIO


| GPIO | Purpose               |
|------|-----------------------|
| PC10 | INA226 Bus 1 ALERT    |
| PC11 | INA226 Bus 2 ALERT    |


---

# System Clock Configuration


| Parameter              | Value                  |
|------------------------|------------------------|
| MCU                    | STM32F722RET6          |
| HSE                    | 8 MHz Crystal          |
| LSE                    | Not Fitted             |
| SYSCLK                 | 216 MHz                |
| APB1 Timer Clock       | 108 MHz                |
| APB2 Timer Clock       | 216 MHz                |
| APB1 Peripheral Clock  | 54 MHz                 |
| APB2 Peripheral Clock  | 108 MHz                |


#define FC_SYSCLK_HZ          216000000UL
#define FC_APB1_TIM_HZ        108000000UL
#define FC_APB2_TIM_HZ        216000000UL
#define FC_APB1_PCLK_HZ       54000000UL
#define FC_APB2_PCLK_HZ       108000000UL


---

# STM32F722RET6 LQFP64 Pin Map


Pin numbering follows the STM32F722RET6 LQFP64 package.


## Package Pin Allocation


| Pin | GPIO   | Primary Function              | Notes                         |
|-----|--------|-------------------------------|-------------------------------|
| 1   | VBAT   | Backup Supply                 | 100 nF to GND                  |
| 2   | PC13   | GPS TIMEPULSE                 | Optional 1 PPS input           |
| 3   | PC14   | USB VBUS Detect               | 100 k? / 100 k? divider        |
| 4   | PC15   | Free GPIO                     | Expansion                      |
| 5   | PH0    | HSE OSC_IN                    | 8 MHz crystal                  |
| 6   | PH1    | HSE OSC_OUT                   | 8 MHz crystal                  |
| 7   | NRST   | Reset                         | 100 nF capacitor               |
| 8   | PC0    | ADC1_IN10                     | 12 V rail monitor              |
| 9   | PC1    | ADC1_IN11                     | 5 V rail monitor               |
| 10  | PC2    | SmartAudio TX                 | Software UART                  |
| 11  | PC3    | SD Card CS                    | SPI2 Chip Select               |
| 12  | VSSA   | Analog Ground                 |                                |
| 13  | VDDA   | Analog Supply                 | 1 µF + 10 nF decoupling        |
| 14  | PA0    | UART4 TX                      | Motor 1 Bidirectional DSHOT    |
| 15  | PA1    | ADC1_IN1                      | Battery voltage                |
| 16  | PA2    | USART2 TX                     | ELRS telemetry                 |


| Pin | GPIO   | Primary Function              | Notes                         |
|-----|--------|-------------------------------|-------------------------------|
| 17  | PA3    | USART2 RX                     | CRSF receiver input            |
| 18  | VSS    | Ground                        |                                |
| 19  | VDD    | 3.3 V Supply                  |                                |
| 20  | PA4    | IMU Chip Select               | SPI1                           |
| 21  | PA5    | SPI1 SCK                      | IMU / Barometer                |
| 22  | PA6    | SPI1 MISO                     | IMU / Barometer                |
| 23  | PA7    | SPI1 MOSI                     | IMU / Barometer                |
| 24  | PC4    | IMU Data Ready                | EXTI4                          |
| 25  | PC5    | Barometer Chip Select         | SPI1                           |
| 26  | PB0    | Status LED                    | Blue                           |
| 27  | PB1    | ADC1_IN9                      | 3.3 V rail monitor              |
| 28  | PB2    | BOOT1                         | Tie Low                        |
| 29  | PB10   | USART3 TX                     | Motor 3 DSHOT                  |
| 30  | VCAP1  | Core Regulator                | 4.7 µF low ESR                 |
| 31  | VSS    | Ground                        |                                |
| 32  | VDD    | 3.3 V Supply                  |                                |


| Pin | GPIO   | Primary Function              | Notes                         |
|-----|--------|-------------------------------|-------------------------------|
| 33  | PB12   | Error LED                     | Orange                         |
| 34  | PB13   | SPI2 SCK                      | SD Card                        |
| 35  | PB14   | SPI2 MISO                     | SD Card                        |
| 36  | PB15   | SPI2 MOSI                     | SD Card                        |
| 37  | PC6    | USART6 TX                     | Motor 4 DSHOT                  |
| 38  | PC7    | TIM8_CH2                      | Passive Buzzer PWM             |
| 39  | PC8    | Camera Control                | Video select / enable          |
| 40  | PC9    | SD Card Detect                | Active low                     |
| 41  | PA8    | TIM1_CH1                      | Armed LED PWM                  |
| 42  | PA9    | USART1 TX                     | GPS                            |
| 43  | PA10   | USART1 RX                     | GPS                            |
| 44  | PA11   | USB D-                        | USB OTG FS                     |
| 45  | PA12   | USB D+                        | USB OTG FS                     |
| 46  | PA13   | SWDIO                         | Debug                          |
| 47  | VDD    | 3.3 V Supply                  |                                |
| 48  | VDD    | 3.3 V Supply                  |                                |


| Pin | GPIO   | Primary Function              | Notes                         |
|-----|--------|-------------------------------|-------------------------------|
| 49  | VDD    | 3.3 V Supply                  |                                |
| 50  | PA14   | SWCLK                         | Debug                          |
| 51  | PA15   | Free GPIO                     | GPS TIMEPULSE alternative      |
| 52  | PC10   | INA226 ALERT                  | 12 V Bus EXTI10                |
| 53  | PC11   | INA226 ALERT                  | 5 V Bus EXTI11                 |
| 54  | PC12   | UART5 TX                      | Motor 2 DSHOT                  |
| 55  | PD2    | INA226 ALERT                  | 3.3 V Bus EXTI2                |
| 56  | PB3    | JTDO / Free GPIO              | Expansion                      |
| 57  | PB4    | Mode Select 1                 | Active Low                     |
| 58  | PB5    | Mode Select 2                 | Active Low                     |
| 59  | PB6    | I2C1 SCL                      | GPS Mag + INA226               |
| 60  | PB7    | I2C1 SDA                      | GPS Mag + INA226               |
| 61  | BOOT0  | Boot Mode                     | Pull-up button                 |
| 62  | PB8    | Free GPIO                     | Expansion                      |
| 63  | PB9    | Free GPIO                     | Expansion                      |
| 64  | VSS    | Ground                        |                                |


---

# Peripheral Usage Summary


## Communication Interfaces


| Peripheral   | Pins              | Purpose                          |
|--------------|-------------------|----------------------------------|
| I2C1         | PB6 / PB7         | QMC5883L + INA226 Monitoring     |
| SPI1         | PA5 / PA6 / PA7   | IMU + Barometer                  |
| SPI2         | PB13 / PB14 / PB15| microSD Card                     |
| USART1       | PA9 / PA10        | Matek M10Q-5883 GPS              |
| USART2       | PA2 / PA3         | CRSF / ELRS Receiver             |
| USART3       | PB10              | Motor 3 DSHOT                    |
| UART4        | PA0               | Motor 1 DSHOT                    |
| UART5        | PC12              | Motor 2 DSHOT                    |
| USART6       | PC6               | Motor 4 DSHOT                    |
| USB OTG FS    | PA11 / PA12       | MAVLink USB CDC                  |


---

# ADC Input Summary


| GPIO | ADC Channel | Measurement       |
|------|-------------|-------------------|
| PA1  | ADC1_IN1    | Battery Voltage   |
| PB1  | ADC1_IN9    | 3.3 V Rail        |
| PC0  | ADC1_IN10   | 12 V Rail          |
| PC1  | ADC1_IN11   | 5 V Rail           |


---

# GPIO Output Summary


| GPIO | Function                         |
|------|----------------------------------|
| PA4  | IMU Chip Select                  |
| PC5  | Barometer Chip Select            |
| PC3  | SD Card Chip Select              |
| PC2  | SmartAudio Output                |
| PB0  | Status LED                       |
| PB12 | Error LED                        |
| PA8  | Armed LED PWM                    |
| PC7  | Passive Buzzer PWM               |
| PC8  | Camera Enable / Video Select     |


---

# GPIO Input Summary


| GPIO | Function                         |
|------|----------------------------------|
| PC4  | IMU Data Ready                   |
| PC9  | SD Card Detect                   |
| PC13 | GPS TIMEPULSE                   |
| PC14 | USB VBUS Detect                  |
| PB4  | Mode Select 1                    |
| PB5  | Mode Select 2                    |
| PC10 | INA226 Bus 1 ALERT               |
| PC11 | INA226 Bus 2 ALERT               |
| PD2  | INA226 Bus 3 ALERT               |


---

# Interrupt Allocation


| Source             | GPIO | Interrupt |
|--------------------|------|-----------|
| IMU Data Ready     | PC4  | EXTI4     |
| INA226 Bus 1 Alert | PC10 | EXTI10    |
| INA226 Bus 2 Alert | PC11 | EXTI11    |
| INA226 Bus 3 Alert | PD2  | EXTI2     |
| GPS TIMEPULSE      | PC13 | EXTI13    |


---

# Sensor Bus Allocation


## SPI1


| Device             | Interface | Chip Select | Interrupt |
|--------------------|-----------|-------------|-----------|
| LSM6DSV320XTR IMU  | SPI1      | PA4         | PC4 DRDY  |
| MS5611 Barometer   | SPI1      | PC5         | None      |


---

## I2C1


| Device       | Address |
|--------------|---------|
| QMC5883L     | 0x0D    |
| INA226 Bus 1 | 0x40    |
| INA226 Bus 2 | 0x41    |
| INA226 Bus 3 | 0x44    |


---

## SPI2


| Device       | Interface | Chip Select |
|--------------|-----------|-------------|
| microSD Card | SPI2      | PC3         |


---

# UART Allocation


| UART      | TX Pin | RX Pin | Device                 |
|-----------|--------|--------|------------------------|
| USART1    | PA9    | PA10   | GPS                    |
| USART2    | PA2    | PA3    | CRSF / ELRS            |
| USART3    | PB10   | -      | Motor 3                |
| UART4     | PA0    | -      | Motor 1                |
| UART5     | PC12   | -      | Motor 2                |
| USART6    | PC6    | -      | Motor 4                |


---

# SD Card Configuration


| Parameter       | Value                    |
|-----------------|--------------------------|
| Interface       | SPI2                     |
| Mode            | SPI Only                 |
| Chip Select     | PC3                      |
| Clock           | PB13                     |
| MOSI            | PB15                     |
| MISO            | PB14                     |
| D1 Pull-up      | 47 k? ? 3.3 V            |
| D2 Pull-up      | 47 k? ? 3.3 V            |
| Card Detect     | PC9                      |


---

# USB Configuration


| Signal | Connection                  |
|--------|-----------------------------|
| CC1    | 5.1 k? ? GND                |
| CC2    | 5.1 k? ? GND                |
| VBUS   | Protected 5 V Rail          |
| D-     | USBLC6-2SC6 ? PA11          |
| D+     | USBLC6-2SC6 ? PA12          |


---

# Power Monitoring Summary


## ADC Monitoring


| Rail              | Divider Ratio | Maximum Input |
|-------------------|---------------|---------------|
| Battery           | 11.0          | 16.8 V        |
| 12 V Rail         | 4.03          | 12 V          |
| 5 V Rail          | 2.00          | 5 V           |
| 3.3 V Rail        | 1.00          | 3.3 V         |


---

# INA226 Alert Summary


| Rail       | GPIO | EXTI  | Pull-up          |
|------------|------|-------|------------------|
| 12 V Bus   | PC10 | EXTI10| 4.7 k? ? 3.3 V   |
| 5 V Bus    | PC11 | EXTI11| 4.7 k? ? 3.3 V   |
| 3.3 V Bus  | PD2  | EXTI2 | 4.7 k? ? 3.3 V   |


---

# Hardware Definitions


# IMU - LSM6DSV320XTR


## Hardware Configuration


| Parameter       | Value        |
|----------------|--------------|
| Interface      | SPI1         |
| Chip Select    | PA4          |
| Data Ready     | PC4          |
| Interrupt      | EXTI4        |


## Configuration Definitions


#define IMU_SPI_HANDLE      hspi1

#define IMU_CS_PORT         GPIOA
#define IMU_CS_PIN          GPIO_PIN_4

#define IMU_DRDY_PORT       GPIOC
#define IMU_DRDY_PIN        GPIO_PIN_4

#define IMU_DRDY_IRQn       EXTI4_IRQn


---

# Barometer - MS5611


## Hardware Configuration


| Parameter       | Value |
|----------------|-------|
| Interface      | SPI1  |
| Chip Select    | PC5   |


## Configuration Definitions


#define BARO_SPI_HANDLE     hspi1

#define BARO_CS_PORT        GPIOC
#define BARO_CS_PIN         GPIO_PIN_5


---

# Magnetometer - QMC5883L


## Hardware Configuration


| Parameter       | Value |
|----------------|-------|
| Interface      | I2C1  |
| Address        | 0x0D  |


## Configuration Definitions


#define MAG_I2C_HANDLE      hi2c1

#define MAG_I2C_ADDR        (0x0Du << 1)


---

# INA226 Power Monitors


## Bus Allocation


| Bus | Rail | Address |
|-----|------|---------|
| Bus 1 | 12 V Rail | 0x40 |
| Bus 2 | 5 V Rail | 0x41 |
| Bus 3 | 3.3 V Rail | 0x44 |


## Shunt Configuration


| Parameter       | Value |
|----------------|-------|
| Shunt Resistance | 5 m? |


## Configuration Definitions


#define INA226_I2C_HANDLE   hi2c1

#define INA226_ADDR_BUS1    (0x40u << 1)
#define INA226_ADDR_BUS2    (0x41u << 1)
#define INA226_ADDR_BUS3    (0x44u << 1)

#define INA226_SHUNT_OHM    0.005f


---

# INA226 Alert Configuration


| Rail       | GPIO | EXTI |
|------------|------|------|
| 12 V Bus   | PC10 | EXTI10 |
| 5 V Bus    | PC11 | EXTI11 |
| 3.3 V Bus  | PD2  | EXTI2  |


## Configuration Definitions


#define INA226_ALERT1_PORT  GPIOC
#define INA226_ALERT1_PIN   GPIO_PIN_10
#define INA226_ALERT1_IRQn  EXTI15_10_IRQn


#define INA226_ALERT2_PORT  GPIOC
#define INA226_ALERT2_PIN   GPIO_PIN_11
#define INA226_ALERT2_IRQn  EXTI15_10_IRQn


#define INA226_ALERT3_PORT  GPIOD
#define INA226_ALERT3_PIN   GPIO_PIN_2
#define INA226_ALERT3_IRQn  EXTI2_IRQn


---

# SD Card Configuration


## Hardware Configuration


| Parameter       | Value |
|----------------|-------|
| Interface      | SPI2  |
| Chip Select    | PC3   |
| Detect         | PC9   |


## Configuration Definitions


#define SD_SPI_HANDLE       hspi2


#define SD_CS_PORT          GPIOC
#define SD_CS_PIN           GPIO_PIN_3


#define SD_DETECT_PORT      GPIOC
#define SD_DETECT_PIN       GPIO_PIN_9


---

# GPS - Matek M10Q-5883


## Hardware Configuration


| Parameter       | Value |
|----------------|-------|
| Interface      | USART1 |
| TX             | PA9 |
| RX             | PA10 |


## Configuration Definitions


#define GPS_UART_HANDLE     huart1


---

# GPS TIMEPULSE


| Parameter       | Value |
|----------------|-------|
| GPIO            | PC13 |
| Interrupt       | EXTI13 |


## Configuration Definitions


#define GPS_TP_PORT         GPIOC

#define GPS_TP_PIN          GPIO_PIN_13

#define GPS_TP_IRQn         EXTI15_10_IRQn


---

# CRSF / ELRS Receiver


## Hardware Configuration


| Parameter       | Value |
|----------------|-------|
| Interface      | USART2 |
| TX             | PA2 |
| RX             | PA3 |
| Baud Rate      | 420000 |


## Configuration Definitions


#define CRSF_UART_HANDLE    huart2


---

# Motor Outputs


## Bidirectional DSHOT300


| Motor | UART | GPIO |
|-------|------|------|
| Motor 1 | UART4  | PA0  |
| Motor 2 | UART5  | PC12 |
| Motor 3 | USART3 | PB10 |
| Motor 4 | USART6 | PC6  |


## Configuration Definitions


#define MOTOR1_UART_HANDLE  huart4
#define MOTOR2_UART_HANDLE  huart5
#define MOTOR3_UART_HANDLE  huart3
#define MOTOR4_UART_HANDLE  huart6


#define DSHOT_BAUD          3000000U


#define DSHOT_BIT1_BYTE     0x01u
#define DSHOT_BIT0_BYTE     0xF8u


#define MOTOR_POLES         14
#define MOTOR_POLE_PAIRS    (MOTOR_POLES / 2)


#define MOTOR_IDLE_FPV      150U
#define MOTOR_IDLE_CAMERA   80U


#define MOTOR_MIN_THROTTLE  48U
#define MOTOR_MAX_THROTTLE  2047U


---

# SmartAudio Configuration


## Hardware Configuration


| Parameter       | Value          |
|----------------|----------------|
| GPIO            | PC2            |
| Protocol        | SmartAudio V2  |
| Baud Rate       | 4800           |
| Format          | 8N1            |
| Mode            | Half Duplex    |
| Timer Frequency | 14400 Hz        |


## VTX Wiring


| VTX Pin      | Connection              |
|--------------|-------------------------|
| GND          | Ground                  |
| Video        | Camera CVBS             |
| +5 V         | 5 V Rail                |
| SmartAudio   | PC2 through 100 ?       |
| +VBAT        | 12 V Rail               |


## Configuration Definitions


#define VTX_SA_PORT         GPIOC
#define VTX_SA_PIN          GPIO_PIN_2

#define VTX_SA_BAUD         4800U

#define VTX_SA_TIM_HZ       (VTX_SA_BAUD * 3U)


---

# USB Configuration


## USB OTG FS


| Parameter | Value |
|-----------|-------|
| Interface | USB OTG FS |
| D-        | PA11 |
| D+        | PA12 |


## VBUS Detection


| Parameter | Value |
|-----------|-------|
| GPIO      | PC14 |
| Detection | 100 k? / 100 k? divider |


## Configuration Definitions


#define VBUS_DETECT_PORT    GPIOC

#define VBUS_DETECT_PIN     GPIO_PIN_14


---

# Camera Control


| Parameter | Value |
|-----------|-------|
| GPIO      | PC8   |
| Function  | Camera Enable / Video Select |


## Configuration Definitions


#define CAM_CTRL_PORT       GPIOC

#define CAM_CTRL_PIN        GPIO_PIN_8


---

# LED Configuration


## LED Mapping


| LED        | GPIO | Colour |
|------------|------|--------|
| Status     | PB0  | Blue   |
| Error      | PB12 | Orange |
| Armed      | PA8  | Yellow |


## Configuration Definitions


#define LED_FLASH_PORT      GPIOB

#define LED_FLASH_PIN       GPIO_PIN_0


#define LED_ERROR_PORT      GPIOB

#define LED_ERROR_PIN       GPIO_PIN_12


#define LED_ARMED_PORT      GPIOA

#define LED_ARMED_PIN       GPIO_PIN_8


---

# Buzzer Configuration


## Hardware Configuration


| Parameter | Value |
|-----------|-------|
| GPIO      | PC7   |
| Timer     | TIM8_CH2 |
| Function  | Passive Buzzer PWM |


## Configuration Definitions


#define BUZZER_PORT         GPIOC

#define BUZZER_PIN          GPIO_PIN_7


---

# Mode Selection Configuration


## Hardware Configuration


| Switch | GPIO |
|--------|------|
| MODE1  | PB4  |
| MODE2  | PB5  |


## Configuration Definitions


#define MODE_SW1_PORT       GPIOB

#define MODE_SW1_PIN        GPIO_PIN_4


#define MODE_SW2_PORT       GPIOB

#define MODE_SW2_PIN        GPIO_PIN_5


---

# ADC Configuration


## ADC Handle


#define ADC_HANDLE          hadc1


---

## Channel Allocation


| ADC Channel | GPIO | Measurement |
|-------------|------|-------------|
| ADC1_IN1    | PA1  | Battery Voltage |
| ADC1_IN9    | PB1  | 3.3 V Rail |
| ADC1_IN10   | PC0  | 12 V Rail |
| ADC1_IN11   | PC1  | 5 V Rail |


---

## ADC Definitions


#define ADC_CH_PACK         ADC_CHANNEL_1

#define ADC_CH_BUS12V       ADC_CHANNEL_10

#define ADC_CH_BUS5V        ADC_CHANNEL_11

#define ADC_CH_BUS3V3       ADC_CHANNEL_9


#define ADC_VREF            3.3f


#define ADC_RATIO_PACK      11.0f

#define ADC_RATIO_BUS12V    4.03f

#define ADC_RATIO_BUS5V     2.0f

#define ADC_RATIO_BUS3V3    1.0f


---

# IMU Register Configuration


## LSM6DSV320XTR Settings


| Parameter        | Value |
|-----------------|-------|
| Gyro ODR        | 0x0A |
| Accel ODR       | 0x0A |
| Gyro Full Scale | 0x04 |
| Accel Full Scale| 0x03 |
| Gyro LPF        | 0x02 |
| Gyro Scale      | 0.0700 |
| Accel Scale     | 0.000488 |


## Configuration Definitions


#define IMU_GYRO_ODR_CODE   0x0Au

#define IMU_ACCEL_ODR_CODE  0x0Au


#define IMU_GYRO_FS_CODE    0x04u

#define IMU_ACCEL_FS_CODE   0x03u


#define IMU_GYRO_LPF_CODE   0x02u


#define IMU_GYRO_SCALE      0.0700f

#define IMU_ACCEL_SCALE     0.000488f


---

# Flight Controller Configuration


# RC / CRSF Configuration


## Channel Mapping


| Channel | Index | Function        |
|---------|-------|-----------------|
| CH1     | 0     | Roll            |
| CH2     | 1     | Pitch           |
| CH3     | 2     | Throttle        |
| CH4     | 3     | Yaw             |
| CH5     | 4     | Arm Switch      |
| CH6     | 5     | Flight Mode     |
| CH7     | 6     | Altitude Hold   |


## Configuration Definitions


#define RC_CH_ROLL          0

#define RC_CH_PITCH         1

#define RC_CH_THROTTLE      2

#define RC_CH_YAW           3

#define RC_CH_ARM           4

#define RC_CH_MODE          5

#define RC_CH_ALT_HOLD      6


---

# CRSF Input Configuration


| Parameter        | Value |
|-----------------|-------|
| Minimum         | 172   |
| Centre          | 992   |
| Maximum         | 1811  |
| Deadband        | 0.02  |
| Failsafe Timeout| 300 ms |


## Configuration Definitions


#define RC_MIN              172u

#define RC_MID              992u

#define RC_MAX              1811u


#define RC_DEADBAND         0.02f

#define CRSF_FAILSAFE_MS    300U


---

# Control Loop Rates


| Task                         | Frequency |
|-----------------------------|-----------|
| Attitude / Rate Controller  | 3840 Hz   |
| Velocity Controller         | 100 Hz    |
| Position Controller         | 50 Hz     |
| Magnetometer                | 100 Hz    |
| Barometer                   | 50 Hz     |
| GPS                         | 10 Hz     |
| INA226 Monitoring            | 10 Hz     |


## Configuration Definitions


#define CTRL_LOOP_HZ        3840U

#define VEL_LOOP_HZ         100U

#define POS_LOOP_HZ         50U


#define MAG_HZ              100U

#define BARO_HZ             50U

#define GPS_HZ              10U

#define INA226_HZ           10U


---

# FPV Flight Envelope


## Maximum Rates


| Parameter        | Value |
|-----------------|-------|
| Maximum Roll Rate  | 800 deg/s |
| Maximum Pitch Rate | 800 deg/s |
| Maximum Yaw Rate   | 600 deg/s |


## Expo


| Axis  | Expo |
|-------|------|
| Roll  | 0.40 |
| Pitch | 0.40 |
| Yaw   | 0.20 |


## Configuration Definitions


#define FPV_MAX_ROLL_RATE   800.0f

#define FPV_MAX_PITCH_RATE  800.0f

#define FPV_MAX_YAW_RATE    600.0f


#define FPV_ROLL_EXPO       0.40f

#define FPV_PITCH_EXPO      0.40f

#define FPV_YAW_EXPO        0.20f


---

# Camera / Cinematic Flight Envelope


## Attitude Limits


| Parameter | Value |
|-----------|-------|
| Maximum Roll Angle  | 40 deg |
| Maximum Pitch Angle | 40 deg |


## Maximum Rates


| Axis  | Maximum Rate |
|-------|--------------|
| Roll  | 220 deg/s    |
| Pitch | 220 deg/s    |
| Yaw   | 160 deg/s    |


## Expo


| Axis  | Expo |
|-------|------|
| Roll  | 0.15 |
| Pitch | 0.15 |
| Yaw   | 0.10 |


## Velocity Limits


| Parameter | Value |
|-----------|-------|
| Horizontal Velocity | 5 m/s |
| Vertical Velocity   | 2 m/s |


## Configuration Definitions


#define CAM_MAX_ROLL_ANGLE  40.0f

#define CAM_MAX_PITCH_ANGLE 40.0f


#define CAM_MAX_ROLL_RATE   220.0f

#define CAM_MAX_PITCH_RATE  220.0f

#define CAM_MAX_YAW_RATE    160.0f


#define CAM_ROLL_EXPO       0.15f

#define CAM_PITCH_EXPO      0.15f

#define CAM_YAW_EXPO        0.10f


#define CAM_MAX_VEL_HORIZ   5.0f

#define CAM_MAX_VEL_VERT    2.0f


---

# PID Configuration


# FPV Rate Controller


| Axis  | P      | I      | D       |
|-------|--------|--------|---------|
| Roll  | 0.0450 | 0.0400 | 0.0012  |
| Pitch | 0.0470 | 0.0400 | 0.0014  |
| Yaw   | 0.0600 | 0.0300 | 0.0000  |


## Configuration Definitions


#define PID_FPV_ROLL_RATE_P   0.045f
#define PID_FPV_ROLL_RATE_I   0.040f
#define PID_FPV_ROLL_RATE_D   0.0012f


#define PID_FPV_PITCH_RATE_P  0.047f
#define PID_FPV_PITCH_RATE_I  0.040f
#define PID_FPV_PITCH_RATE_D  0.0014f


#define PID_FPV_YAW_RATE_P    0.060f
#define PID_FPV_YAW_RATE_I    0.030f
#define PID_FPV_YAW_RATE_D    0.000f


---

# Cinematic Rate Controller


| Axis  | P      | I      | D       |
|-------|--------|--------|---------|
| Roll  | 0.0300 | 0.0250 | 0.0008  |
| Pitch | 0.0320 | 0.0250 | 0.0009  |
| Yaw   | 0.0400 | 0.0200 | 0.0000  |


## Configuration Definitions


#define PID_CAM_ROLL_RATE_P   0.030f
#define PID_CAM_ROLL_RATE_I   0.025f
#define PID_CAM_ROLL_RATE_D   0.0008f


#define PID_CAM_PITCH_RATE_P  0.032f
#define PID_CAM_PITCH_RATE_I  0.025f
#define PID_CAM_PITCH_RATE_D  0.0009f


#define PID_CAM_YAW_RATE_P    0.040f
#define PID_CAM_YAW_RATE_I    0.020f
#define PID_CAM_YAW_RATE_D    0.000f


---

# Attitude Controller


| Parameter | Gain |
|-----------|------|
| Roll      | 7.0  |
| Pitch     | 7.0  |
| Heading   | 3.5  |


## Configuration Definitions


#define PID_CAM_ROLL_ANG_P    7.0f

#define PID_CAM_PITCH_ANG_P   7.0f

#define PID_CAM_YAW_HDG_P     3.5f


---

# Velocity Controller


## Horizontal Velocity


| Gain          | Value |
|---------------|-------|
| P             | 1.5   |
| I             | 0.3   |
| D             | 0.0   |
| I Limit       | 5.0   |
| Output Limit  | 9.81  |


## Vertical Velocity


| Gain          | Value |
|---------------|-------|
| P             | 2.5   |
| I             | 0.5   |
| D             | 0.5   |
| I Limit       | 0.3   |
| Output Limit  | 0.4   |


## Configuration Definitions


#define PID_VEL_HORIZ_P       1.5f

#define PID_VEL_HORIZ_I       0.3f

#define PID_VEL_HORIZ_D       0.0f

#define PID_VEL_HORIZ_I_MAX   5.0f

#define PID_VEL_HORIZ_OUT_MAX 9.81f


#define PID_VEL_VERT_P        2.5f

#define PID_VEL_VERT_I        0.5f

#define PID_VEL_VERT_D        0.5f

#define PID_VEL_VERT_I_MAX    0.3f

#define PID_VEL_VERT_OUT_MAX  0.4f


---

# Altitude Controller


| Gain | Value |
|------|-------|
| P    | 3.0   |
| I    | 0.5   |
| D    | 1.5   |


## Configuration Definitions


#define PID_ALT_P             3.0f

#define PID_ALT_I             0.5f

#define PID_ALT_D             1.5f


---

# Position Controller


| Parameter      | Value |
|----------------|-------|
| P              | 0.80  |
| I              | 0.05  |
| D              | 0.20  |
| Output Limit   | 3.0   |


## Configuration Definitions


#define PID_POS_P             0.8f

#define PID_POS_I             0.05f

#define PID_POS_D             0.20f

#define PID_POS_OUT_MAX       3.0f


---

# Safety Configuration


## Flight Safety Limits


| Parameter              | Value |
|------------------------|-------|
| Low Battery Voltage    | 10.8 V |
| Critical Battery Voltage | 10.2 V |
| Maximum Tilt Angle     | 70 deg |
| GPS Fence Radius       | 300 m |


## Configuration Definitions


#define SAFETY_LOW_BATT_V     10.8f

#define SAFETY_CRIT_BATT_V    10.2f

#define SAFETY_MAX_TILT_DEG   70.0f

#define SAFETY_GPS_FENCE_M    300.0f


---

# Error Flags


## Error Flag Allocation


| Flag              | Value  | Description              |
|-------------------|--------|--------------------------|
| FC_ERR_NONE       | 0x000  | No Error                 |
| FC_ERR_IMU        | 0x001  | IMU Failure              |
| FC_ERR_RC_LOSS    | 0x002  | Radio Link Lost          |
| FC_ERR_LOW_BATT   | 0x004  | Battery Low              |
| FC_ERR_TILT_LIMIT | 0x008  | Excessive Tilt           |
| FC_ERR_GPS_FENCE  | 0x010  | Geofence Triggered       |
| FC_ERR_SD         | 0x020  | SD Card Failure          |
| FC_ERR_MAG        | 0x040  | Magnetometer Failure     |
| FC_ERR_BARO       | 0x080  | Barometer Failure        |
| FC_ERR_MOTOR_LOSS | 0x100  | Motor Failure            |
| FC_ERR_BUS1_OC    | 0x200  | 12 V Overcurrent         |
| FC_ERR_BUS2_OC    | 0x400  | 5 V Overcurrent          |
| FC_ERR_BUS3_OC    | 0x800  | 3.3 V Overcurrent        |


## Configuration Definitions


#define FC_ERR_NONE           0x000U

#define FC_ERR_IMU            0x001U

#define FC_ERR_RC_LOSS        0x002U

#define FC_ERR_LOW_BATT       0x004U

#define FC_ERR_TILT_LIMIT     0x008U

#define FC_ERR_GPS_FENCE      0x010U

#define FC_ERR_SD             0x020U

#define FC_ERR_MAG            0x040U

#define FC_ERR_BARO           0x080U

#define FC_ERR_MOTOR_LOSS     0x100U

#define FC_ERR_BUS1_OC        0x200U

#define FC_ERR_BUS2_OC        0x400U

#define FC_ERR_BUS3_OC        0x800U


---

# FreeRTOS Configuration


## Task Stack Allocation


| Task             | Stack Size (Words) |
|------------------|--------------------|
| Control Loop     | 768                |
| Safety           | 256                |
| Barometer        | 256                |
| Magnetometer     | 256                |
| GPS              | 384                |
| MAVLink          | 512                |
| Logger           | 512                |
| CRSF             | 256                |
| Power Monitor    | 256                |


## Configuration Definitions


#define STACK_CTRL_LOOP       768

#define STACK_SAFETY          256

#define STACK_BARO            256

#define STACK_MAG             256

#define STACK_GPS             384

#define STACK_MAVLINK         512

#define STACK_LOGGER          512

#define STACK_CRSF            256

#define STACK_POWER_MON       256


---

# Logging Configuration


| Parameter          | Value |
|-------------------|-------|
| SD Flush Interval | 500 ms |


## Configuration Definitions


#define LOG_SD_FLUSH_MS       500


---

# Watchdog Configuration


| Parameter                  | Value |
|---------------------------|-------|
| Watchdog Type             | Independent Watchdog (IWDG) |
| Timeout                   | 32 ms |


## Configuration Definitions


#define IWDG_TIMEOUT_MS       32


---

# Final Configuration Summary


| Category              | Value |
|-----------------------|-------|
| MCU                   | STM32F722RET6 @ 216 MHz |
| IMU                   | LSM6DSV320XTR |
| IMU Rate              | 3840 Hz |
| Attitude Loop         | 3840 Hz |
| Velocity Loop         | 100 Hz |
| Position Loop         | 50 Hz |
| GPS                   | Matek M10Q-5883 |
| GPS Rate              | 10 Hz |
| Magnetometer          | QMC5883L |
| Barometer             | MS5611 |
| Motor Protocol        | Bidirectional DSHOT300 |
| Radio Protocol        | CRSF / ELRS |
| Storage               | SPI microSD |
| Telemetry             | USB CDC + CRSF |
| RTOS                  | FreeRTOS |
