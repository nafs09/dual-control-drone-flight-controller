#ifndef FC_CONFIG_H
#define FC_CONFIG_H

/* ============================================================
 *  Drone Flight Controller — Master Configuration
 *
 *  MCU  : STM32F722RET6 @ 216 MHz (Cortex-M7 + single-precision FPU)
 *  Pkg  : LQFP64 (64-pin)
 *
 *  PERIPHERAL COUNT — LQFP64 HARD LIMITS:
 *    USART/UART : 6 total (USART1,2,3,6 + UART4,5)
 *                 UART7/8 exist on chip but on PE/PF not exposed in 64-pin
 *    SPI        : 3 (SPI1,2,3)
 *    I2C        : 3 (I2C1,2,3)
 *
 *  ALLOCATION:
 *    USART1 → GPS         (PA9 TX / PA10 RX)
 *    USART2 → CRSF/ELRS   (PA2 TX / PA3 RX)
 *    USART3 → Motor 3     (PB10  half-duplex, was MAVLink — moved to USB CDC)
 *    USART6 → Motor 4     (PC6   half-duplex)
 *    UART4  → Motor 1     (PA0   half-duplex)
 *    UART5  → Motor 2     (PC12  half-duplex)
 *    SPI1   → IMU + Baro  (PA5 SCK / PA6 MISO / PA7 MOSI)
 *    SPI2   → SD card     (PB13 SCK / PB14 MISO / PB15 MOSI)
 *    I2C1   → Magnetometer(PB6 SCL / PB7 SDA)
 *    USB FS → MAVLink CDC (PA11 D- / PA12 D+)
 *
 *  BUG FIX #PINS:
 *    PA5 was previously assigned as STATUS_LED. PA5 = SPI1_SCK — fatal conflict.
 *    Status LED moved to PB0. Error LED to PB12. Buzzer to PC7.
 * ============================================================ */

/* ---------- System ----------------------------------------- */
#define FC_SYSCLK_HZ        216000000UL
#define FC_APB1_TIM_HZ      108000000UL   /* TIM2-7,12-14 after TIMPRE=1 */
#define FC_APB2_TIM_HZ      216000000UL   /* TIM1,8,9,10,11 */
#define FC_APB1_PCLK_HZ      54000000UL
#define FC_APB2_PCLK_HZ     108000000UL

/* ============================================================
 *  PIN DEFINITIONS  (LQFP64, verified against DS11853 Rev 9)
 *
 *  Numbering: pin 1 = bottom-left (package notch), counterclockwise.
 *  Left side (1-16), bottom (17-32), right (33-48), top (49-64).
 * ============================================================ */

/* --- Left side (pins 1-16) --------------------------------- */
/* Pin  1: VBAT — 100 nF to GND, optional 3 V coin cell       */
/* Pin  2: PC13 — free GPIO (RTC wake-up capable)             */
/* Pin  3: PC14 — OSC32_IN  (32.768 kHz XTAL)                 */
/* Pin  4: PC15 — OSC32_OUT                                    */
/* Pin  5: PH0  — OSC_IN    (8 MHz HSE XTAL)                  */
/* Pin  6: PH1  — OSC_OUT                                      */
/* Pin  7: NRST — 100 nF to GND, debug connector              */
/* Pin  8: PC0  — ADC1_IN10 → Current sensor input            */
/* Pin  9: PC1  — GPIO output → SD card /CS                   */
/* Pin 10: PC2  — GPIO free                                    */
/* Pin 11: PC3  — GPIO free                                    */
/* Pin 12: VSSA — star GND, short to pin 13                   */
/* Pin 13: VDDA — 1 µF + 10 nF to VSSA                       */
/* Pin 14: PA0  — UART4_TX (AF8) → Motor 1 half-duplex        */
/* Pin 15: PA1  — ADC1_IN1 → Battery voltage divider          */
/* Pin 16: PA2  — USART2_TX (AF7) → CRSF TX                  */

/* --- Bottom side (pins 17-32) ------------------------------ */
/* Pin 17: PA3  — USART2_RX (AF7) → CRSF RX                  */
/* Pin 18: VSS                                                 */
/* Pin 19: VDD                                                 */
/* Pin 20: PA4  — GPIO output → IMU /CS (SPI1)                */
/* Pin 21: PA5  — SPI1_SCK  (AF5)                             */
/* Pin 22: PA6  — SPI1_MISO (AF5)                             */
/* Pin 23: PA7  — SPI1_MOSI (AF5)                             */
/* Pin 24: PC4  — GPIO input EXTI4 → IMU DRDY (INT1)         */
/* Pin 25: PC5  — GPIO output → Baro /CS (SPI1)               */
/* Pin 26: PB0  — GPIO output → Status LED (GREEN)            */
/* Pin 27: PB1  — ADC1_IN9  → RSSI / spare ADC               */
/* Pin 28: PB2  — BOOT1, tie to GND                           */
/* Pin 29: PB10 — USART3_TX (AF7) → Motor 3 half-duplex      */
/* Pin 30: VCAP_1 — 4.7 µF ESR<0.2 Ω to GND (NOT GPIO)      */
/* Pin 31: VSS                                                 */
/* Pin 32: VDD                                                 */

/* --- Right side (pins 33-48) ------------------------------- */
/* Pin 33: PB12 — GPIO output → Error LED (RED)               */
/* Pin 34: PB13 — SPI2_SCK  (AF5) → SD card                  */
/* Pin 35: PB14 — SPI2_MISO (AF5) → SD card                  */
/* Pin 36: PB15 — SPI2_MOSI (AF5) → SD card                  */
/* Pin 37: PC6  — USART6_TX (AF8) → Motor 4 half-duplex      */
/* Pin 38: PC7  — GPIO output → Buzzer (TIM8_CH2 AF3 for PWM)*/
/* Pin 39: PC8  — GPIO free (TIM8_CH3 AF3, camera trigger)    */
/* Pin 40: PC9  — GPIO free                                    */
/* Pin 41: PA8  — GPIO / TIM1_CH1 (AF1) → camera shutter      */
/* Pin 42: PA9  — USART1_TX (AF7) → GPS TX                   */
/* Pin 43: PA10 — USART1_RX (AF7) → GPS RX                   */
/* Pin 44: PA11 — USB_OTG_FS_DM → MAVLink USB CDC (D-)        */
/* Pin 45: PA12 — USB_OTG_FS_DP → MAVLink USB CDC (D+)        */
/* Pin 46: PA13 — SWDIO (debug), leave accessible              */
/* Pin 47: VCAP_2 — 100 nF to VDD (no second regulator cap)   */
/* Pin 48: VDD                                                 */

/* --- Top side (pins 49-64) --------------------------------- */
/* Pin 49: PA14 — SWCLK (debug)                               */
/* Pin 50: PA15 — GPIO free (JTDI, remap if debug unneeded)   */
/* Pin 51: PC10 — GPIO free (UART4_TX alt, if rerouted)       */
/* Pin 52: PC11 — GPIO free                                    */
/* Pin 53: PC12 — UART5_TX (AF8) → Motor 2 half-duplex       */
/* Pin 54: PD2  — GPIO input EXTI2 → Mag data-ready (INT)    */
/* Pin 55: PB3  — GPIO free (JTDO)                            */
/* Pin 56: PB4  — GPIO free                                    */
/* Pin 57: PB5  — GPIO free                                    */
/* Pin 58: PB6  — I2C1_SCL (AF4) → QMC5883L                  */
/* Pin 59: PB7  — I2C1_SDA (AF4) → QMC5883L                  */
/* Pin 60: BOOT0 — tie GND, add pull-up button for DFU        */
/* Pin 61: PB8  — GPIO free (I2C1_SCL alt)                   */
/* Pin 62: PB9  — GPIO free (I2C1_SDA alt)                   */
/* Pin 63: VSS                                                 */
/* Pin 64: VDD                                                 */

/* ---- Named peripheral macros (match above table) ---------- */

/* IMU LSM6DSV320XTR — SPI1 */
#define IMU_SPI_HANDLE      hspi1
#define IMU_CS_PORT         GPIOA
#define IMU_CS_PIN          GPIO_PIN_4    /* Pin 20 */
#define IMU_DRDY_PORT       GPIOC
#define IMU_DRDY_PIN        GPIO_PIN_4    /* Pin 24 — EXTI4 */
#define IMU_DRDY_IRQn       EXTI4_IRQn

/* Barometer MS5611 — SPI1 (shared bus, separate CS) */
#define BARO_SPI_HANDLE     hspi1
#define BARO_CS_PORT        GPIOC
#define BARO_CS_PIN         GPIO_PIN_5    /* Pin 25 */

/* Magnetometer QMC5883L — I2C1 */
#define MAG_I2C_HANDLE      hi2c1
#define MAG_DRDY_PORT       GPIOD
#define MAG_DRDY_PIN        GPIO_PIN_2    /* Pin 54 — EXTI2 */

/* GPS — USART1 */
#define GPS_UART_HANDLE     huart1

/* CRSF/ELRS — USART2 @ 420 kbaud */
#define CRSF_UART_HANDLE    huart2

/* MAVLink — USB CDC (not a UART handle; use CDC_Transmit_FS()) */
/* No define needed; use USBD_CDC_SetTxBuffer / CDC_Transmit_FS */

/* SD card — SPI2 */
#define SD_SPI_HANDLE       hspi2
#define SD_CS_PORT          GPIOC
#define SD_CS_PIN           GPIO_PIN_1    /* Pin 9 */

/* Motor UART handles — half-duplex bidir DSHOT300 @ 3 Mbaud */
/* BUG FIX #PINS: Motor 4 was UART7 (not available in LQFP64).
 *                Now correctly assigned to USART3 on PB10.     */
#define MOTOR1_UART_HANDLE  huart4    /* PA0,  pin 14, UART4   */
#define MOTOR2_UART_HANDLE  huart5    /* PC12, pin 53, UART5   */
#define MOTOR3_UART_HANDLE  huart3    /* PB10, pin 29, USART3  */
#define MOTOR4_UART_HANDLE  huart6    /* PC6,  pin 37, USART6  */

/* Status indicators */
/* BUG FIX #PINS: PA5 was STATUS_LED but PA5 = SPI1_SCK. Moved to PB0. */
#define STATUS_LED_PORT     GPIOB
#define STATUS_LED_PIN      GPIO_PIN_0    /* Pin 26 — green LED */
#define ERROR_LED_PORT      GPIOB
#define ERROR_LED_PIN       GPIO_PIN_12   /* Pin 33 — red LED   */
#define BUZZER_PORT         GPIOC
#define BUZZER_PIN          GPIO_PIN_7    /* Pin 38 */

/* Battery + current ADC */
#define BATT_VOLT_ADC_CH    ADC_CHANNEL_1    /* PA1, pin 15 */
#define BATT_CURR_ADC_CH    ADC_CHANNEL_10   /* PC0, pin 8  */

/* ---------- DSHOT300 via UART half-duplex ------------------ */
/* DSHOT300 bit period = 3.333 µs
 * UART baud = 3,000,000 (one byte per DSHOT bit)
 * APB1 UARTs (UART3,4,5): 54 MHz / 18 = 3,000,000 exact
 * APB2 UARTs (USART6):    108 MHz / 36 = 3,000,000 exact      */
#define DSHOT_BAUD          3000000U
#define DSHOT_BIT1_BYTE     0x01u    /* ~80% HIGH (TX inverted) */
#define DSHOT_BIT0_BYTE     0xF8u    /* ~40% HIGH (TX inverted) */
#define MOTOR_POLES         14       /* typical 2306/2407 motor */
#define MOTOR_POLE_PAIRS    (MOTOR_POLES / 2)
#define MOTOR_IDLE_FPV      150U
#define MOTOR_IDLE_CAMERA   80U
#define MOTOR_MIN_THROTTLE  48U
#define MOTOR_MAX_THROTTLE  2047U

/* ---------- IMU (LSM6DSV320XTR) --------------------------- */
#define IMU_GYRO_ODR_CODE   0x0Au    /* 3840 Hz */
#define IMU_ACCEL_ODR_CODE  0x0Au
#define IMU_GYRO_FS_CODE    0x04u    /* ±2000 dps */
#define IMU_ACCEL_FS_CODE   0x03u    /* ±16 g */
#define IMU_GYRO_LPF_CODE   0x02u    /* ~ODR/10 */
#define IMU_GYRO_SCALE      0.0700f                        /* deg/LSB  */
#define IMU_ACCEL_SCALE     0.000488f                      /* g/LSB    */

/* ---------- CRSF ------------------------------------------ */
#define RC_CH_ROLL          0
#define RC_CH_PITCH         1
#define RC_CH_THROTTLE      2
#define RC_CH_YAW           3
#define RC_CH_ARM           4
#define RC_CH_MODE          5   /* low=FPV, high=Camera */
#define RC_CH_ALT_HOLD      6
#define RC_MIN              172u
#define RC_MID              992u
#define RC_MAX              1811u
#define RC_DEADBAND         0.02f
#define CRSF_FAILSAFE_MS    300U

/* ---------- Control rates ---------------------------------- */
#define CTRL_LOOP_HZ        3840U   /* IMU-paced via DRDY      */
#define VEL_LOOP_HZ         100U    /* velocity controller      */
#define POS_LOOP_HZ         50U     /* position controller      */
#define MAG_HZ              100U
#define BARO_HZ             50U
#define GPS_HZ              10U

/* ---------- Flight envelope -------------------------------- */
#define FPV_MAX_ROLL_RATE   800.0f  /* deg/s */
#define FPV_MAX_PITCH_RATE  800.0f
#define FPV_MAX_YAW_RATE    600.0f
#define FPV_ROLL_EXPO       0.40f
#define FPV_PITCH_EXPO      0.40f
#define FPV_YAW_EXPO        0.20f

#define CAM_MAX_ROLL_ANGLE  40.0f   /* degrees */
#define CAM_MAX_PITCH_ANGLE 40.0f
#define CAM_MAX_ROLL_RATE   220.0f
#define CAM_MAX_PITCH_RATE  220.0f
#define CAM_MAX_YAW_RATE    160.0f
#define CAM_ROLL_EXPO       0.15f
#define CAM_PITCH_EXPO      0.15f
#define CAM_YAW_EXPO        0.10f

/* Camera / velocity mode stick → velocity mapping */
#define CAM_MAX_VEL_HORIZ   5.0f    /* m/s max horizontal velocity from sticks */
#define CAM_MAX_VEL_VERT    2.0f    /* m/s max vertical velocity from sticks   */

/* ---------- PID gains — FPV rate controller --------------- */
#define PID_FPV_ROLL_RATE_P     0.045f
#define PID_FPV_ROLL_RATE_I     0.040f
#define PID_FPV_ROLL_RATE_D     0.0012f
#define PID_FPV_PITCH_RATE_P    0.047f
#define PID_FPV_PITCH_RATE_I    0.040f
#define PID_FPV_PITCH_RATE_D    0.0014f
#define PID_FPV_YAW_RATE_P      0.060f
#define PID_FPV_YAW_RATE_I      0.030f
#define PID_FPV_YAW_RATE_D      0.000f

/* ---------- PID gains — Camera rate controller ------------ */
#define PID_CAM_ROLL_RATE_P     0.030f
#define PID_CAM_ROLL_RATE_I     0.025f
#define PID_CAM_ROLL_RATE_D     0.0008f
#define PID_CAM_PITCH_RATE_P    0.032f
#define PID_CAM_PITCH_RATE_I    0.025f
#define PID_CAM_PITCH_RATE_D    0.0009f
#define PID_CAM_YAW_RATE_P      0.040f
#define PID_CAM_YAW_RATE_I      0.020f
#define PID_CAM_YAW_RATE_D      0.000f

/* ---------- PID gains — Attitude (angle) controller ------- */
#define PID_CAM_ROLL_ANG_P      7.0f
#define PID_CAM_PITCH_ANG_P     7.0f
#define PID_CAM_YAW_HDG_P       3.5f

/* ---------- PID gains — Velocity controller (NEW) --------- */
/*  Output: desired NED acceleration [m/s²] from velocity error [m/s]
 *  Then converted to body tilt angle: θ = a / g
 *  Horizontal only — vertical handled by altitude PID separately.
 *
 *  Tuning guide (see process.md):
 *    Start Kp = 1.0, increase until oscillation, then halve.
 *    Add Ki = Kp/10, watch integrator behaviour in wind.
 *    Kd usually 0 — EKF velocity is already smooth.
 *    Max tilt: limit so position control never commands dangerous angles.  */
#define PID_VEL_HORIZ_P         1.5f     /* (m/s²) per (m/s) error  */
#define PID_VEL_HORIZ_I         0.3f     /* (m/s²)/(m/s·s)          */
#define PID_VEL_HORIZ_D         0.0f
#define PID_VEL_HORIZ_I_MAX     5.0f     /* m/s² — prevent windup   */
#define PID_VEL_HORIZ_OUT_MAX   9.81f    /* m/s² ≡ 45° max tilt     */
#define PID_VEL_VERT_P          2.5f     /* throttle per (m/s) error */
#define PID_VEL_VERT_I          0.5f
#define PID_VEL_VERT_D          0.5f
#define PID_VEL_VERT_I_MAX      0.3f
#define PID_VEL_VERT_OUT_MAX    0.4f     /* throttle increment [0..1]*/

/* ---------- PID gains — Altitude hold --------------------- */
#define PID_ALT_P               3.0f
#define PID_ALT_I               0.5f
#define PID_ALT_D               1.5f

/* ---------- PID gains — Position hold --------------------- */
#define PID_POS_P               0.8f
#define PID_POS_I               0.05f
#define PID_POS_D               0.20f
#define PID_POS_OUT_MAX         3.0f     /* m/s max velocity from position error */

/* ---------- Safety ---------------------------------------- */
#define SAFETY_LOW_BATT_V       10.8f
#define SAFETY_CRIT_BATT_V      10.2f
#define SAFETY_MAX_TILT_DEG     70.0f
#define SAFETY_GPS_FENCE_M      300.0f

/* Error flag bitmask */
#define FC_ERR_NONE             0x00U
#define FC_ERR_IMU              0x01U
#define FC_ERR_RC_LOSS          0x02U
#define FC_ERR_LOW_BATT         0x04U
#define FC_ERR_TILT_LIMIT       0x08U
#define FC_ERR_GPS_FENCE        0x10U
#define FC_ERR_SD               0x20U
#define FC_ERR_MAG              0x40U
#define FC_ERR_BARO             0x80U
#define FC_ERR_MOTOR_LOSS       0x100U   /* Motor telemetry indicates failure */

/* ---------- RTOS stack sizes (words) ---------------------- */
#define STACK_CTRL_LOOP         768
#define STACK_SAFETY            256
#define STACK_BARO              256
#define STACK_MAG               384
#define STACK_GPS               384
#define STACK_MAVLINK           512
#define STACK_LOGGER            512
#define STACK_CRSF              256

/* ---------- Logging --------------------------------------- */
#define LOG_QUEUE_DEPTH         64
#define LOG_SD_FLUSH_MS         500

/* ---------- LIDAR (future) -------------------------------- */
/* Uncomment when chosen:
 * #define LIDAR_ENABLED
 * #define LIDAR_UART_HANDLE huartX
 * #define LIDAR_MAX_RANGE_M 12.0f  */

/* ---------- Watchdog -------------------------------------- */
#define IWDG_TIMEOUT_MS         32

#endif /* FC_CONFIG_H */
