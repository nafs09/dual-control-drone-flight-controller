
#pragma once
#include <stdint.h>
#include "../core/shared_state.hpp"

struct UART_HandleTypeDef;

namespace FC { namespace Drivers {

/**
 * @brief  Bidirectional DSHOT300 — UART Half-Duplex
 *
 * MOTOR → UART ASSIGNMENT (STM32F722RET6 LQFP64)
 * ────────────────────────────────────────────────
 *   Motor 1 → UART4   PA0  pin 14  APB1 54 MHz → 3 Mbaud = 54M/18 exact
 *   Motor 2 → UART5   PC12 pin 53  APB1         → 3 Mbaud exact
 *   Motor 3 → USART3  PB10 pin 29  APB1         → 3 Mbaud exact
 *   Motor 4 → USART6  PC6  pin 37  APB2 108MHz  → 3 Mbaud = 108M/36 exact
 *
 * BUG FIX #PINS: Previous versions referenced UART7 for Motor 4.
 *   UART7 pins are on Port E (PE7/PE8) which is NOT exposed in LQFP64.
 *   Correct assignment is USART6 (APB2) on PC6, which IS available.
 *
 * CUBEMX SETUP (repeat for each motor UART):
 *   Mode:              Asynchronous
 *   Baud rate:         3,000,000
 *   Data bits:         8, Stop bits: 1, No parity
 *   Half-duplex:       ENABLED  (CR3.HDSEL = 1)
 *   TX inversion:      ENABLED  (CR2.TXINV = 1)
 *   RX inversion:      ENABLED  (CR2.RXINV = 1)
 *   TX DMA:            DMA stream, byte width, memory-increment, normal
 *   RX DMA:            DMA stream, byte width, memory-increment, normal
 *   Global interrupt:  ENABLED
 *
 * HAL CALLBACK WIRING (in stm32f7xx_it.c or Src/main.c):
 *   extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* h) {
 *       if      (h == &huart4)  FC::Drivers::DShot::onTxComplete(0);
 *       else if (h == &huart5)  FC::Drivers::DShot::onTxComplete(1);
 *       else if (h == &huart3)  FC::Drivers::DShot::onTxComplete(2);
 *       else if (h == &huart6)  FC::Drivers::DShot::onTxComplete(3);
 *   }
 *   extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* h) {
 *       if      (h == &huart4)  FC::Drivers::DShot::onRxComplete(0);
 *       else if (h == &huart5)  FC::Drivers::DShot::onRxComplete(1);
 *       else if (h == &huart3)  FC::Drivers::DShot::onRxComplete(2);
 *       else if (h == &huart6)  { FC::Drivers::DShot::onRxComplete(3);
 *                                  FC::Drivers::DShot::updateAvgFreq(); }
 *   }
 *
 * ESC FIRMWARE:
 *   BLHeli_32: enable "Bidirectional DSHOT" in BLHeliSuite32.
 *   AM32:      enable "Bidirectional DSHOT" in AM32 Configurator.
 *   Protocol must match: DSHOT300.
 *
 * ENCODING (TX inverted, 8N1 at 3 Mbaud):
 *   DSHOT bit 1 → byte 0x01 → ~80% HIGH (T1H spec: 74.9%)
 *   DSHOT bit 0 → byte 0xF8 → ~40% HIGH (T0H spec: 37.4%)
 *
 * GCR RESPONSE:
 *   ESC replies ≥30 µs after last TX bit.
 *   3 bytes (24 bits captured, 21 used).
 *   GCR 5→4 bit decode → 16-bit word → period → eRPM.
 *   eRPM = 60,000,000 / (period_us × pole_pairs)
 */
class DShot {
public:
    static constexpr int  NUM_MOTORS   = 4;
    static constexpr int  FRAME_BYTES  = 16;
    static constexpr int  TRAIL_BYTES  = 2;
    static constexpr int  TX_BYTES     = FRAME_BYTES + TRAIL_BYTES;
    static constexpr int  RX_BYTES     = 3;
    static constexpr int  MOTOR_POLES  = 14;
    static constexpr int  POLE_PAIRS   = MOTOR_POLES / 2;
    static constexpr uint8_t BIT1_BYTE = 0x01u;
    static constexpr uint8_t BIT0_BYTE = 0xF8u;
    static constexpr float kF_DEFAULT  = 1.5e-8f;   /* N / RPM² — calibrate */

    static bool   init(UART_HandleTypeDef* huarts[4], float kF = kF_DEFAULT);
    static void   sendMotors(const uint16_t cmds[NUM_MOTORS]);
    static void   sendCommand(uint8_t motor_idx, uint8_t cmd, int times = 10);
    static void   kill();
    static void   idle();
    static void   onTxComplete(uint8_t motor_idx);
    static void   onRxComplete(uint8_t motor_idx);
    static void   updateAvgFreq();

    static float kF_;

private:
    static UART_HandleTypeDef* huarts_[NUM_MOTORS];
    static uint8_t txBuf_[NUM_MOTORS][TX_BYTES];
    static uint8_t rxBuf_[NUM_MOTORS][RX_BYTES];

    static void    encodeFrame(uint8_t* dst, uint16_t value, bool telemetry);
    static bool    decodeGCR(const uint8_t* rx3, uint16_t& out_period);
    static const uint8_t kGcrDecode[32];
};

}} // FC::Drivers
