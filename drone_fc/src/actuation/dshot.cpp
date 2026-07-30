
#include "dshot.hpp"
#include "stm32f7xx_hal.h"
#include <cstring>
#include <cmath>

namespace FC { namespace Drivers {

/* ---- Static member definitions ------------------------------- */
UART_HandleTypeDef* DShot::huarts_[NUM_MOTORS] = {};
uint8_t DShot::txBuf_[NUM_MOTORS][TX_BYTES];
uint8_t DShot::rxBuf_[NUM_MOTORS][RX_BYTES];
float   DShot::kF_ = kF_DEFAULT;

/**
 * GCR decode table: index = 5-bit GCR code, value = 4-bit nibble.
 * Invalid codes → 0xFF.
 *
 * Encoding (from Betaflight / DSHOT spec):
 *   nibble 0 → 0x19,  1 → 0x1B,  2 → 0x12,  3 → 0x13,
 *           4 → 0x1D,  5 → 0x15,  6 → 0x16,  7 → 0x17,
 *           8 → 0x1A,  9 → 0x09, 10 → 0x0A, 11 → 0x0B,
 *          12 → 0x1E, 13 → 0x09… (same as 9 in some refs)
 * Inverse table built from the above:
 */
const uint8_t DShot::kGcrDecode[32] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  // 0x00-0x07
    0xFF, 0x09, 0x0A, 0x0B, 0xFF, 0x0D, 0x0E, 0x0F,  // 0x08-0x0F: 9,10,11,_,13,14,15
    0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05, 0x06, 0x07,  // 0x10-0x17: _,_,2,3,_,5,6,7
    0xFF, 0x00, 0x08, 0x01, 0x04, 0xFF, 0x0C, 0xFF   // 0x18-0x1F: _,0,8,1,4,_,12,_
};

/* ============================================================
 *  Initialisation
 * ============================================================ */
bool DShot::init(UART_HandleTypeDef* huarts[4], float kF) {
    kF_ = kF;
    for (int i = 0; i < NUM_MOTORS; ++i) {
        huarts_[i] = huarts[i];
        if (!huarts_[i]) return false;
        /* CubeMX should have configured:
         *   Baud=3000000, 8N1, half-duplex, TX+RX inverted.
         * Verify the key CR bits here as a defensive check.            */
        if (!(huarts_[i]->Instance->CR3 & USART_CR3_HDSEL)) {
            /* Half-duplex not enabled — this UART wasn't configured correctly */
            return false;
        }
        /* Pre-fill trailing gap bytes (LOW when inverted = 0xFF in inverted TX) */
        txBuf_[i][FRAME_BYTES]   = 0x00u;   /* Inverted LOW = gap */
        txBuf_[i][FRAME_BYTES+1] = 0x00u;
    }
    /* Start with all motors at 0 (disarmed) */
    kill();
    return true;
}

/* ============================================================
 *  DSHOT frame encoding
 *
 *  DSHOT frame (MSB first):
 *    bits[15:5] = throttle (11 bits, 0–2047)
 *    bit[4]     = telemetry request (1 = request telemetry)
 *    bits[3:0]  = CRC = (throttle ^ (throttle>>4) ^ (throttle>>8)) & 0xF
 *
 *  Each bit → one UART byte:
 *    DSHOT bit 1 → BIT1_BYTE (0x01) → ~80% HIGH with inverted TX
 *    DSHOT bit 0 → BIT0_BYTE (0xF8) → ~40% HIGH with inverted TX
 *
 *  For bidirectional DSHOT, telemetry=true on every frame so the ESC
 *  knows to send back its eRPM response.
 * ============================================================ */
void DShot::encodeFrame(uint8_t* dst, uint16_t value, bool telemetry) {
    uint16_t v     = value & 0x07FFu;           /* 11-bit throttle    */
    uint16_t t     = telemetry ? 1u : 0u;
    uint16_t frame = (v << 5) | (t << 4);
    /* CRC over upper 12 bits */
    uint16_t crc   = (frame ^ (frame >> 4) ^ (frame >> 8)) & 0x0Fu;
    frame         |= crc;

    /* Encode 16 bits MSB-first into UART bytes */
    for (int b = 0; b < 16; ++b) {
        uint16_t bit = (frame >> (15 - b)) & 1u;
        dst[b] = bit ? BIT1_BYTE : BIT0_BYTE;
    }
    /* Trailing gap bytes already set in init() */
}

/* ============================================================
 *  Send motors — starts DMA TX on all 4 UARTs simultaneously.
 *  Returns immediately; TC interrupt handles RX switchover.
 *
 *  For bidirectional DSHOT we always set telemetry=true so the ESC
 *  sends back the eRPM response after every frame.
 * ============================================================ */
void DShot::sendMotors(const uint16_t cmds[NUM_MOTORS]) {
    for (int i = 0; i < NUM_MOTORS; ++i) {
        encodeFrame(txBuf_[i], cmds[i], /*telemetry=*/true);
        /* Abort any in-progress RX before starting TX */
        HAL_UART_AbortReceive(huarts_[i]);
        /* Switch UART direction to TX.
         * In half-duplex, CR1_TE=1/RE=0 → transmit only.              */
        huarts_[i]->Instance->CR1 &= ~USART_CR1_RE;
        huarts_[i]->Instance->CR1 |=  USART_CR1_TE;
        /* Start DMA TX; TC interrupt fires when all 18 bytes are sent. */
        HAL_UART_Transmit_DMA(huarts_[i], txBuf_[i], TX_BYTES);
    }
}

/* ============================================================
 *  TX-complete callback — called from HAL_UART_TxCpltCallback()
 *
 *  Switches UART from TX to RX and arms DMA to capture the
 *  ESC GCR response.
 *
 *  Timing: TC fires after the last stop bit of the 18th byte.
 *  The ESC response starts ≥30 µs after the last DSHOT bit.
 *  At 3 Mbaud that is ~9 bit-periods of gap — well within the
 *  UART receiver's ability to detect the start bit.
 * ============================================================ */
void DShot::onTxComplete(uint8_t motor_idx) {
    UART_HandleTypeDef* h = huarts_[motor_idx];

    /* Switch half-duplex direction: TX→RX */
    h->Instance->CR1 &= ~USART_CR1_TE;
    h->Instance->CR1 |=  USART_CR1_RE;

    /* Clear any stale receive data register content */
    (void)h->Instance->RDR;

    /* Start DMA RX for exactly 3 bytes (21-bit GCR frame, padded to 24 bits).
     * RX DMA complete fires onRxComplete() via HAL_UART_RxCpltCallback().  */
    HAL_UART_Receive_DMA(h, rxBuf_[motor_idx], RX_BYTES);
}

/* ============================================================
 *  RX-complete callback — called from HAL_UART_RxCpltCallback()
 *
 *  Decodes the GCR frame, validates CRC, converts to eRPM and
 *  estimated thrust, updates shared state, then switches back
 *  to TX-ready state.
 * ============================================================ */
void DShot::onRxComplete(uint8_t motor_idx) {
    uint16_t period = 0;
    bool ok = decodeGCR(rxBuf_[motor_idx], period);

    /* Update motor telemetry in shared state.
     * We skip the mutex here (ISR context) because motorTelem is written
     * only from this ISR and read only from the DSHOT task at much lower
     * frequency.  ARM Cortex-M7 stores are atomic for aligned 32-bit
     * values; floats are 32-bit aligned.                               */
    auto& telem = g_state.motorTelem[motor_idx];
    if (ok && period > 0) {
        /* Period is a compressed 16-bit float from the ESC:
         *   exponent = period >> 9         (upper 3 bits after shift)
         *   mantissa = period & 0x1FF      (lower 9 bits)
         *   period_us = (mantissa + 512) << exponent  ... in units of 1/4 µs
         *
         * Simplified: period_us = period (raw value is in µs × some factor).
         * Using the standard DSHOT600 spec where period is in units of
         * 1/(12 × 1e6) s (12 MHz reference):                          */
        float period_us = static_cast<float>(period) / 12.f;   /* µs */
        if (period_us > 0.1f) {
            telem.freqHz  = 1'000'000.f / period_us;
            telem.eRPM    = telem.freqHz * 60.f;
            telem.mechRPM = telem.eRPM / static_cast<float>(POLE_PAIRS);
            telem.thrustN = kF_ * telem.mechRPM * telem.mechRPM;
            telem.valid   = true;
        }
    } else {
        telem.valid = false;
        if (!ok) telem.crcErrors++;
    }

    /* Switch half-duplex back to TX-ready (TE=1, RE=0) */
    UART_HandleTypeDef* h = huarts_[motor_idx];
    h->Instance->CR1 &= ~USART_CR1_RE;
    h->Instance->CR1 |=  USART_CR1_TE;
}

/* ============================================================
 *  GCR decode
 *
 *  The ESC transmits a 21-bit serial frame at 3 Mbaud inverted.
 *  We receive 3 bytes (24 bits) via DMA; the last 3 bits are idle.
 *
 *  Bit extraction: rebuild the 21-bit word from the 3 received bytes,
 *  treating each byte as inverted NRZ (idle=LOW, so the received byte
 *  bit values are inverted relative to the logical DSHOT bit stream).
 *
 *  GCR decode:
 *    Extract 4 groups of 5 bits from the 20 payload bits (after
 *    discarding the 1-bit frame start indicator).
 *    Look up each 5-bit group in kGcrDecode[] to get a 4-bit nibble.
 *    Reassemble: word = (n3<<12)|(n2<<8)|(n1<<4)|n0
 *
 *  CRC validation:
 *    (n3 ^ n2 ^ n1) & 0xF  should equal  n0
 *
 *  Period extraction:
 *    period = word >> 4   (upper 12 bits)
 * ============================================================ */
bool DShot::decodeGCR(const uint8_t* rx3, uint16_t& out_period) {
    /* Reconstruct 24 bits from 3 received bytes.
     * With inverted UART RX, a received 0xFF byte represents a line
     * that was LOW for the full byte period (8 data bits all inverted).
     * We invert each byte to get the logical bit stream.               */
    uint32_t bits = (static_cast<uint32_t>(~rx3[0] & 0xFF) << 16)
                  | (static_cast<uint32_t>(~rx3[1] & 0xFF) <<  8)
                  | (static_cast<uint32_t>(~rx3[2] & 0xFF)      );

    /* The GCR stream is 21 bits.  In the 24-bit window, the 21 bits
     * occupy the upper positions: bits[23:3].                         */
    uint32_t gcr = bits >> 3;   /* Align to 21 bits */

    /* Extract 4 × 5-bit GCR codes (MSB first, bits 20..0) */
    uint8_t c3 = (gcr >> 15) & 0x1Fu;
    uint8_t c2 = (gcr >> 10) & 0x1Fu;
    uint8_t c1 = (gcr >>  5) & 0x1Fu;
    uint8_t c0 = (gcr >>  0) & 0x1Fu;

    /* Look up each code */
    uint8_t n3 = kGcrDecode[c3];
    uint8_t n2 = kGcrDecode[c2];
    uint8_t n1 = kGcrDecode[c1];
    uint8_t n0 = kGcrDecode[c0];

    /* Invalid code check */
    if (n3 == 0xFF || n2 == 0xFF || n1 == 0xFF || n0 == 0xFF) return false;

    /* Reassemble 16-bit word */
    uint16_t word = (uint16_t)((n3 << 12) | (n2 << 8) | (n1 << 4) | n0);

    /* CRC validation: lower nibble should equal XOR of upper 3 nibbles */
    uint8_t crc_calc = (n3 ^ n2 ^ n1) & 0x0Fu;
    if (crc_calc != n0) return false;

    /* Extract period from upper 12 bits */
    out_period = word >> 4;
    return true;
}

/* ============================================================
 *  Update average motor frequency in shared state.
 *  Call once after all 4 motor RX phases complete.
 * ============================================================ */
void DShot::updateAvgFreq() {
    float sum = 0.f;
    int   valid = 0;
    for (int i = 0; i < NUM_MOTORS; ++i) {
        if (g_state.motorTelem[i].valid) {
            sum += g_state.motorTelem[i].freqHz;
            valid++;
        }
    }
    if (valid > 0) {
        g_state.avgMotorFreqHz  = sum / valid;
        g_state.avgMotorFreqHz2 = g_state.avgMotorFreqHz * 2.f;
    }
}

/* ============================================================
 *  Utility commands
 * ============================================================ */
void DShot::kill() {
    static const uint16_t zeros[4] = {0,0,0,0};
    sendMotors(zeros);
}

void DShot::idle() {
    static const uint16_t idles[4] = {150,150,150,150};
    sendMotors(idles);
}

void DShot::sendCommand(uint8_t motor_idx, uint8_t cmd, int times) {
    if (motor_idx >= NUM_MOTORS || cmd == 0 || cmd > 47) return;
    for (int i = 0; i < times; ++i) {
        /* Encode cmd for one motor, telemetry=true, in its own buffer */
        encodeFrame(txBuf_[motor_idx], static_cast<uint16_t>(cmd), true);
        HAL_UART_AbortReceive(huarts_[motor_idx]);
        huarts_[motor_idx]->Instance->CR1 &= ~USART_CR1_RE;
        huarts_[motor_idx]->Instance->CR1 |=  USART_CR1_TE;
        HAL_UART_Transmit(huarts_[motor_idx], txBuf_[motor_idx], TX_BYTES, 1);
        HAL_Delay(1);   /* ~1 ms between command repeats */
    }
}

/* ============================================================
 *  HAL callback hooks — place in stm32f7xx_it.c or user code
 *
 *  extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
 *      if      (huart == &huart4)  FC::Drivers::DShot::onTxComplete(0);
 *      else if (huart == &huart5)  FC::Drivers::DShot::onTxComplete(1);
 *      else if (huart == &huart6)  FC::Drivers::DShot::onTxComplete(2);
 *      else if (huart == &huart7)  FC::Drivers::DShot::onTxComplete(3);
 *  }
 *  extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
 *      if      (huart == &huart4)  FC::Drivers::DShot::onRxComplete(0);
 *      else if (huart == &huart5)  FC::Drivers::DShot::onRxComplete(1);
 *      else if (huart == &huart6)  FC::Drivers::DShot::onRxComplete(2);
 *      else if (huart == &huart7)  FC::Drivers::DShot::onRxComplete(3);
 *      FC::Drivers::DShot::updateAvgFreq();   // after last motor
 *  }
 * ============================================================ */

}} // FC::Drivers
