#pragma once

/**
 * @file CRC16.h
 * @brief CRC16-CCITT (polynomial 0x1021, initial value 0xFFFF).
 *
 * This is NOT the same as Modbus CRC16 (0x8005). Using CCITT keeps
 * the transport CRC completely independent of any upper-layer protocol.
 *
 * Usage:
 *   uint16_t crc = crc16_ccitt(data, len);
 *
 * Incremental:
 *   uint16_t crc = 0xFFFF;
 *   crc = crc16_update(crc, byte1);
 *   crc = crc16_update(crc, byte2);
 */

#include <stdint.h>

/**
 * @brief Feed one byte into an ongoing CRC calculation.
 * Start with crc = 0xFFFF.
 */
inline uint16_t crc16_update(uint16_t crc, uint8_t byte) {
    crc ^= (uint16_t)byte << 8;
    for (uint8_t i = 0; i < 8; i++) {
        crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                              : (uint16_t)(crc << 1);
    }
    return crc;
}

/**
 * @brief Compute CRC16-CCITT over a byte array.
 */
inline uint16_t crc16_ccitt(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFFu;
    while (len--) {
        crc = crc16_update(crc, *data++);
    }
    return crc;
}
