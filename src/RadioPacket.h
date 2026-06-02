#pragma once

/**
 * @file RadioPacket.h
 * @brief Wire-format definitions for the HC-12 Radio Transport protocol.
 *
 * Wire layout (big-endian byte order for CRC):
 * ----------------------------------------------------------
 * | SOF | DEST | SRC | SEQ | TYPE | LEN | PAYLOAD | CRC16  |
 * ----------------------------------------------------------
 *    1      1     1     1     1     1      0–64       2
 *
 * Total wire length: 7 + LEN bytes (max 71 bytes).
 * CRC16-CCITT covers bytes 1–(5+LEN) inclusive (DEST through PAYLOAD).
 *
 * Address space:
 *   0x01       -> Master
 *   0x10–0xFE  -> Slaves (238 possible nodes)
 *   0xFF       -> Broadcast (no ACK expected)
 */

#include <stdint.h>

// --- Constants ---

static constexpr uint8_t RADIO_SOF = 0xAA;
static constexpr uint8_t RADIO_ADDR_MASTER = 0x01;
static constexpr uint8_t RADIO_ADDR_BROADCAST = 0xFF;
static constexpr uint8_t RADIO_SLAVE_BASE = 0x10;  /// First valid slave address

static constexpr uint8_t RADIO_MAX_PAYLOAD = 64;
static constexpr uint8_t RADIO_HEADER_SIZE = 6;  /// SOF+DEST+SRC+SEQ+TYPE+LEN
static constexpr uint8_t RADIO_CRC_SIZE = 2;
static constexpr uint8_t RADIO_MIN_FRAME = RADIO_HEADER_SIZE + RADIO_CRC_SIZE;                      /// 8 bytes (empty payload)
static constexpr uint8_t RADIO_MAX_FRAME = RADIO_HEADER_SIZE + RADIO_MAX_PAYLOAD + RADIO_CRC_SIZE;  /// 72 bytes

// --- Packet Types ---

/**
 * @brief Packet type field (byte 4 of the wire frame).
 *
 * The transport layer is completely payload-agnostic.
 * Application protocols (Modbus, custom binary, etc.) are carried in PKT_DATA.
 */
enum class PacketType : uint8_t {
    DATA = 0x00,    /// Generic application payload (Modbus RTU, custom, ...)
    ACK = 0x01,     /// Transport-level acknowledgement
    NACK = 0x02,    /// Negative ACK (CRC error or rejected duplicate)
    PING = 0x03,    /// Link-quality probe / discovery broadcast
    PONG = 0x04,    /// Reply to PING (may carry node metadata in payload)
    STATUS = 0x05,  /// Diagnostic / statistics frame
};

// --- In-memory packet representation ---

/**
 * @brief In-memory representation of one radio packet.
 *
 * Not the wire layout — use RadioPacket::encode() / decode() to
 * convert to/from the flat byte array that goes over the air.
 */
struct RadioPacket {
    uint8_t dest;
    uint8_t src;
    uint8_t seq;
    PacketType type;
    uint8_t len;  /// Payload length (0–RADIO_MAX_PAYLOAD)
    uint8_t payload[RADIO_MAX_PAYLOAD];

    /**
     * @brief Serialise this packet into buf[].
     * @param buf  Output buffer — must be at least RADIO_MAX_FRAME bytes.
     * @return     Number of bytes written.
     */
    uint8_t encode(uint8_t* buf) const;

    /**
     * @brief Parse a flat byte array into this packet.
     * @param buf  Input buffer (raw bytes off the wire, starting at SOF).
     * @param len  Number of bytes in buf.
     * @return     true if the frame is valid (good SOF + CRC match).
     */
    bool decode(const uint8_t* buf, uint8_t frameLen);
};
