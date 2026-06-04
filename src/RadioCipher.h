#pragma once

/**
 * @file RadioCipher.h
 * @brief AES-128-CTR payload cipher for HC-12 radio packets.
 *
 * Uses the ESP32 hardware AES accelerator via mbedTLS (mbedtls/aes.h).
 * The hardware block is always available on ESP32 and requires no extra
 * library - it is part of the ESP-IDF bundled with the Arduino ESP32 core.
 *
 * Key is supplied at construction via init() and is immutable afterwards.
 *
 * --- CTR mode properties ---
 *  - Output length == input length (zero wire overhead).
 *  - Self-inverse: crypt(crypt(x, nonce), nonce) == x.
 *    Encrypt and decrypt are the same function call.
 *
 * --- Nonce construction ---
 *  The 16-byte AES counter block is built from the packet header fields
 *  that are already on the wire in plaintext:
 *
 *    counter[0]  = dest
 *    counter[1]  = src
 *    counter[2]  = seq
 *    counter[3..15] = 0x00
 *
 *  Because (dest, src, seq) is unique per direction per packet within the
 *  256-packet duplicate-detection window, nonce reuse is bounded to the
 *  same window - no extra wire bytes are needed.
 *
 * @note This file is ESP32-only. No software AES fallback is provided.
 */

#include <mbedtls/aes.h>
#include <stdint.h>

class RadioCipher {
   public:
    static constexpr uint8_t KEY_SIZE = 16;   /// AES-128: 16 bytes
    static constexpr uint8_t NONCE_SIZE = 3;  /// { dest, src, seq }

    RadioCipher() = default;

    /**
     * @brief Load the pre-shared key and run the AES-128 key schedule.
     *
     * Must be called exactly once, from RadioTransport::begin().
     * The key is stored internally; the caller's copy can be discarded
     * (or zeroed) after this returns.
     *
     * @param key  16-byte pre-shared key.
     */
    void init(const uint8_t key[KEY_SIZE]);

    /**
     * @brief In-place AES-128-CTR encrypt or decrypt.
     *
     * CTR mode is self-inverse: calling crypt() twice with the same nonce
     * restores the original data. There is no separate decrypt function.
     *
     * @param nonce3  3-byte packet nonce: { dest, src, seq }
     * @param buf     Payload buffer, modified in place.
     * @param len     Payload length in bytes (0-RADIO_MAX_PAYLOAD).
     */
    void crypt(const uint8_t nonce3[NONCE_SIZE], uint8_t* buf, uint8_t len) const;

   private:
    /**
     * The mbedTLS AES context holds the expanded key schedule.
     * Marked mutable because mbedtls_aes_crypt_ctr() takes a non-const
     * pointer even though it only reads the key schedule during crypt().
     */
    mutable mbedtls_aes_context _ctx;
};
