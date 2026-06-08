/**
 * @file RadioCipher.cpp
 * @brief AES-128-CTR cipher implementation using the ESP32 hardware AES accelerator.
 *
 * mbedtls_aes_crypt_ctr() internally calls the ESP32 hardware AES block cipher,
 * so the key schedule runs in hardware and the throughput is typically 10–30×
 * faster than an equivalent software AES implementation.
 *
 * CTR counter block layout (16 bytes, big-endian AES input):
 *   [0] = dest   [1] = src   [2] = seq   [3..15] = 0x00
 *
 * mbedtls_aes_crypt_ctr() signature (from mbedtls/aes.h):
 *   int mbedtls_aes_crypt_ctr(
 *       mbedtls_aes_context *ctx,
 *       size_t               length,
 *       size_t              *nc_off,        // stream block offset (start at 0)
 *       unsigned char        nonce_counter[16],
 *       unsigned char        stream_block[16],
 *       const unsigned char *input,
 *       unsigned char       *output
 *   );
 * Returns 0 on success. On ESP32 this never fails once the context is set up.
 */

#include "RadioCipher.h"

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

void RadioCipher::init(const uint8_t key[KEY_SIZE]) {
    mbedtls_aes_init(&_ctx);
    // setkey_enc is correct for CTR mode regardless of encrypt/decrypt direction,
    // because CTR encrypts the counter block and XORs - the AES direction is
    // always "encrypt" for the counter block itself.
    mbedtls_aes_setkey_enc(&_ctx, key, 128u);
}

// ---------------------------------------------------------------------------
// crypt() - in-place AES-128-CTR (self-inverse)
// ---------------------------------------------------------------------------

void RadioCipher::crypt(const uint8_t nonce3[NONCE_SIZE], uint8_t* buf, uint8_t len) const {
    if (len == 0) return;

    // Build the 16-byte AES-CTR counter block.
    // Bytes beyond [2] stay zero - no counter wrap within a single 64-byte payload.
    uint8_t counter[16] = {};
    counter[0] = nonce3[0];  // dest
    counter[1] = nonce3[1];  // src
    counter[2] = nonce3[2];  // seq

    // stream_block holds the current AES-ECB output of the counter.
    // nc_off tracks the byte offset within stream_block for sub-block payloads.
    uint8_t stream_block[16] = {};
    size_t nc_off = 0;

    mbedtls_aes_crypt_ctr(&_ctx, len, &nc_off, counter, stream_block, buf, buf);
}
