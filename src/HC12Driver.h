#pragma once

/**
 * @file HC12Driver.h
 * @brief Low-level driver for the HC-12 433 MHz wireless UART module.
 *
 * Responsibilities:
 *  - UART TX/RX via HardwareSerial (ESP32 UART1 / UART2)
 *  - SET pin management for AT command mode vs. transparent mode
 *  - AT command engine: channel, TX power, FU mode, baud rate
 *  - Runtime reconfiguration without power cycling
 *  - Lock-free RX ring buffer (no blocking reads)
 *
 * HC-12 AT command notes:
 *  - SET pin LOW + 40 ms -> enter AT mode (radio deaf during AT mode)
 *  - SET pin HIGH + 80 ms -> return to transparent mode
 *  - AT commands always run at 9600 baud N81 regardless of configured rate
 *  - Responses end with "\r\n"
 *
 * Tested with HC-12 firmware v2.3 and v2.6.
 */

#include <Arduino.h>
#include <HardwareSerial.h>

#include "RingBuffer.h"

// --- HC-12 module constants ---

static constexpr uint32_t HC12_AT_BAUD = 9600;         /// Fixed AT-mode baud
static constexpr uint32_t HC12_AT_ENTER_MS = 40;       /// SET LOW hold before commands
static constexpr uint32_t HC12_AT_EXIT_MS = 80;        /// SET HIGH hold after commands
static constexpr uint32_t HC12_AT_CMD_TIMEOUT = 300;   /// ms waiting for AT response
static constexpr uint32_t HC12_POWER_STEP_DELAY = 50;  /// Extra settle after AT+Px

// --- Configuration struct ---

/** @brief HC-12 FU (Function) mode selection. */
enum class HC12Mode : uint8_t {
    FU1 = 1,  /// 250 ms latency, 5 km range, 1200 bps OTA
    FU2 = 2,  /// sleep/wake, extreme range, very low throughput
    FU3 = 3,  /// 15 ms latency, 1 km range (recommended for transport)
    FU4 = 4,  /// direct transparent, 1200 bps UART only
};

/**
 * @brief Complete HC-12 configuration.
 * All fields are applied in order during begin() / configure().
 */
struct HC12Config {
    uint8_t channel;  /// RF channel 1–127  (AT+C001 ... AT+C127)
    uint8_t power;    /// TX power level 1–8 (AT+P1 ... AT+P8, 1=-1dBm, 8=+20dBm)
    HC12Mode mode;    /// FU1–FU4           (AT+FU1 ... AT+FU4)
    uint32_t baud;    /// Transparent-mode UART baud (1200/2400/4800/9600/19200/38400/57600/115200)
};

/** @brief Sensible defaults for a reliable industrial link. */
static constexpr HC12Config HC12_DEFAULT_CONFIG = {
    .channel = 5,
    .power = 8,
    .mode = HC12Mode::FU3,
    .baud = 19200,
};

// --- HC12Driver class ---

class HC12Driver {
   public:
    // --- Lifecycle ---

    /**
     * @brief Initialise the driver.
     *
     * Configures the SET pin, opens the UART, and applies cfg via AT commands.
     *
     * @param serial   HardwareSerial instance (e.g. Serial1, Serial2).
     * @param setPin   GPIO connected to HC-12 SET pin.
     * @param rxPin    UART RX GPIO (pass -1 to use the serial default).
     * @param txPin    UART TX GPIO (pass -1 to use the serial default).
     * @param cfg      Module configuration to apply.
     * @return true on success; false if AT communication failed.
     */
    bool begin(HardwareSerial* serial, uint8_t setPin,
               int rxPin = -1, int txPin = -1,
               const HC12Config& cfg = HC12_DEFAULT_CONFIG);

    // --- Runtime reconfiguration ---

    /**
     * @brief Apply a complete new configuration.
     * Briefly pauses the radio (~120 ms total). Thread-safe if called from
     * a single task only.
     */
    bool configure(const HC12Config& cfg);

    /**
     * @brief Change only the RF channel at runtime.
     * @param ch Channel 1–127.
     */
    bool setChannel(uint8_t ch);

    /**
     * @brief Change only the TX power level at runtime.
     * @param p Power 1–8 (1 = –1 dBm, 8 = +20 dBm).
     */
    bool setPower(uint8_t p);

    /**
     * @brief Change only the FU mode at runtime.
     * Note: FU4 uses a different AT-mode baud (1200); avoid mixing modes.
     */
    bool setMode(HC12Mode mode);

    /** @brief Return the currently applied configuration. */
    HC12Config getConfig() const { return _config; }

    // --- Data path ---

    /**
     * @brief Transmit bytes over the air.
     * Blocks until all bytes are pushed into the UART TX FIFO.
     * @param data   Pointer to data.
     * @param len    Number of bytes.
     * @return true always (provided begin() succeeded).
     */
    bool send(const uint8_t* data, uint16_t len);

    /**
     * @brief Number of bytes waiting in the RX ring buffer.
     */
    uint16_t available();

    /**
     * @brief Read up to maxLen bytes from the RX ring buffer.
     * @return Actual bytes copied.
     */
    uint16_t read(uint8_t* buf, uint16_t maxLen);

    /**
     * @brief Pump bytes from the UART hardware FIFO into the software ring buffer.
     * Call this as often as possible — typically at the start of your main loop
     * or in a dedicated FreeRTOS task.
     */
    void update();

    // --- Diagnostics ---

    /** @brief Return true if the last configure/begin() was successful. */
    bool isConfigured() const { return _configured; }

    /** @brief Return number of RX bytes dropped due to ring-buffer overflow. */
    uint32_t rxOverflows() const { return _rxOverflow; }

   private:
    // --- AT command engine ---

    bool enterAT();
    bool exitAT();

    /**
     * @brief Send one AT command and wait for the expected response token.
     * @param cmd      Command string, e.g. "AT+C005"
     * @param expect   Token that must appear in the response, e.g. "OK"
     * @return true if expect was found within HC12_AT_CMD_TIMEOUT ms.
     */
    bool sendAT(const char* cmd, const char* expect = "OK");

    void flushSerial(uint32_t ms = 10);

    // --- Members ---

    HardwareSerial* _serial = nullptr;
    uint8_t _setPin = 0;
    int _rxPin = -1;
    int _txPin = -1;
    HC12Config _config = HC12_DEFAULT_CONFIG;
    bool _configured = false;
    uint32_t _rxOverflow = 0;

    /** Software RX ring buffer — 512 bytes keeps 2 full max-size frames. */
    RingBuffer<512> _rxBuf;
};
