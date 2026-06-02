/**
 * @file HC12Driver.cpp
 * @brief HC-12 module driver implementation.
 */

#include "HC12Driver.h"

#include <stdio.h>   // snprintf
#include <string.h>  // strstr

// --- Lifecycle ---

bool HC12Driver::begin(HardwareSerial* serial, uint8_t setPin,
                       int rxPin, int txPin,
                       const HC12Config& cfg) {
    _serial = serial;
    _setPin = setPin;
    _rxPin = rxPin;
    _txPin = txPin;

    // SET pin: output, start HIGH (transparent mode)
    pinMode(_setPin, OUTPUT);
    digitalWrite(_setPin, HIGH);

    // Open UART at configured transparent baud
    if (_rxPin >= 0 && _txPin >= 0) {
        _serial->begin(cfg.baud, SERIAL_8N1, _rxPin, _txPin);
    } else {
        _serial->begin(cfg.baud, SERIAL_8N1);
    }

    delay(100);  // let module power up

    _configured = configure(cfg);
    return _configured;
}

// --- Runtime reconfiguration ---

bool HC12Driver::configure(const HC12Config& cfg) {
    if (!enterAT()) return false;

    bool ok = true;

    // 1. FU mode
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "AT+FU%d", (int)cfg.mode);
    ok &= sendAT(cmd, "OK+FU");

    // 2. UART baud
    // Valid HC-12 baud rates (AT+Bxxxxx)
    snprintf(cmd, sizeof(cmd), "AT+B%lu", (unsigned long)cfg.baud);
    ok &= sendAT(cmd, "OK+B");

    // 3. Channel
    snprintf(cmd, sizeof(cmd), "AT+C%03d", (int)cfg.channel);
    ok &= sendAT(cmd, "OK+C");

    // 4. TX power
    snprintf(cmd, sizeof(cmd), "AT+P%d", (int)cfg.power);
    ok &= sendAT(cmd, "OK+P");

    exitAT();

    if (ok) {
        _config = cfg;
        // Reopen serial at new baud after exit (HC-12 now uses cfg.baud)
        _serial->flush();
        if (_rxPin >= 0 && _txPin >= 0) {
            _serial->begin(cfg.baud, SERIAL_8N1, _rxPin, _txPin);
        } else {
            _serial->begin(cfg.baud, SERIAL_8N1);
        }
    }

    return ok;
}

bool HC12Driver::setChannel(uint8_t ch) {
    if (ch < 1 || ch > 127) return false;
    if (!enterAT()) return false;

    char cmd[16];
    snprintf(cmd, sizeof(cmd), "AT+C%03d", (int)ch);
    bool ok = sendAT(cmd, "OK+C");

    exitAT();
    if (ok) _config.channel = ch;
    return ok;
}

bool HC12Driver::setPower(uint8_t p) {
    if (p < 1 || p > 8) return false;
    if (!enterAT()) return false;

    char cmd[8];
    snprintf(cmd, sizeof(cmd), "AT+P%d", (int)p);
    bool ok = sendAT(cmd, "OK+P");

    exitAT();
    delay(HC12_POWER_STEP_DELAY);  // extra settle after power change
    if (ok) _config.power = p;
    return ok;
}

bool HC12Driver::setMode(HC12Mode mode) {
    if (!enterAT()) return false;

    char cmd[12];
    snprintf(cmd, sizeof(cmd), "AT+FU%d", (int)mode);
    bool ok = sendAT(cmd, "OK+FU");

    exitAT();
    if (ok) _config.mode = mode;
    return ok;
}

// --- Data path ---

bool HC12Driver::send(const uint8_t* data, uint16_t len) {
    if (!_serial) return false;
    _serial->write(data, len);
    _serial->flush();  // wait for TX FIFO to drain
    return true;
}

uint16_t HC12Driver::available() {
    return _rxBuf.available();
}

uint16_t HC12Driver::read(uint8_t* buf, uint16_t maxLen) {
    return _rxBuf.read(buf, maxLen);
}

void HC12Driver::update() {
    while (_serial && _serial->available()) {
        uint8_t b = (uint8_t)_serial->read();
        if (!_rxBuf.push(b)) {
            _rxOverflow++;
        }
    }
}

// --- AT command engine (private) ---

bool HC12Driver::enterAT() {
    if (!_serial) return false;

    // Assert SET LOW -> enter AT mode
    digitalWrite(_setPin, LOW);
    delay(HC12_AT_ENTER_MS);

    const uint32_t bauds[] = {_config.baud, 9600, 1200, 2400, 4800, 19200, 38400, 57600, 115200};
    for (uint8_t i = 0; i < sizeof(bauds) / sizeof(bauds[0]); i++) {
        _serial->flush();
        if (_rxPin >= 0 && _txPin >= 0) {
            _serial->begin(bauds[i], SERIAL_8N1, _rxPin, _txPin);
        } else {
            _serial->begin(bauds[i], SERIAL_8N1);
        }
        delay(20);  // wait for UART to stabilize and TX to idle high

        // Verify module is alive - try twice to clear potential line level glitches
        if (sendAT("AT", "OK") || sendAT("AT", "OK")) {
            return true;
        }
    }

    return false;
}

bool HC12Driver::exitAT() {
    // De-assert SET -> back to transparent mode
    digitalWrite(_setPin, HIGH);
    delay(HC12_AT_EXIT_MS);
    flushSerial(20);

    // Restore transparent baud (may have changed during configure())
    _serial->flush();
    if (_rxPin >= 0 && _txPin >= 0) {
        _serial->begin(_config.baud, SERIAL_8N1, _rxPin, _txPin);
    } else {
        _serial->begin(_config.baud, SERIAL_8N1);
    }

    return true;
}

bool HC12Driver::sendAT(const char* cmd, const char* expect) {
    if (!_serial) return false;

    // Flush any pending RX garbage
    flushSerial(10);

    // Send command with CR LF
    _serial->print(cmd);
    _serial->print(F("\r\n"));
    _serial->flush();

    // Read response, look for expect token
    char resp[64];
    uint8_t idx = 0;
    uint32_t t0 = millis();

    while (millis() - t0 < HC12_AT_CMD_TIMEOUT) {
        if (_serial->available()) {
            char c = (char)_serial->read();
            if (idx < sizeof(resp) - 1) {
                resp[idx++] = c;
                resp[idx] = '\0';
            }
            // Check for expected token after each character
            if (strstr(resp, expect) != nullptr) {
                return true;
            }
        }
    }

    return false;
}

void HC12Driver::flushSerial(uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        while (_serial && _serial->available()) {
            _serial->read();
        }
    }
}
