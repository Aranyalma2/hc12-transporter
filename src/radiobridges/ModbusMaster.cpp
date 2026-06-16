/**
 * @file ModbusMaster.cpp
 * @brief ModbusMaster -- Modbus RTU bridging over RadioTransport, master side.
 */

#include "ModbusMaster.h"

#include <esp_log.h>
#include <string.h>

static const char* TAG_MM = "ModbusMaster";

// ---------------------------------------------------------------------------
// Modbus RTU frame helpers (file-local)
// ---------------------------------------------------------------------------

/**
 * @brief Compute Modbus RTU CRC-16 (polynomial 0xA001, init 0xFFFF).
 * @param buf  Data bytes (NOT including the two CRC bytes themselves).
 * @param len  Number of data bytes.
 * @return 16-bit CRC, little-endian byte order as stored in the frame.
 */
static uint16_t _modbusRtuCrc(const uint8_t* buf, uint8_t len) {
    uint16_t crc = 0xFFFFu;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x0001u) ? ((crc >> 1) ^ 0xA001u) : (crc >> 1);
        }
    }
    return crc;
}

/**
 * @brief Locate and validate a Modbus RTU frame inside a potentially noisy
 *        byte buffer.
 *
 * Handles two common RS-485 / UART noise patterns:
 *  1. **Leading 0x00 bytes** -- spurious nulls that appear before the device
 *     address byte are skipped.
 *  2. **Trailing garbage bytes** -- extra bytes appended after the real frame
 *     are stripped by trying progressively shorter lengths until a CRC-16
 *     match is found (longest valid match wins).
 *
 * A candidate frame is additionally rejected when:
 *  - Device address is 0 or outside the standard range 1-247.
 *  - Function code is 0 or in the exception-response range (>= 0x80).
 *
 * @param buf    Raw buffer received from the serial port.
 * @param len    Number of bytes in buf.
 * @param out    Output buffer for the cleaned frame (must be >= len bytes).
 * @return       Length of the cleaned frame (>= 4) or 0 if no valid frame
 *               could be found.
 */
static uint8_t _parseModbusFrame(const uint8_t* buf, uint8_t len,
                                  uint8_t* out) {
    // --- Step 1: skip leading 0x00 noise bytes ---
    uint8_t start = 0;
    while (start < len && buf[start] == 0x00u) start++;

    const uint8_t* p = buf + start;
    uint8_t avail = len - start;

    if (avail < 4u) return 0u;  // too short for addr + FC + 2-byte CRC

    // --- Step 2: try lengths from avail down to 4, longest valid match wins ---
    for (uint8_t tryLen = avail; tryLen >= 4u; tryLen--) {
        // CRC is stored little-endian in the last two bytes.
        uint16_t crcStored = (uint16_t)p[tryLen - 2u] |
                             ((uint16_t)p[tryLen - 1u] << 8u);
        uint16_t crcCalc   = _modbusRtuCrc(p, (uint8_t)(tryLen - 2u));

        if (crcCalc != crcStored) continue;

        // Sanity-check device address and function code.
        uint8_t addr = p[0];
        uint8_t fc   = p[1];
        if (addr < 1u || addr > 247u) continue;  // invalid device address
        if (fc  < 1u || fc  > 127u)  continue;  // 0 invalid; >= 0x80 = exception

        memcpy(out, p, tryLen);
        return tryLen;
    }

    return 0u;  // no valid frame found
}

// --- Lifecycle ---

bool ModbusMaster::begin(RadioTransport* transport, uint32_t rxTimeoutMs) {
    _transport = transport;
    _rxTimeoutMs = rxTimeoutMs;

    memset(_route, 0, sizeof(_route));
    memset(_failCount, 0, sizeof(_failCount));
    memset(_pendingRespData, 0, sizeof(_pendingRespData));

    _pendingActive = false;
    _pendingMbAddr = 0;
    _pendingRespLen = 0;
    _respQHead = _respQTail = 0;

    _pendingMutex = xSemaphoreCreateMutex();
    _responseSem = xSemaphoreCreateBinary();
    _respMutex = xSemaphoreCreateMutex();

    return (_pendingMutex && _responseSem && _respMutex && transport != nullptr);
}

// --- Serial bridge ---

void ModbusMaster::attachSerial(Stream* serial, uint32_t silenceMs, int rstPin) {
    _serial = serial;
    _silenceMs = silenceMs;
    _rstPin = rstPin;
    if (_rstPin >= 0) {
        pinMode(_rstPin, OUTPUT);
        digitalWrite(_rstPin, LOW);  // start in RX mode
    }
}

// --- API mode ---

void ModbusMaster::onResponse(ResponseCallback cb) {
    _responseCb = cb;
}

bool ModbusMaster::available() const {
    if (_responseCb) return false;
    return _respQHead != _respQTail;
}

bool ModbusMaster::receive(uint8_t* frame, uint8_t* len, uint8_t* radioSrc) {
    if (_responseCb) return false;
    if (xSemaphoreTake(_respMutex, pdMS_TO_TICKS(5)) != pdTRUE) return false;

    bool ok = (_respQHead != _respQTail);
    if (ok) {
        RespEntry& e = _respQueue[_respQTail];
        if (radioSrc) *radioSrc = e.radioSrc;
        if (len) *len = e.len;
        if (frame && e.len > 0) memcpy(frame, e.data, e.len);
        _respQTail = (_respQTail + 1u) % MODBUS_MASTER_RESP_QUEUE;
    }

    xSemaphoreGive(_respMutex);
    return ok;
}

// --- Route management ---

void ModbusMaster::clearRoutes() {
    memset(_route, 0, sizeof(_route));
    memset(_failCount, 0, sizeof(_failCount));
}

void ModbusMaster::addRoute(uint8_t modbusAddr, uint8_t radioAddr) {
    _route[modbusAddr] = radioAddr;
    _failCount[modbusAddr] = 0;
}

// --- Internal send + wait ---

bool ModbusMaster::send(const uint8_t* frame, uint8_t len) {
    // Serialise: only one Modbus request in-flight at a time.
    if (xSemaphoreTake(_pendingMutex, pdMS_TO_TICKS(_rxTimeoutMs + 200)) != pdTRUE)
        return false;

    bool ok = _sendFrame(frame, len);

    xSemaphoreGive(_pendingMutex);
    return ok;
}

bool ModbusMaster::_sendFrame(const uint8_t* frame, uint8_t len) {
    if (!_transport || len < 4 || len > RADIO_MAX_PAYLOAD) return false;

    // --- Validate and strip noise from the raw frame ---
    // Strip leading 0x00 bytes and trailing garbage; verify Modbus CRC-16.
    uint8_t cleanBuf[RADIO_MAX_PAYLOAD];
    uint8_t cleanLen = _parseModbusFrame(frame, len, cleanBuf);
    if (cleanLen == 0u) {
        ESP_LOGW(TAG_MM, "_sendFrame: rejected -- no valid Modbus RTU frame "
                         "in %d raw bytes", len);
        return false;
    }
    if (cleanLen != len) {
        ESP_LOGD(TAG_MM, "_sendFrame: noise stripped %d -> %d bytes", len, cleanLen);
    }
    frame = cleanBuf;
    len   = cleanLen;

    uint8_t mbAddr = frame[0];

    // Determine radio destination: use learned route or fall back to broadcast.
    uint8_t dest = _route[mbAddr];
    if (dest == 0) {
        dest = RADIO_ADDR_BROADCAST;
        ESP_LOGD(TAG_MM, "TX mbAddr=0x%02X route unknown -- using broadcast", mbAddr);
    } else {
        ESP_LOGD(TAG_MM, "TX mbAddr=0x%02X -> radio=0x%02X len=%d", mbAddr, dest, len);
    }

    // Arm the response semaphore BEFORE sending so _processReceivedPacket()
    // can give it even if the response arrives very quickly.
    _pendingActive = true;
    _pendingMbAddr = mbAddr;
    _pendingRespLen = 0;

    // Transport-level send: blocks until radio ACK or all retries fail.
    // For broadcast, returns immediately after TX.
    bool txOk = _transport->send(dest, PacketType::DATA, frame, len);
    if (!txOk && dest != RADIO_ADDR_BROADCAST) {
        // Radio delivery failed entirely -- count as a Modbus failure too.
        _pendingActive = false;
        if (++_failCount[mbAddr] >= _maxFailures) {
            ESP_LOGW(TAG_MM, "route evict mbAddr=0x%02X (radio TX failed, %d failures)",
                     mbAddr, _maxFailures);
            _route[mbAddr] = 0;
            _failCount[mbAddr] = 0;
        }
        return false;
    }

    // Wait for the Modbus-level response while actively pumping the transport
    // receive queue. This is required in serial bridge mode: _sendFrame() is
    // called from _runTask() which owns the only place that drains
    // transport.receive(). A plain blocking xSemaphoreTake() would freeze the
    // loop and starve _processReceivedPacket() -- the exact deadlock seen in logs.
    bool responseOk = false;
    {
        uint32_t deadline = millis() + _rxTimeoutMs;
        uint8_t pSrc;
        PacketType pType;
        uint8_t pData[RADIO_MAX_PAYLOAD];
        uint8_t pLen;

        while (millis() < deadline) {
            // Drain any packets the transport has queued for us.
            while (_transport->receive(&pSrc, &pType, pData, &pLen)) {
                if (pType == PacketType::DATA) {
                    _processReceivedPacket(pSrc, pData, pLen);
                }
            }
            // Check if our response has arrived (semaphore given by _processReceivedPacket).
            if (xSemaphoreTake(_responseSem, 0) == pdTRUE) {
                responseOk = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    _pendingActive = false;

    if (!responseOk) {
        // Modbus timeout: device did not respond.
        ESP_LOGW(TAG_MM, "Modbus timeout mbAddr=0x%02X (no response in %lu ms)",
                 mbAddr, (unsigned long)_rxTimeoutMs);
        if (++_failCount[mbAddr] >= _maxFailures) {
            ESP_LOGW(TAG_MM, "route evict mbAddr=0x%02X (%d timeouts)",
                     mbAddr, _maxFailures);
            _route[mbAddr] = 0;
            _failCount[mbAddr] = 0;
        }
        return false;
    }

    // Success: reset fail counter.
    _failCount[mbAddr] = 0;
    ESP_LOGD(TAG_MM, "response mbAddr=0x%02X from radio=0x%02X len=%d",
             mbAddr, _pendingRespRadio, _pendingRespLen);

    // Deliver response to callback or queue.
    if (_responseCb) {
        _responseCb(_pendingRespRadio, _pendingRespData, _pendingRespLen);
    } else {
        xSemaphoreTake(_respMutex, portMAX_DELAY);
        uint8_t next = (_respQHead + 1u) % MODBUS_MASTER_RESP_QUEUE;
        if (next != _respQTail) {
            RespEntry& e = _respQueue[_respQHead];
            e.radioSrc = _pendingRespRadio;
            e.len = _pendingRespLen;
            if (_pendingRespLen > 0) memcpy(e.data, _pendingRespData, _pendingRespLen);
            _respQHead = next;
        }
        xSemaphoreGive(_respMutex);
    }

    return true;
}

// --- Response processing (called from _runTask) ---

void ModbusMaster::_processReceivedPacket(uint8_t radioSrc,
                                          const uint8_t* data, uint8_t len) {
    if (len < 1) return;
    uint8_t mbAddr = data[0];

    // Learn route from every incoming DATA packet.
    if (radioSrc != RADIO_ADDR_BROADCAST) {
        if (_route[mbAddr] != radioSrc) {
            ESP_LOGD(TAG_MM, "route learn mbAddr=0x%02X -> radio=0x%02X",
                     mbAddr, radioSrc);
        }
        _route[mbAddr] = radioSrc;
    }

    // If this is the response to the in-flight request, give the semaphore.
    if (_pendingActive && mbAddr == _pendingMbAddr) {
        _pendingRespRadio = radioSrc;
        _pendingRespLen = (len <= RADIO_MAX_PAYLOAD) ? len : RADIO_MAX_PAYLOAD;
        memcpy(_pendingRespData, data, _pendingRespLen);
        xSemaphoreGive(_responseSem);
    }
}

// --- Silence-based serial frame reader ---

uint8_t ModbusMaster::_readSerialFrame(uint8_t* buf, uint8_t maxLen) {
    // Wait until at least one byte arrives (with an outer timeout to avoid spinning forever).
    uint32_t startWait = millis();
    while (!_serial->available()) {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (millis() - startWait > 1000) return 0;  // nothing arrived for 1 s
    }

    // Read bytes until inter-frame silence (_silenceMs with no new byte).
    uint8_t count = 0;
    uint32_t lastByteMs = millis();

    while (count < maxLen) {
        if (_serial->available()) {
            buf[count++] = (uint8_t)_serial->read();
            lastByteMs = millis();
        } else {
            if (millis() - lastByteMs >= _silenceMs) break;  // end of frame
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    return count;
}

// --- Background task ---

bool ModbusMaster::startTask(uint32_t stackDepth, UBaseType_t priority, BaseType_t core) {
    if (_taskHandle) return true;
    return xTaskCreatePinnedToCore(
               _taskFunc, "modbus_master",
               stackDepth, this, priority, &_taskHandle, core) == pdPASS;
}

void ModbusMaster::stopTask() {
    if (_taskHandle) {
        vTaskDelete(_taskHandle);
        _taskHandle = nullptr;
    }
}

void ModbusMaster::_taskFunc(void* arg) {
    static_cast<ModbusMaster*>(arg)->_runTask();
}

void ModbusMaster::_runTask() {
    uint8_t rxBuf[RADIO_MAX_PAYLOAD];

    for (;;) {
        // --- Poll transport receive queue for incoming responses ---
        {
            uint8_t src;
            PacketType type;
            uint8_t data[RADIO_MAX_PAYLOAD];
            uint8_t len;

            while (_transport->receive(&src, &type, data, &len)) {
                if (type == PacketType::DATA) {
                    _processReceivedPacket(src, data, len);
                } else if (type == PacketType::PING) {
                    // Just reply to PINGs with PONGs
                    _transport->send(src, PacketType::PONG, nullptr, 0);
                }
                // Other packet types (PONG, STATUS) are ignored by the bridge.
            }
        }

        // --- Serial bridge: read a frame from UART and forward it ---
        if (_serial) {
            uint8_t flen = _readSerialFrame(rxBuf, sizeof(rxBuf));
            if (flen >= 4) {
                // Take pending mutex so no concurrent API send() interferes.
                if (xSemaphoreTake(_pendingMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    bool ok = _sendFrame(rxBuf, flen);

                    if (ok && _pendingRespLen > 0) {
                        // Write response back to the serial port.
                        // Assert direction pin (TX mode) before writing.
                        if (_rstPin >= 0) digitalWrite(_rstPin, HIGH);
                        _serial->write(_pendingRespData, _pendingRespLen);
                        _serial->flush();                              // wait for last byte to leave the UART TX FIFO
                        if (_rstPin >= 0) digitalWrite(_rstPin, LOW);  // back to RX mode
                    }
                    xSemaphoreGive(_pendingMutex);
                }
                continue;  // immediately loop back to handle next serial frame
            }
        }

        // No serial frame and no pending response -- yield for a tick.
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
