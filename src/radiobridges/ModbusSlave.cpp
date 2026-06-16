/**
 * @file ModbusSlave.cpp
 * @brief ModbusSlave -- Modbus RTU bridging over RadioTransport, slave side.
 */

#include "ModbusSlave.h"

#include <esp_log.h>
#include <string.h>

static const char* TAG_MS = "ModbusSlave";

// --- Lifecycle ---

bool ModbusSlave::begin(RadioTransport* transport) {
    _transport = transport;

    memset(_reqQueue, 0, sizeof(_reqQueue));
    _reqQHead = _reqQTail = 0;

    _reqMutex = xSemaphoreCreateMutex();
    _reqNotify = xSemaphoreCreateBinary();

    return (_reqMutex && _reqNotify && transport != nullptr);
}

// --- Serial bridge ---

void ModbusSlave::attachSerial(Stream* serial,
                               uint32_t responseTimeoutMs,
                               uint32_t silenceMs,
                               int rstPin) {
    _serial = serial;
    _responseTimeoutMs = responseTimeoutMs;
    _silenceMs = silenceMs;
    _rstPin = rstPin;
    if (_rstPin >= 0) {
        pinMode(_rstPin, OUTPUT);
        digitalWrite(_rstPin, LOW);  // start in RX mode
    }
}

// --- Local handler ---

void ModbusSlave::onRequest(RequestCallback cb) {
    _requestCb = cb;
}

// --- Serial response reader ---

uint8_t ModbusSlave::_readSerialResponse(uint8_t* buf, uint8_t maxLen) {
    uint32_t deadline = millis() + _responseTimeoutMs;
    uint32_t lastByteMs = 0;
    uint8_t count = 0;
    bool started = false;

    while (millis() < deadline && count < maxLen) {
        if (_serial->available()) {
            buf[count++] = (uint8_t)_serial->read();
            lastByteMs = millis();
            started = true;
        } else {
            if (started && (millis() - lastByteMs) >= _silenceMs) {
                break;  // end of response frame (inter-frame silence)
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    return count;
}

// --- Request handling ---

void ModbusSlave::_handleRequest(uint8_t radioSrc,
                                 const uint8_t* data, uint8_t len) {
    uint8_t respBuf[RADIO_MAX_PAYLOAD];
    uint8_t respLen = 0;

    ESP_LOGD(TAG_MS, "request from radio=0x%02X len=%d", radioSrc, len);

    if (_serial) {
        // --- Serial bridge mode ---
        // Assert direction pin (TX mode) before forwarding the request to RS-485.
        if (_rstPin >= 0) digitalWrite(_rstPin, HIGH);
        _serial->write(data, len);
        _serial->flush();                              // wait for last byte out before switching to RX
        if (_rstPin >= 0) digitalWrite(_rstPin, LOW);  // back to RX mode to receive response
        ESP_LOGD(TAG_MS, "forwarded %d bytes to serial -- waiting response", len);

        respLen = _readSerialResponse(respBuf, sizeof(respBuf));
        ESP_LOGD(TAG_MS, "serial response %d bytes", respLen);

    } else if (_requestCb) {
        // --- Local callback mode ---
        _requestCb(radioSrc, data, len, respBuf, &respLen);
        ESP_LOGD(TAG_MS, "callback response %d bytes", respLen);
    }

    // Send response back to the master if we have one.
    // sendAsync() is safe here (we are in the slave task, not in the transport callback).
    if (respLen > 0) {
        ESP_LOGD(TAG_MS, "sending response %d bytes -> radio=0x%02X", respLen, radioSrc);
        _transport->sendAsync(radioSrc, PacketType::DATA, respBuf, respLen);
    } else {
        ESP_LOGD(TAG_MS, "no response for this request (radioSrc=0x%02X)", radioSrc);
    }
}

// --- Background task ---

bool ModbusSlave::startTask(uint32_t stackDepth, UBaseType_t priority, BaseType_t core) {
    if (_taskHandle) return true;
    return xTaskCreatePinnedToCore(
               _taskFunc, "modbus_slave",
               stackDepth, this, priority, &_taskHandle, core) == pdPASS;
}

void ModbusSlave::stopTask() {
    if (_taskHandle) {
        vTaskDelete(_taskHandle);
        _taskHandle = nullptr;
    }
}

void ModbusSlave::_taskFunc(void* arg) {
    static_cast<ModbusSlave*>(arg)->_runTask();
}

void ModbusSlave::_runTask() {
    for (;;) {
        // Poll the transport receive queue for incoming DATA packets.
        {
            uint8_t src;
            PacketType type;
            uint8_t data[RADIO_MAX_PAYLOAD];
            uint8_t len;

            while (_transport->receive(&src, &type, data, &len)) {
                if (type == PacketType::DATA && len > 0) {
                    _handleRequest(src, data, len);
                } else if (type == PacketType::PING) {
                    // Just reply to PINGs with PONGs
                    _transport->send(src, PacketType::PONG, nullptr, 0);
                }
                // Other packet types (PING, PONG, STATUS) ignored by the slave bridge.
            }
        }

        // Yield for 1 ms before polling again.
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
