/**
 * @file RadioTransport.cpp
 * @brief RadioTransport -- TX/RX state machines, ACK/retry, auto-power.
 *
 * All state machine work runs inside the FreeRTOS background task spawned by
 * startTask(). The public API (send, sendAsync, receive) is thread-safe.
 *
 * Mutex strategy:
 *  _txMutex  -- protects _txState and all TX state variables.
 *               Held by _updateTx() and briefly by sendAsync() / _processComplete().
 *               Released around vTaskDelay() in the SENDING case so sendAsync()
 *               callers are not blocked for the full txRxSwitchDelayMs window.
 *  _rxMutex  -- protects _rxQueue head/tail, taken briefly in _rxEnqueue() / receive().
 *  _ackEvent -- binary semaphore given when a unicast TX resolves (ACK, NACK, timeout).
 *               send() blocks on this; never held across any blocking call.
 *  _txTrigger-- binary semaphore given by sendAsync() to wake the task immediately
 *               instead of waiting for the 5 ms heartbeat tick.
 */

#include "RadioTransport.h"

#include "CRC16.h"
#include "esp_log.h"

static const char* TAG_RT = "RadioTransport";

// --- Packet encode / decode (RadioPacket methods) ---

uint8_t RadioPacket::encode(uint8_t* buf) const {
    buf[0] = RADIO_SOF;
    buf[1] = dest;
    buf[2] = src;
    buf[3] = seq;
    buf[4] = (uint8_t)type;
    buf[5] = len;
    if (len > 0) {
        memcpy(buf + RADIO_HEADER_SIZE, payload, len);
    }
    // CRC covers bytes 1..(5+len): DEST, SRC, SEQ, TYPE, LEN, PAYLOAD
    uint16_t crc = crc16_ccitt(buf + 1, (uint16_t)(5u + len));
    buf[RADIO_HEADER_SIZE + len] = (uint8_t)(crc & 0xFFu);
    buf[RADIO_HEADER_SIZE + len + 1] = (uint8_t)(crc >> 8u);
    return (uint8_t)(RADIO_HEADER_SIZE + len + RADIO_CRC_SIZE);
}

bool RadioPacket::decode(const uint8_t* buf, uint8_t frameLen) {
    if (frameLen < RADIO_MIN_FRAME) return false;
    if (buf[0] != RADIO_SOF) return false;
    uint8_t plen = buf[5];
    if (plen > RADIO_MAX_PAYLOAD) return false;
    if (frameLen != (uint8_t)(RADIO_HEADER_SIZE + plen + RADIO_CRC_SIZE)) return false;

    uint16_t crcCalc = crc16_ccitt(buf + 1, (uint16_t)(5u + plen));
    uint16_t crcRecv = (uint16_t)buf[RADIO_HEADER_SIZE + plen] |
                       ((uint16_t)buf[RADIO_HEADER_SIZE + plen + 1] << 8u);
    if (crcCalc != crcRecv) return false;

    dest = buf[1];
    src = buf[2];
    seq = buf[3];
    type = (PacketType)buf[4];
    len = plen;
    if (len > 0) {
        memcpy(payload, buf + RADIO_HEADER_SIZE, len);
    }
    return true;
}

// --- Lifecycle ---

bool RadioTransport::begin(HC12Driver* driver, const TransportConfig& cfg) {
    _driver = driver;
    _cfg = cfg;

    memset(&_stats, 0, sizeof(_stats));
    memset(&_lastSeq, 0, sizeof(_lastSeq));
    memset(&_seqSeen, 0, sizeof(_seqSeen));
    memset(&_slaves, 0, sizeof(_slaves));
    memset(&_txPacket, 0, sizeof(_txPacket));
    memset(&_rxPacket, 0, sizeof(_rxPacket));

    _txState = TxState::IDLE;
    _rxState = RxState::WAIT_SOF;
    _txSeq = 0;
    _rxQHead = 0;
    _rxQTail = 0;
    _rxCallback = nullptr;
    _txLastTxMs = 0;
    _pendingPower = 0;

    // Initialise cipher unconditionally.
    // Key defaults to all-zero bytes if not set in cfg - replace with your own key.
    _cipher.init(cfg.encryptionKey);
    ESP_LOGI("RadioTransport", "AES-128-CTR encryption initialised");

    return true;
}

// --- Background task ---

bool RadioTransport::startTask(uint32_t stackDepth, UBaseType_t priority, BaseType_t core) {
    if (_taskHandle) return true;  // already running

    _txMutex = xSemaphoreCreateMutex();
    _rxMutex = xSemaphoreCreateMutex();
    _ackEvent = xSemaphoreCreateBinary();
    _txTrigger = xSemaphoreCreateBinary();

    if (!_txMutex || !_rxMutex || !_ackEvent || !_txTrigger) return false;

    return xTaskCreatePinnedToCore(
               _taskFunc, "hc12_transport",
               stackDepth, this, priority, &_taskHandle, core) == pdPASS;
}

void RadioTransport::stopTask() {
    if (_taskHandle) {
        vTaskDelete(_taskHandle);
        _taskHandle = nullptr;
    }
}

void RadioTransport::_taskFunc(void* arg) {
    RadioTransport* self = static_cast<RadioTransport*>(arg);
    SemaphoreHandle_t rxSem = self->_driver->rxSemaphore();

    for (;;) {
        // Block until the UART ISR gives rxSem (new bytes arrived) or 5 ms elapse.
        // The 5 ms timeout is the TX-timeout heartbeat: _updateTx() needs to run
        // often enough to detect ACK timeouts even when no bytes are arriving.
        xSemaphoreTake(rxSem, pdMS_TO_TICKS(5));
        xSemaphoreTake(self->_txTrigger, 0);  // drain without blocking

        // _updateRx does not hold _txMutex.
        // _sendAck() inside _processComplete() is a direct _driver->send() call
        // that bypasses the mutex entirely -- safe from any context.
        self->_updateRx();
        self->_updateTx();
    }
}

// --- RX API ---

void RadioTransport::onReceive(RadioRxCallback cb) {
    _rxCallback = cb;
}

bool RadioTransport::available() const {
    if (_rxCallback != nullptr) return false;
    return _rxQHead != _rxQTail;
}

bool RadioTransport::receive(uint8_t* src, PacketType* type,
                             uint8_t* data, uint8_t* len) {
    if (_rxCallback != nullptr) return false;

    // Brief lock: protect _rxQHead / _rxQTail from concurrent _rxEnqueue().
    if (xSemaphoreTake(_rxMutex, pdMS_TO_TICKS(5)) != pdTRUE) return false;

    bool ok = (_rxQHead != _rxQTail);
    if (ok) {
        const RadioPacket& pkt = _rxQueue[_rxQTail];
        if (src) *src = pkt.src;
        if (type) *type = pkt.type;
        if (len) *len = pkt.len;
        if (data && pkt.len > 0) memcpy(data, pkt.payload, pkt.len);
        _rxQTail = (_rxQTail + 1u) % HC12_RX_QUEUE_DEPTH;
    }

    xSemaphoreGive(_rxMutex);
    return ok;
}

// --- TX API ---

bool RadioTransport::send(uint8_t dest, PacketType type,
                          const uint8_t* data, uint8_t len) {
    if (!sendAsync(dest, type, data, len)) return false;

    // Broadcast: no ACK expected, already done after UART write.
    if (dest == RADIO_ADDR_BROADCAST) return true;

    // Block the calling task until the background task resolves the ACK.
    // The CPU is fully released to other tasks while waiting.
    uint32_t timeoutMs = (uint32_t)_cfg.ackTimeoutMs * (_cfg.retries + 1u) + 200u;
    xSemaphoreTake(_ackEvent, pdMS_TO_TICKS(timeoutMs));
    return _txLastResult;
}

bool RadioTransport::sendAsync(uint8_t dest, PacketType type,
                               const uint8_t* data, uint8_t len) {
    if (xSemaphoreTake(_txMutex, pdMS_TO_TICKS(10)) != pdTRUE) return false;
    bool ok = _sendAsyncInternal(dest, type, data, len);
    xSemaphoreGive(_txMutex);

    // Wake the background task immediately so TX starts without waiting
    // for the next 5 ms heartbeat tick.
    if (ok) xSemaphoreGive(_txTrigger);
    return ok;
}

bool RadioTransport::_sendAsyncInternal(uint8_t dest, PacketType type,
                                        const uint8_t* data, uint8_t len) {
    // Caller must hold _txMutex.
    if (_txState != TxState::IDLE) return false;  // busy

    uint8_t plen = (len > RADIO_MAX_PAYLOAD) ? RADIO_MAX_PAYLOAD : len;

    _txPacket.dest = dest;
    _txPacket.src = _cfg.localAddr;
    _txPacket.seq = _txSeq++;
    _txPacket.type = type;
    _txPacket.len = plen;
    if (plen > 0) memcpy(_txPacket.payload, data, plen);

    // Encrypt DATA payload in place before queuing for TX.
    if (type == PacketType::DATA && plen > 0) {
        uint8_t nonce[RadioCipher::NONCE_SIZE] = {
            _txPacket.dest, _txPacket.src, _txPacket.seq};
        _cipher.crypt(nonce, _txPacket.payload, plen);
    }

    _txRetriesLeft = _cfg.retries;
    _txBroadcast = (dest == RADIO_ADDR_BROADCAST);
    _txLastResult = false;
    _txState = TxState::SENDING;
    return true;
}

bool RadioTransport::isBusy() const {
    return _txState != TxState::IDLE;
}

uint8_t RadioTransport::slavePower(uint8_t slaveAddr) const {
    for (uint8_t i = 0; i < HC12_MAX_SLAVES; i++) {
        if (_slaves[i].used && _slaves[i].addr == slaveAddr) {
            return _slaves[i].currentPower;
        }
    }
    return _cfg.autoPowerMaxP;
}

// --- TX state machine ---

void RadioTransport::_updateTx() {
    xSemaphoreTake(_txMutex, portMAX_DELAY);

    // Apply a deferred power change only when the TX machine is fully idle.
    // setPower() enters AT mode for ~150 ms; we release the mutex around it
    // so sendAsync() callers are not blocked for that duration.
    if (_pendingPower != 0 && _txState == TxState::IDLE) {
        uint8_t p = _pendingPower;
        _pendingPower = 0;
        xSemaphoreGive(_txMutex);
        _driver->setPower(p);
        xSemaphoreTake(_txMutex, portMAX_DELAY);
    }

    switch (_txState) {
        case TxState::IDLE:
            break;

        case TxState::SENDING: {
            // Enforce inter-packet gap
            uint32_t now = millis();
            if (_txLastTxMs > 0 && (now - _txLastTxMs) < _cfg.interPacketGapMs) {
                break;  // wait
            }

            _stats.retries += (_txRetriesLeft < _cfg.retries) ? 1u : 0u;
            if (_txRetriesLeft < _cfg.retries) {
                ESP_LOGD(TAG_RT, "TX retry seq=0x%02X dest=0x%02X retriesLeft=%d",
                         _txPacket.seq, _txPacket.dest, _txRetriesLeft);
            }
            _doSend(_txPacket);  // encode + UART write only, no blocking delay
            ESP_LOGD(TAG_RT, "TX sent seq=0x%02X dest=0x%02X type=0x%02X len=%d",
                     _txPacket.seq, _txPacket.dest, (uint8_t)_txPacket.type, _txPacket.len);

            if (_txBroadcast) {
                _txLastTxMs = millis();
                _txLastResult = true;
                _txState = TxState::IDLE;
                _stats.broadcasts++;
            } else {
                // Release mutex while giving HC-12 time to switch from TX to RX mode.
                // _txState is WAIT_ACK so sendAsync() callers will see busy and return
                // false immediately after taking the mutex.
                _txState = TxState::WAIT_ACK;
                xSemaphoreGive(_txMutex);
                vTaskDelay(pdMS_TO_TICKS(_cfg.txRxSwitchDelayMs));
                xSemaphoreTake(_txMutex, portMAX_DELAY);
                _txSentMs = millis();
                _txLastTxMs = _txSentMs;
            }
            break;
        }

        case TxState::WAIT_ACK:
            if ((millis() - _txSentMs) >= _cfg.ackTimeoutMs) {
                _stats.timeouts++;
                if (_txRetriesLeft > 0) {
                    _txRetriesLeft--;
                    SlaveState* s = _findOrAllocSlave(_txPacket.dest);
                    if (s) s->retryAccum++;
                    _txState = TxState::SENDING;
                    ESP_LOGD(TAG_RT, "TX ACK timeout seq=0x%02X dest=0x%02X -- retrying (%d left)",
                             _txPacket.seq, _txPacket.dest, _txRetriesLeft);
                } else {
                    // All retries exhausted -- fail and unblock send().
                    _txLastResult = false;
                    _txState = TxState::IDLE;
                    _stats.txFailed++;
                    ESP_LOGW(TAG_RT, "TX FAILED seq=0x%02X dest=0x%02X -- all retries exhausted",
                             _txPacket.seq, _txPacket.dest);
                    xSemaphoreGive(_txMutex);
                    xSemaphoreGive(_ackEvent);
                    return;  // mutex already released
                }
            }
            break;

        case TxState::SUCCESS:  // resolved inline in _processComplete(); not reached
            _txLastResult = true;
            _txState = TxState::IDLE;
            break;

        case TxState::RETRY:  // legacy; not reached in event-driven mode
            _txState = TxState::SENDING;
            break;

        case TxState::FAILED:  // resolved inline in _processComplete(); not reached
            _txLastResult = false;
            _txState = TxState::IDLE;
            break;
    }

    // Auto-power evaluation (runs independent of TX state)
    if (_cfg.autoPowerEnabled) {
        uint32_t now = millis();
        for (uint8_t i = 0; i < HC12_MAX_SLAVES; i++) {
            SlaveState& s = _slaves[i];
            if (!s.used) continue;
            if ((now - s.lastCheckMs) >= _cfg.autoPowerIntervalMs) {
                runAutoPower(s.addr);
            }
        }
    }

    xSemaphoreGive(_txMutex);
}

void RadioTransport::_doSend(const RadioPacket& pkt) {
    // Encode the packet and push it to the UART TX FIFO.
    // The txRxSwitchDelayMs pause after this call is handled by _updateTx()
    // with the mutex released so other tasks are not blocked during the wait.
    uint8_t buf[RADIO_MAX_FRAME];
    uint8_t n = pkt.encode(buf);
    _driver->send(buf, n);
}

// --- RX state machine ---

void RadioTransport::_updateRx() {
    // No mutex held here -- _rxBuf is single-producer (ISR) / single-consumer
    // (this task) safe. _processComplete() takes _txMutex briefly only for the
    // ACK/NACK branch; _sendAck() bypasses the mutex entirely.
    while (_driver->available()) {
        uint8_t b;
        _driver->read(&b, 1);

        switch (_rxState) {
            case RxState::WAIT_SOF:
                if (b == RADIO_SOF) {
                    _rxState = RxState::RD_DEST;
                }
                break;

            case RxState::RD_DEST:
                _rxPacket.dest = b;
                _rxState = RxState::RD_SRC;
                break;

            case RxState::RD_SRC:
                _rxPacket.src = b;
                _rxState = RxState::RD_SEQ;
                break;

            case RxState::RD_SEQ:
                _rxPacket.seq = b;
                _rxState = RxState::RD_TYPE;
                break;

            case RxState::RD_TYPE:
                _rxPacket.type = (PacketType)b;
                _rxState = RxState::RD_LEN;
                break;

            case RxState::RD_LEN:
                if (b > RADIO_MAX_PAYLOAD) {
                    // Invalid length -- discard and resync
                    _rxState = RxState::WAIT_SOF;
                } else {
                    _rxPacket.len = b;
                    _rxPayloadIdx = 0;
                    _rxState = (b == 0) ? RxState::RD_CRC1 : RxState::RD_PAYLOAD;
                }
                break;

            case RxState::RD_PAYLOAD:
                _rxPacket.payload[_rxPayloadIdx++] = b;
                if (_rxPayloadIdx >= _rxPacket.len) {
                    _rxState = RxState::RD_CRC1;
                }
                break;

            case RxState::RD_CRC1:
                _rxCrc1 = b;
                _rxState = RxState::RD_CRC2;
                break;

            case RxState::RD_CRC2: {
                // Reconstruct and verify CRC: [DEST SRC SEQ TYPE LEN PAYLOAD]
                uint8_t tmp[5 + RADIO_MAX_PAYLOAD];
                tmp[0] = _rxPacket.dest;
                tmp[1] = _rxPacket.src;
                tmp[2] = _rxPacket.seq;
                tmp[3] = (uint8_t)_rxPacket.type;
                tmp[4] = _rxPacket.len;
                if (_rxPacket.len > 0) memcpy(tmp + 5, _rxPacket.payload, _rxPacket.len);

                uint16_t crcCalc = crc16_ccitt(tmp, (uint16_t)(5u + _rxPacket.len));
                uint16_t crcRecv = (uint16_t)_rxCrc1 | ((uint16_t)b << 8u);

                _rxState = RxState::WAIT_SOF;  // always reset after frame attempt

                if (crcCalc != crcRecv) {
                    _stats.crcErrors++;
                    ESP_LOGW(TAG_RT, "RX CRC error src=0x%02X dest=0x%02X seq=0x%02X",
                             _rxPacket.src, _rxPacket.dest, _rxPacket.seq);
                    break;
                }

                // Address filter: accept only packets for us or broadcast
                if (_rxPacket.dest != _cfg.localAddr &&
                    _rxPacket.dest != RADIO_ADDR_BROADCAST) {
                    break;  // not for us
                }

                _processComplete();
                break;
            }
        }  // switch
    }
}

void RadioTransport::_processComplete() {
    // --- ACK/NACK handling ---
    // These are resolved immediately here so send() is unblocked without
    // waiting for the next _updateTx() heartbeat tick.
    if (_rxPacket.type == PacketType::ACK || _rxPacket.type == PacketType::NACK) {
        bool shouldUnblock = false;

        xSemaphoreTake(_txMutex, portMAX_DELAY);

        if (_txState == TxState::WAIT_ACK &&
            _rxPacket.dest == _cfg.localAddr &&
            _rxPacket.len >= 1 &&
            _rxPacket.payload[0] == _txPacket.seq) {
            if (_rxPacket.type == PacketType::ACK) {
                _txLastResult = true;
                _txState = TxState::IDLE;
                _stats.txPackets++;
                ESP_LOGD(TAG_RT, "RX ACK seq=0x%02X src=0x%02X -- TX success",
                         _rxPacket.seq, _rxPacket.src);
                if (_cfg.autoPowerEnabled && !_txBroadcast) {
                    _findOrAllocSlave(_txPacket.dest);
                }
                shouldUnblock = true;
            } else {
                // NACK: retry or fail
                if (_txRetriesLeft > 0) {
                    _txRetriesLeft--;
                    _txState = TxState::SENDING;
                    ESP_LOGD(TAG_RT, "RX NACK seq=0x%02X src=0x%02X -- retrying",
                             _rxPacket.seq, _rxPacket.src);
                } else {
                    _txLastResult = false;
                    _txState = TxState::IDLE;
                    _stats.txFailed++;
                    shouldUnblock = true;
                    ESP_LOGW(TAG_RT, "RX NACK seq=0x%02X src=0x%02X -- TX FAILED (no retries)",
                             _rxPacket.seq, _rxPacket.src);
                }
            }
        }

        xSemaphoreGive(_txMutex);

        if (shouldUnblock) xSemaphoreGive(_ackEvent);
        return;  // ACK/NACK never delivered to application
    }

    // --- Duplicate detection ---
    if (_seqSeen[_rxPacket.src] &&
        _rxPacket.seq == _lastSeq[_rxPacket.src]) {
        // Duplicate -- discard payload, re-send ACK
        _stats.duplicates++;
        ESP_LOGD(TAG_RT, "RX duplicate seq=0x%02X src=0x%02X -- re-ACKing",
                 _rxPacket.seq, _rxPacket.src);
        _sendAck(_rxPacket.src, _rxPacket.seq, true);
        return;
    }
    _seqSeen[_rxPacket.src] = true;
    _lastSeq[_rxPacket.src] = _rxPacket.seq;

    // --- Send ACK (unicast only) ---
    bool isBroadcast = (_rxPacket.dest == RADIO_ADDR_BROADCAST);
    if (!isBroadcast) {
        _sendAck(_rxPacket.src, _rxPacket.seq, true);
    }

    _stats.rxPackets++;
    ESP_LOGD(TAG_RT, "RX pkt src=0x%02X dest=0x%02X seq=0x%02X type=0x%02X len=%d",
             _rxPacket.src, _rxPacket.dest, _rxPacket.seq,
             (uint8_t)_rxPacket.type, _rxPacket.len);

    // --- Decrypt payload before delivering to the application ---
    // Performed after CRC, address filter, duplicate detection, and ACK.
    // Only DATA frames carry encrypted application payload.
    if (_rxPacket.type == PacketType::DATA && _rxPacket.len > 0) {
        uint8_t nonce[RadioCipher::NONCE_SIZE] = {
            _rxPacket.dest, _rxPacket.src, _rxPacket.seq};
        _cipher.crypt(nonce, _rxPacket.payload, _rxPacket.len);
    }

    // --- Deliver to application ---
    if (_rxCallback) {
        _rxCallback(_rxPacket.src, _rxPacket.type, _rxPacket.payload, _rxPacket.len);
    } else if (!_rxEnqueue(_rxPacket)) {
        ESP_LOGW(TAG_RT, "RX queue full -- packet dropped src=0x%02X seq=0x%02X",
                 _rxPacket.src, _rxPacket.seq);
    }
}

void RadioTransport::_sendAck(uint8_t dest, uint8_t seq, bool ack) {
    // Direct UART write: bypasses the TX state machine and _txMutex entirely.
    // Safe to call from _processComplete() regardless of mutex state.
    RadioPacket ap;
    ap.dest = dest;
    ap.src = _cfg.localAddr;
    ap.seq = seq;
    ap.type = ack ? PacketType::ACK : PacketType::NACK;
    ap.len = 1;
    ap.payload[0] = seq;  // echo back the seq being acknowledged

    uint8_t buf[RADIO_MAX_FRAME];
    uint8_t n = ap.encode(buf);
    _driver->send(buf, n);
    _txLastTxMs = millis();
    _stats.acksSent++;
}

bool RadioTransport::_rxEnqueue(const RadioPacket& pkt) {
    xSemaphoreTake(_rxMutex, portMAX_DELAY);
    uint8_t next = (_rxQHead + 1u) % HC12_RX_QUEUE_DEPTH;
    bool ok = (next != _rxQTail);
    if (ok) {
        _rxQueue[_rxQHead] = pkt;
        _rxQHead = next;
    }
    xSemaphoreGive(_rxMutex);
    return ok;
}

// --- Auto-power control ---

RadioTransport::SlaveState* RadioTransport::_findOrAllocSlave(uint8_t addr) {
    for (uint8_t i = 0; i < HC12_MAX_SLAVES; i++) {
        if (_slaves[i].used && _slaves[i].addr == addr) return &_slaves[i];
    }
    for (uint8_t i = 0; i < HC12_MAX_SLAVES; i++) {
        if (!_slaves[i].used) {
            _slaves[i].used = true;
            _slaves[i].addr = addr;
            _slaves[i].currentPower = _cfg.autoPowerMaxP;
            _slaves[i].retryAccum = 0;
            _slaves[i].cleanIntervalCount = 0;
            _slaves[i].lastCheckMs = millis();
            return &_slaves[i];
        }
    }
    return nullptr;  // no free slot
}

void RadioTransport::runAutoPower(uint8_t slaveAddr) {
    if (!_cfg.autoPowerEnabled || !_driver) return;

    SlaveState* s = _findOrAllocSlave(slaveAddr);
    if (!s) return;

    uint32_t now = millis();
    if ((now - s->lastCheckMs) < _cfg.autoPowerIntervalMs) return;
    s->lastCheckMs = now;

    bool changed = false;

    if (s->retryAccum >= _cfg.autoPowerHighThresh) {
        if (s->currentPower < _cfg.autoPowerMaxP) {
            s->currentPower++;
            changed = true;
        }
        s->cleanIntervalCount = 0;
    } else if (s->retryAccum == 0) {
        s->cleanIntervalCount++;
        if (s->cleanIntervalCount >= _cfg.autoPowerCleanSteps) {
            if (s->currentPower > _cfg.autoPowerMinP) {
                s->currentPower--;
                changed = true;
            }
            s->cleanIntervalCount = 0;
        }
    } else {
        s->cleanIntervalCount = 0;
    }

    s->retryAccum = 0;

    if (changed) {
        uint8_t globalMaxP = _cfg.autoPowerMinP;
        for (uint8_t i = 0; i < HC12_MAX_SLAVES; i++) {
            if (_slaves[i].used && _slaves[i].currentPower > globalMaxP) {
                globalMaxP = _slaves[i].currentPower;
            }
        }

        if (_driver->getConfig().power != globalMaxP) {
            // Schedule the AT-mode power change for the next IDLE cycle in
            // _updateTx() so it never runs while the mutex is held.
            _pendingPower = globalMaxP;
        }
    }
}
