/**
 * @file RadioTransport.cpp
 * @brief RadioTransport — TX/RX state machines, ACK/retry, auto-power.
 */

#include "RadioTransport.h"

#include "CRC16.h"

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

    // Verify CRC
    uint16_t crcCalc = crc16_ccitt(buf + 1, (uint16_t)(5u + plen));
    uint16_t crcRecv = (uint16_t)buf[RADIO_HEADER_SIZE + plen] | ((uint16_t)buf[RADIO_HEADER_SIZE + plen + 1] << 8u);
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
    _txLastTxMs = 0;

    return true;
}

// --- TX API ---

bool RadioTransport::send(uint8_t dest, PacketType type,
                          const uint8_t* data, uint8_t len) {
    if (!sendAsync(dest, type, data, len)) return false;

    // Spin until done (blocking)
    uint32_t startMs = millis();
    uint32_t timeoutMs = (uint32_t)_cfg.ackTimeoutMs * (_cfg.retries + 1u) + 200u;

    while (millis() - startMs < timeoutMs) {
        update();
        if (!isBusy()) return _txLastResult;
    }

    // Timeout fallback
    _txState = TxState::IDLE;
    _txLastResult = false;
    return false;
}

bool RadioTransport::sendAsync(uint8_t dest, PacketType type,
                               const uint8_t* data, uint8_t len) {
    if (_txState != TxState::IDLE) return false;  // busy

    uint8_t plen = (len > RADIO_MAX_PAYLOAD) ? RADIO_MAX_PAYLOAD : len;

    _txPacket.dest = dest;
    _txPacket.src = _cfg.localAddr;
    _txPacket.seq = _txSeq++;
    _txPacket.type = type;
    _txPacket.len = plen;
    if (plen > 0) memcpy(_txPacket.payload, data, plen);

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

// --- RX API ---

bool RadioTransport::available() const {
    return _rxQHead != _rxQTail;
}

bool RadioTransport::receive(uint8_t* src, PacketType* type,
                             uint8_t* data, uint8_t* len) {
    if (_rxQHead == _rxQTail) return false;

    const RadioPacket& pkt = _rxQueue[_rxQTail];
    if (src) *src = pkt.src;
    if (type) *type = pkt.type;
    if (len) *len = pkt.len;
    if (data && pkt.len > 0) memcpy(data, pkt.payload, pkt.len);

    _rxQTail = (_rxQTail + 1u) % HC12_RX_QUEUE_DEPTH;
    return true;
}

// --- Main engine ---

void RadioTransport::update() {
    if (_driver) _driver->update();  // pump UART bytes into ring buffer
    _updateRx();
    _updateTx();
}

// --- TX state machine ---

void RadioTransport::_updateTx() {
    switch (_txState) {
        case TxState::IDLE:
            break;

        case TxState::SENDING: {
            // Enforce inter-packet gap
            uint32_t now = millis();
            if (_txLastTxMs > 0 && (now - _txLastTxMs) < _cfg.interPacketGapMs) {
                break;  // wait
            }

            _doSend(_txPacket);
            _txSentMs = millis();
            _txLastTxMs = _txSentMs;
            _stats.retries += (_txRetriesLeft < _cfg.retries) ? 1u : 0u;

            if (_txBroadcast) {
                // Broadcast: no ACK, done
                _txLastResult = true;
                _txState = TxState::IDLE;
                _stats.broadcasts++;
            } else {
                _txState = TxState::WAIT_ACK;
            }
            break;
        }

        case TxState::WAIT_ACK:
            if ((millis() - _txSentMs) >= _cfg.ackTimeoutMs) {
                _stats.timeouts++;
                if (_txRetriesLeft > 0) {
                    _txRetriesLeft--;
                    // Track for auto-power
                    SlaveState* s = _findOrAllocSlave(_txPacket.dest);
                    if (s) s->retryAccum++;
                    _txState = TxState::SENDING;
                } else {
                    _txState = TxState::FAILED;
                    _stats.txFailed++;
                }
            }
            break;

        case TxState::SUCCESS:
            _txLastResult = true;
            _txState = TxState::IDLE;
            _stats.txPackets++;
            if (_cfg.autoPowerEnabled && !_txBroadcast) {
                _findOrAllocSlave(_txPacket.dest);
            }
            break;

        case TxState::RETRY:
            // Should not reach here directly; handled in WAIT_ACK
            _txState = TxState::SENDING;
            break;

        case TxState::FAILED:
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
}

void RadioTransport::_doSend(const RadioPacket& pkt) {
    uint8_t buf[RADIO_MAX_FRAME];
    uint8_t n = pkt.encode(buf);
    _driver->send(buf, n);
    delay(_cfg.txRxSwitchDelayMs);  // give HC-12 time to switch RX
}

// --- RX state machine ---

void RadioTransport::_updateRx() {
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
                    // Invalid length — discard and resync
                    _rxState = RxState::WAIT_SOF;
                } else {
                    _rxPacket.len = b;
                    _rxPayloadIdx = 0;
                    if (b == 0) {
                        _rxState = RxState::RD_CRC1;
                    } else {
                        _rxState = RxState::RD_PAYLOAD;
                    }
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
                // Reconstruct and verify CRC
                // Build a temp buffer for CRC check: [DEST SRC SEQ TYPE LEN PAYLOAD]
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
    // --- ACK/NACK handling (terminates WAIT_ACK) ---
    if (_rxPacket.type == PacketType::ACK || _rxPacket.type == PacketType::NACK) {
        if (_txState == TxState::WAIT_ACK &&
            _rxPacket.dest == _cfg.localAddr &&
            _rxPacket.len >= 1 &&
            _rxPacket.payload[0] == _txPacket.seq) {
            if (_rxPacket.type == PacketType::ACK) {
                _txState = TxState::SUCCESS;
            } else {
                // NACK: retry immediately
                if (_txRetriesLeft > 0) {
                    _txRetriesLeft--;
                    _txState = TxState::SENDING;
                } else {
                    _txState = TxState::FAILED;
                }
            }
        }
        return;  // ACK/NACK never delivered to application
    }

    // --- Duplicate detection ---
    if (_seqSeen[_rxPacket.src] &&
        _rxPacket.seq == _lastSeq[_rxPacket.src]) {
        // Duplicate — discard payload, re-send ACK
        _stats.duplicates++;
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

    // --- Deliver to application queue ---
    if (!_rxEnqueue(_rxPacket)) {
        // Queue full — packet dropped (stats could be added here)
    }
}

void RadioTransport::_sendAck(uint8_t dest, uint8_t seq, bool ack) {
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
    uint8_t next = (_rxQHead + 1u) % HC12_RX_QUEUE_DEPTH;
    if (next == _rxQTail) return false;  // full
    _rxQueue[_rxQHead] = pkt;
    _rxQHead = next;
    return true;
}

// --- Auto-power control ---

RadioTransport::SlaveState* RadioTransport::_findOrAllocSlave(uint8_t addr) {
    for (uint8_t i = 0; i < HC12_MAX_SLAVES; i++) {
        if (_slaves[i].used && _slaves[i].addr == addr) return &_slaves[i];
    }
    // Allocate new slot
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
        // High retry rate -> increase power
        if (s->currentPower < _cfg.autoPowerMaxP) {
            s->currentPower++;
            changed = true;
        }
        s->cleanIntervalCount = 0;
    } else if (s->retryAccum == 0) {
        // Clean interval -> increment clean counter
        s->cleanIntervalCount++;
        if (s->cleanIntervalCount >= _cfg.autoPowerCleanSteps) {
            // Decrease power after enough clean intervals
            if (s->currentPower > _cfg.autoPowerMinP) {
                s->currentPower--;
                changed = true;
            }
            s->cleanIntervalCount = 0;
        }
    } else {
        // Moderate retries -> stay, reset clean counter
        s->cleanIntervalCount = 0;
    }

    // Reset accumulator for next interval
    s->retryAccum = 0;

    // Apply the highest required power across all nodes to the hardware
    if (changed) {
        uint8_t globalMaxP = _cfg.autoPowerMinP;
        for (uint8_t i = 0; i < HC12_MAX_SLAVES; i++) {
            if (_slaves[i].used && _slaves[i].currentPower > globalMaxP) {
                globalMaxP = _slaves[i].currentPower;
            }
        }

        if (_driver->getConfig().power != globalMaxP) {
            _driver->setPower(globalMaxP);
        }
    }
}
