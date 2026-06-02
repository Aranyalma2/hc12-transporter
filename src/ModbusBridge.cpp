/**
 * @file ModbusBridge.cpp
 * @brief ModbusBridge implementation — Modbus RTU ↔ RadioTransport mapping.
 */

#include "ModbusBridge.h"

#include <string.h>

// --- Lifecycle ---

bool ModbusBridge::begin(RadioTransport* transport, uint8_t localRadioAddr) {
    _transport = transport;
    _localRadioAddr = localRadioAddr;

    memset(_routes, 0, sizeof(_routes));
    memset(_localDevices, 0, sizeof(_localDevices));
    memset(_responseQueue, 0, sizeof(_responseQueue));
    memset(_requestQueue, 0, sizeof(_requestQueue));

    _routeCount = 0;
    _localDeviceCount = 0;
    _respQHead = _respQTail = 0;
    _reqQHead = _reqQTail = 0;

    return (transport != nullptr);
}

// --- Master — Route table ---

bool ModbusBridge::addRoute(uint8_t modbusAddr, uint8_t radioAddr) {
    // Update existing entry if modbusAddr already mapped
    for (uint8_t i = 0; i < _routeCount; i++) {
        if (_routes[i].modbusAddr == modbusAddr) {
            _routes[i].radioAddr = radioAddr;
            return true;
        }
    }
    if (_routeCount >= MODBUS_MAX_ROUTES) return false;
    _routes[_routeCount].modbusAddr = modbusAddr;
    _routes[_routeCount].radioAddr = radioAddr;
    _routeCount++;
    return true;
}

void ModbusBridge::removeRoute(uint8_t modbusAddr) {
    for (uint8_t i = 0; i < _routeCount; i++) {
        if (_routes[i].modbusAddr == modbusAddr) {
            // Compact the table
            _routes[i] = _routes[--_routeCount];
            return;
        }
    }
}

void ModbusBridge::clearRoutes() {
    _routeCount = 0;
}

uint8_t ModbusBridge::lookupRadioAddr(uint8_t modbusAddr) const {
    for (uint8_t i = 0; i < _routeCount; i++) {
        if (_routes[i].modbusAddr == modbusAddr) {
            return _routes[i].radioAddr;
        }
    }
    return 0;  // not found
}

// --- Master — Discovery ---

uint8_t ModbusBridge::discover(uint32_t timeoutMs) {
    if (!_transport) return 0;

    // Broadcast an empty PING
    _transport->send(RADIO_ADDR_BROADCAST, PacketType::PING, nullptr, 0);

    // Collect PONG replies
    uint8_t startCount = _routeCount;
    uint32_t t0 = millis();

    while (millis() - t0 < timeoutMs) {
        update();  // Let ModbusBridge::update() process all packets, avoiding DATA drop
    }

    return _routeCount - startCount;
}

// --- Master — TX ---

bool ModbusBridge::sendModbus(uint8_t modbusAddr, const uint8_t* frame, uint8_t len) {
    uint8_t radioAddr = lookupRadioAddr(modbusAddr);
    if (radioAddr == 0) return false;  // no route
    return sendModbusDirect(radioAddr, frame, len);
}

bool ModbusBridge::sendModbusDirect(uint8_t radioAddr, const uint8_t* frame, uint8_t len) {
    if (!_transport) return false;
    if (len > RADIO_MAX_PAYLOAD) return false;
    return _transport->send(radioAddr, PacketType::DATA, frame, len);
}

// --- Master — RX ---

bool ModbusBridge::modbusAvailable() const {
    return _respQHead != _respQTail;
}

bool ModbusBridge::receiveModbus(ModbusFrame* frame) {
    return _dequeueFrame(_responseQueue, _respQHead, _respQTail, frame);
}

// --- Slave — Registration ---

bool ModbusBridge::registerLocalDevice(uint8_t modbusAddr) {
    // Check if already registered
    for (uint8_t i = 0; i < _localDeviceCount; i++) {
        if (_localDevices[i] == modbusAddr) return true;
    }
    if (_localDeviceCount >= MODBUS_MAX_LOCAL_DEVICES) return false;
    _localDevices[_localDeviceCount++] = modbusAddr;
    return true;
}

void ModbusBridge::unregisterLocalDevice(uint8_t modbusAddr) {
    for (uint8_t i = 0; i < _localDeviceCount; i++) {
        if (_localDevices[i] == modbusAddr) {
            _localDevices[i] = _localDevices[--_localDeviceCount];
            return;
        }
    }
}

// --- Slave — RX ---

bool ModbusBridge::requestAvailable() const {
    return _reqQHead != _reqQTail;
}

bool ModbusBridge::receiveRequest(ModbusFrame* frame) {
    return _dequeueFrame(_requestQueue, _reqQHead, _reqQTail, frame);
}

// --- Slave — TX ---

bool ModbusBridge::sendResponse(uint8_t masterRadioAddr,
                                const uint8_t* frame, uint8_t len) {
    if (!_transport) return false;
    if (len > RADIO_MAX_PAYLOAD) return false;
    return _transport->send(masterRadioAddr, PacketType::DATA, frame, len);
}

// --- Common — update() ---

void ModbusBridge::update() {
    if (!_transport) return;
    _transport->update();

    uint8_t src;
    PacketType type;
    uint8_t payload[RADIO_MAX_PAYLOAD];
    uint8_t len;

    while (_transport->receive(&src, &type, payload, &len)) {
        // Build a temporary RadioPacket for the handlers
        RadioPacket pkt;
        pkt.src = src;
        pkt.dest = _localRadioAddr;
        pkt.type = type;
        pkt.len = len;
        if (len > 0) memcpy(pkt.payload, payload, len);

        switch (type) {
            case PacketType::PING:
                _handlePing(pkt);
                break;
            case PacketType::PONG:
                _handlePong(pkt);
                break;
            case PacketType::DATA:
                _handleData(pkt);
                break;
            default:
                break;
        }
    }
}

// --- Private helpers ---

void ModbusBridge::_handlePing(const RadioPacket& pkt) {
    // Slave responds to PING with a PONG listing local Modbus addresses
    if (_localDeviceCount == 0) return;

    uint8_t resp[RADIO_MAX_PAYLOAD];
    resp[0] = _localRadioAddr;

    uint8_t count = _localDeviceCount;
    if (count > RADIO_MAX_PAYLOAD - 2) count = RADIO_MAX_PAYLOAD - 2;

    resp[1] = count;
    for (uint8_t i = 0; i < count; i++) {
        resp[2 + i] = _localDevices[i];
    }
    uint8_t respLen = (uint8_t)(2u + count);

    // Reply to the pinger (master's radio address is in pkt.src, or broadcast -> reply to master)
    uint8_t replyTo = (pkt.src != 0) ? pkt.src : RADIO_ADDR_MASTER;
    _transport->send(replyTo, PacketType::PONG, resp, respLen);
}

void ModbusBridge::_handlePong(const RadioPacket& pkt) {
    // Master receives PONGs during discover() — handled in discover() spin loop.
    // If a PONG arrives outside discover(), we still update the route table.
    if (pkt.len < 2) return;
    uint8_t count = pkt.payload[1];
    for (uint8_t i = 0; i < count && (i + 2u) < pkt.len; i++) {
        addRoute(pkt.payload[2 + i], pkt.src);
    }
}

void ModbusBridge::_handleData(const RadioPacket& pkt) {
    ModbusFrame f;
    f.radioSrc = pkt.src;
    f.len = pkt.len;
    if (pkt.len > 0) memcpy(f.data, pkt.payload, pkt.len);

    bool isMaster = (_localRadioAddr == RADIO_ADDR_MASTER);

    if (isMaster) {
        // Master receives a response from a slave
        _enqueueFrame(_responseQueue, _respQHead, _respQTail, f);
    } else {
        // Slave receives a request from master
        _enqueueFrame(_requestQueue, _reqQHead, _reqQTail, f);
    }
}

bool ModbusBridge::_isLocalDevice(uint8_t modbusAddr) const {
    for (uint8_t i = 0; i < _localDeviceCount; i++) {
        if (_localDevices[i] == modbusAddr) return true;
    }
    return false;
}

bool ModbusBridge::_enqueueFrame(ModbusFrame* queue, uint8_t& head, uint8_t tail,
                                 const ModbusFrame& f) {
    uint8_t next = (head + 1u) % FRAME_QUEUE_DEPTH;
    if (next == tail) return false;  // full
    queue[head] = f;
    head = next;
    return true;
}

bool ModbusBridge::_dequeueFrame(ModbusFrame* queue, uint8_t head, uint8_t& tail,
                                 ModbusFrame* out) {
    if (head == tail) return false;  // empty
    if (out) *out = queue[tail];
    tail = (tail + 1u) % FRAME_QUEUE_DEPTH;
    return true;
}
