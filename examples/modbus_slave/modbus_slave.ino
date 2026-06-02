/**
 * modbus_slave.ino — Modbus RTU slave via RadioTransport + ModbusBridge
 *
 * Demonstrates:
 *  - ModbusBridge on the slave side
 *  - Registering multiple Modbus device addresses on one radio node
 *  - Responding to discovery PING with the device list
 *  - Receiving Modbus requests from master and forwarding to local RS-485
 *    (or, for this demo, simulating a response internally)
 *
 * Wiring:
 *   ESP32 GPIO17 (TX1) -> HC-12 RX
 *   ESP32 GPIO16 (RX1) -> HC-12 TX
 *   ESP32 GPIO4        -> HC-12 SET
 *
 * This node serves Modbus addresses 0x01 and 0x02 over radio address 0x10.
 *
 * For real RS-485 forwarding, replace the simulated response with a
 * HardwareSerial write to your MAX3485 and read back the response.
 */

#include <HC12Transporter.h>
#include <ModbusBridge.h>

// --- Pins ---
static constexpr uint8_t HC12_SET_PIN = 4;
static constexpr uint8_t HC12_RX_PIN = 16;
static constexpr uint8_t HC12_TX_PIN = 17;

// --- This slave's radio address ---
static constexpr uint8_t MY_RADIO_ADDR = 0x10;

// --- Modbus device addresses served by this radio node ---
static constexpr uint8_t MODBUS_DEVICES[] = {0x01, 0x02};
static constexpr uint8_t MODBUS_DEV_COUNT = 2;

// --- Objects ---
HC12Driver radio;
RadioTransport transport;
ModbusBridge bridge;

// --- Modbus CRC16 ---
static uint16_t modbusCRC(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

/**
 * @brief Simulate a Modbus FC03 response for demonstration.
 * In production, write request to RS-485, wait for and read actual response.
 */
static uint8_t simulateFC03Response(const uint8_t* req, uint8_t reqLen,
                                    uint8_t* resp) {
    if (reqLen < 8 || req[1] != 0x03) return 0;

    uint8_t devAddr = req[0];
    uint8_t regCount = req[5];  // low byte of register count

    resp[0] = devAddr;
    resp[1] = 0x03;
    resp[2] = (uint8_t)(regCount * 2);  // byte count
    // Fill registers with fake values (register value = reg address * 10)
    uint16_t startReg = ((uint16_t)req[2] << 8) | req[3];
    for (uint8_t i = 0; i < regCount; i++) {
        uint16_t val = (uint16_t)((startReg + i) * 10);
        resp[3 + i * 2] = (uint8_t)(val >> 8);
        resp[3 + i * 2 + 1] = (uint8_t)(val & 0xFF);
    }
    uint8_t respLen = (uint8_t)(3u + regCount * 2u);
    uint16_t crc = modbusCRC(resp, respLen);
    resp[respLen] = (uint8_t)(crc & 0xFF);
    resp[respLen + 1] = (uint8_t)(crc >> 8);
    return (uint8_t)(respLen + 2u);
}

void setup() {
    Serial.begin(115200);
    Serial.printf("[SLAVE 0x%02X] Modbus bridge example\n", MY_RADIO_ADDR);

    HC12Config hcCfg = {.channel = 5, .power = 8, .mode = HC12Mode::FU3, .baud = 19200};
    if (!radio.begin(&Serial1, HC12_SET_PIN, HC12_RX_PIN, HC12_TX_PIN, hcCfg)) {
        Serial.println(F("[SLAVE] HC-12 FAILED"));
        while (true) delay(1000);
    }

    TransportConfig tCfg = TRANSPORT_DEFAULT_CONFIG;
    tCfg.localAddr = MY_RADIO_ADDR;
    tCfg.retries = 3;
    tCfg.ackTimeoutMs = 75;
    transport.begin(&radio, tCfg);

    bridge.begin(&transport, MY_RADIO_ADDR);

    // Register which Modbus addresses this node serves
    for (uint8_t i = 0; i < MODBUS_DEV_COUNT; i++) {
        bridge.registerLocalDevice(MODBUS_DEVICES[i]);
        Serial.printf("[SLAVE] Registered Modbus device 0x%02X\n", MODBUS_DEVICES[i]);
    }

    Serial.println(F("[SLAVE] Ready"));
}

void loop() {
    bridge.update();

    ModbusFrame req;
    while (bridge.requestAvailable()) {
        if (!bridge.receiveRequest(&req)) break;

        Serial.printf("[SLAVE] Request from radio 0x%02X (%d bytes): ", req.radioSrc, req.len);
        for (uint8_t i = 0; i < req.len; i++) Serial.printf("%02X ", req.data[i]);
        Serial.println();

        // Validate Modbus CRC
        if (req.len >= 4) {
            uint16_t crcCalc = modbusCRC(req.data, req.len - 2);
            uint16_t crcRecv = (uint16_t)req.data[req.len - 2] | ((uint16_t)req.data[req.len - 1] << 8);
            if (crcCalc != crcRecv) {
                Serial.println(F("[SLAVE] Modbus CRC error — discarding"));
                continue;
            }
        }

        // Generate response (simulated; replace with real RS-485 exchange)
        uint8_t resp[RADIO_MAX_PAYLOAD];
        uint8_t rlen = simulateFC03Response(req.data, req.len, resp);
        if (rlen == 0) {
            Serial.println(F("[SLAVE] Unsupported Modbus function"));
            continue;
        }

        Serial.printf("[SLAVE] Sending response (%d bytes) to radio 0x%02X\n", rlen, req.radioSrc);
        bridge.sendResponse(req.radioSrc, resp, rlen);
    }
}
