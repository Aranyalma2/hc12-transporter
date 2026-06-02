/**
 * modbus_master.ino — Modbus RTU master via RadioTransport + ModbusBridge
 *
 * Demonstrates:
 *  - ModbusBridge on the master side
 *  - Broadcast discovery of slave nodes and their Modbus addresses
 *  - Sending Modbus RTU read-holding-registers requests
 *  - Receiving and printing Modbus RTU responses
 *
 * Wiring:
 *   ESP32 GPIO17 (TX1) -> HC-12 RX
 *   ESP32 GPIO16 (RX1) -> HC-12 TX
 *   ESP32 GPIO4        -> HC-12 SET
 *
 * The Modbus framing itself (building FC03 requests etc.) is done here
 * manually for illustration. Use a Modbus library for production.
 */

#include <HC12Transporter.h>
#include <ModbusBridge.h>

// --- Pins ---
static constexpr uint8_t HC12_SET_PIN = 4;
static constexpr uint8_t HC12_RX_PIN = 16;
static constexpr uint8_t HC12_TX_PIN = 17;

// --- Objects ---
HC12Driver radio;
RadioTransport transport;
ModbusBridge bridge;

// --- Modbus CRC16 (standard Modbus poly 0x8005) for RTU frame integrity ---
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

// Build a FC03 Read Holding Registers request frame
static uint8_t buildFC03(uint8_t devAddr, uint16_t startReg, uint16_t count,
                         uint8_t* out) {
    out[0] = devAddr;
    out[1] = 0x03;
    out[2] = (uint8_t)(startReg >> 8);
    out[3] = (uint8_t)(startReg & 0xFF);
    out[4] = (uint8_t)(count >> 8);
    out[5] = (uint8_t)(count & 0xFF);
    uint16_t crc = modbusCRC(out, 6);
    out[6] = (uint8_t)(crc & 0xFF);
    out[7] = (uint8_t)(crc >> 8);
    return 8;
}

// --- Polling state ---
static uint8_t pollSlaves[] = {0x01, 0x02, 0x05};  // Modbus device addresses to poll
static uint8_t pollCount = 3;
static uint8_t pollIdx = 0;
static uint32_t lastPollMs = 0;
static bool waitingResponse = false;
static uint32_t responseDue = 0;
static uint8_t lastPollDev = 0;

void setup() {
    Serial.begin(115200);
    Serial.println(F("[MASTER] Modbus bridge example"));

    HC12Config hcCfg = {.channel = 5, .power = 8, .mode = HC12Mode::FU3, .baud = 19200};
    if (!radio.begin(&Serial1, HC12_SET_PIN, HC12_RX_PIN, HC12_TX_PIN, hcCfg)) {
        Serial.println(F("[MASTER] HC-12 FAILED"));
        while (true) delay(1000);
    }

    TransportConfig tCfg = TRANSPORT_DEFAULT_CONFIG;
    tCfg.localAddr = RADIO_ADDR_MASTER;
    tCfg.autoPowerEnabled = true;
    tCfg.autoPowerIntervalMs = 5000;
    transport.begin(&radio, tCfg);

    bridge.begin(&transport, RADIO_ADDR_MASTER);

    // --- Discover slaves ---
    Serial.println(F("[MASTER] Broadcasting discovery PING..."));
    uint8_t found = bridge.discover(800);
    Serial.printf("[MASTER] Found %d slave node(s)\n", found);

    // You can also add static routes in case discovery is skipped:
    // bridge.addRoute(0x01, 0x10);
    // bridge.addRoute(0x02, 0x10);
    // bridge.addRoute(0x05, 0x11);
}

void loop() {
    bridge.update();

    // --- Check for a Modbus response ---
    if (waitingResponse) {
        ModbusFrame resp;
        if (bridge.receiveModbus(&resp)) {
            waitingResponse = false;
            Serial.printf("[MASTER] Response from radio 0x%02X (%d bytes): ", resp.radioSrc, resp.len);
            for (uint8_t i = 0; i < resp.len; i++) Serial.printf("%02X ", resp.data[i]);
            Serial.println();
        } else if (millis() > responseDue) {
            // Application-level timeout (transport already retried)
            Serial.printf("[MASTER] Timeout waiting for Modbus dev 0x%02X\n", lastPollDev);
            waitingResponse = false;
        }
        return;
    }

    // --- Poll next slave every 500 ms ---
    if (millis() - lastPollMs < 500) return;
    lastPollMs = millis();

    uint8_t devAddr = pollSlaves[pollIdx];
    pollIdx = (pollIdx + 1) % pollCount;

    // Build FC03: read 2 holding registers starting at 0x0000
    uint8_t frame[8];
    uint8_t flen = buildFC03(devAddr, 0x0000, 2, frame);

    Serial.printf("[MASTER] Polling Modbus dev 0x%02X ... ", devAddr);
    bool sent = bridge.sendModbus(devAddr, frame, flen);
    if (!sent) {
        Serial.println(F("No route / transport error"));
        return;
    }
    Serial.println(F("sent, waiting response..."));
    lastPollDev = devAddr;
    waitingResponse = true;
    responseDue = millis() + 500;  // 500 ms application timeout
}
