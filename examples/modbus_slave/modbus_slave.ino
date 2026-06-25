/**
 * modbus_slave.ino -- Modbus RTU slave via RadioTransport + ModbusSlave
 *
 * Demonstrates two RX modes of ModbusSlave:
 *
 *  Mode A -- Local callback mode (default in this example):
 *    onRequest() callback is called for every incoming Modbus frame.
 *    The callback fills a response buffer which is sent back to the master.
 *    Use this when the ESP32 itself is the Modbus slave device.
 *
 *  Mode B -- Serial bridge mode (uncomment SERIAL_BRIDGE_MODE below):
 *    Attach a second UART connected to a real RS-485 Modbus device.
 *    Incoming frames are forwarded to RS-485; the response is read back
 *    and sent to the master automatically.
 *    The slave does not inspect Modbus addresses at all.
 *
 * Wiring (HC-12):
 *   ESP32 GPIO33 (TX1) -> HC-12 RX
 *   ESP32 GPIO32 (RX1) -> HC-12 TX
 *   ESP32 GPIO12       -> HC-12 SET
 *
 * Wiring (Serial bridge, Mode B):
 *   ESP32 GPIO17 (TX2) -> MAX3485 DI (RS-485 TX)
 *   ESP32 GPIO16 (RX2) -> MAX3485 RO (RS-485 RX)
 *   (DE/RE driven by a separate GPIO or wired to transmit always, depending on hardware)
 */

#include <HC12Transporter.h>
#include <radiobridges/ModbusSlave.h>

// Uncomment to enable RS-485 serial bridge mode (see Mode B above)
// #define SERIAL_BRIDGE_MODE

// --- HC-12 pins ---
static constexpr uint8_t HC12_SET_PIN = 12;
static constexpr uint8_t HC12_RX_PIN = 32;
static constexpr uint8_t HC12_TX_PIN = 33;

// --- This slave's radio address ---
static constexpr uint8_t MY_RADIO_ADDR = 0x10;
static const uint8_t SHARED_KEY[16] = {0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30}; // "0000000000000000"

// --- Objects ---
HC12Driver radio;
RadioTransport transport;
ModbusSlave bridge;

// --- Modbus CRC16 ---
static uint16_t modbusCRC(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

// --- Local callback (Mode A) ---
// The slave does not care which Modbus address is being addressed.
// Handle the request and fill the response buffer.
static void handleRequest(uint8_t radioSrc,
                          const uint8_t* req, uint8_t reqLen,
                          uint8_t* resp, uint8_t* respLen) {
    *respLen = 0;

    // Basic frame validation
    if (reqLen < 4) return;
    uint16_t crcCalc = modbusCRC(req, reqLen - 2);
    uint16_t crcRecv = (uint16_t)req[reqLen - 2] | ((uint16_t)req[reqLen - 1] << 8);
    if (crcCalc != crcRecv) {
        Serial.println(F("[SLAVE] Modbus CRC error"));
        return;
    }

    uint8_t devAddr = req[0];
    uint8_t fc = req[1];

    Serial.printf("[SLAVE] Request from radio 0x%02X: dev=0x%02X fc=0x%02X (%d bytes)\n",
                  radioSrc, devAddr, fc, reqLen);

    if (fc == 0x03 && reqLen == 8) {
        // FC03: Read Holding Registers -- simulate a response with fake data.
        uint16_t startReg = ((uint16_t)req[2] << 8) | req[3];
        uint8_t regCount = req[5];  // low byte

        resp[0] = devAddr;
        resp[1] = 0x03;
        resp[2] = (uint8_t)(regCount * 2);
        for (uint8_t i = 0; i < regCount; i++) {
            uint16_t val = (uint16_t)((startReg + i) * 10);
            resp[3 + i * 2] = (uint8_t)(val >> 8);
            resp[3 + i * 2 + 1] = (uint8_t)(val & 0xFF);
        }
        uint8_t rlen = (uint8_t)(3u + regCount * 2u);
        uint16_t crc = modbusCRC(resp, rlen);
        resp[rlen] = (uint8_t)(crc & 0xFF);
        resp[rlen + 1] = (uint8_t)(crc >> 8);
        *respLen = rlen + 2;
    } else {
        Serial.printf("[SLAVE] Unsupported function code 0x%02X\n", fc);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.printf("[SLAVE 0x%02X] Modbus bridge example\n", MY_RADIO_ADDR);

    pinMode(35, INPUT);

    // --- HC-12 ---
    HC12Config hcCfg;
    hcCfg.channel = 5;
    hcCfg.power = 8;
    hcCfg.mode = HC12Mode::FU3;
    hcCfg.baud = 19200;

    if (!radio.begin(&Serial1, HC12_SET_PIN, HC12_RX_PIN, HC12_TX_PIN, hcCfg)) {
        Serial.println(F("[SLAVE] HC-12 FAILED"));
    }

    // --- Transport ---
    TransportConfig tCfg = TRANSPORT_DEFAULT_CONFIG;
    tCfg.localAddr = MY_RADIO_ADDR;
    tCfg.retries = 3;
    tCfg.ackTimeoutMs = 75;
    tCfg.autoPowerEnabled = false;
    memcpy(tCfg.encryptionKey, SHARED_KEY, 16);

    transport.begin(&radio, tCfg);
    transport.startTask();

    // --- Modbus bridge ---
    bridge.begin(&transport);

#ifdef SERIAL_BRIDGE_MODE
    // Mode B: transparent RS-485 bridge.
    // All incoming radio frames are forwarded to Serial2 byte-for-byte.
    Serial2.begin(9600, SERIAL_8N1, 16, 17);
    bridge.attachSerial(&Serial2,
                        500 /* responseTimeoutMs */,
                        5 /* silenceMs */);
    bridge.startTask();
    Serial.println(F("[SLAVE] Serial bridge mode active on Serial2"));

#else
    // Mode A: local callback handles all incoming Modbus frames.
    bridge.onRequest(handleRequest);
    bridge.startTask();
    Serial.println(F("[SLAVE] Local callback mode active"));
#endif
}

void loop() {
    // The bridge task handles everything. loop() is free for other logic.
    static uint32_t lastStatMs = 0;
    if (millis() - lastStatMs >= 30000) {
        lastStatMs = millis();
        const LinkStats& st = transport.stats();
        Serial.printf("[STATS] rx=%lu tx=%lu crcErr=%lu dup=%lu\n",
                      st.rxPackets, st.txPackets, st.crcErrors, st.duplicates);
    }

    if (digitalRead(35)) {
        ESP.restart();
    }
}
