/**
 * modbus_master.ino -- Modbus RTU master via RadioTransport + ModbusMaster
 *
 * Demonstrates two TX modes of ModbusMaster:
 *
 *  Mode A -- API mode (default in this example):
 *    Build Modbus RTU frames manually and call bridge.send().
 *    Responses arrive in the bridge.receive() queue.
 *
 *  Mode B -- Serial bridge mode (uncomment SERIAL_BRIDGE_MODE below):
 *    Attach a second UART. Any Modbus RTU frame arriving on that UART is
 *    forwarded over radio automatically. The response is written back to
 *    the same UART. Your Modbus master software just uses a COM port.
 *
 * Route learning:
 *    Unknown Modbus addresses are sent as broadcast. When the first response
 *    arrives the master learns which radio slave serves that Modbus address.
 *    Subsequent requests are sent unicast to that slave.
 *    After 3 consecutive timeouts the route is evicted (broadcast again).
 *
 * Wiring (HC-12):
 *   ESP32 GPIO33 (TX1) -> HC-12 RX
 *   ESP32 GPIO32 (RX1) -> HC-12 TX
 *   ESP32 GPIO12       -> HC-12 SET
 *
 * Wiring (Serial bridge, Mode B):
 *   ESP32 GPIO17 (TX2) -> RS-485 / Modbus master device RX
 *   ESP32 GPIO16 (RX2) -> RS-485 / Modbus master device TX
 */

#include <HC12Transporter.h>
#include <radiobridges/ModbusMaster.h>

// Uncomment to enable serial-bridge mode (see Mode B above)
#define SERIAL_BRIDGE_MODE

// --- HC-12 pins ---
static constexpr uint8_t HC12_SET_PIN = 12;
static constexpr uint8_t HC12_RX_PIN = 32;
static constexpr uint8_t HC12_TX_PIN = 33;

static const uint8_t SHARED_KEY[16] = {0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30};  // "0000000000000000"

// --- Objects ---
HC12Driver radio;
RadioTransport transport;
ModbusMaster bridge;

// --- Modbus CRC16 (standard poly 0xA001) ---
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

// --- Build FC03 Read Holding Registers request ---
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

// --- Modbus addresses to poll (API mode) ---
static constexpr uint8_t POLL_ADDRS[] = {0x01, 0x02, 0x05};
static constexpr uint8_t POLL_COUNT = sizeof(POLL_ADDRS) / sizeof(POLL_ADDRS[0]);

void setup() {
    Serial.begin(115200);
    Serial.println(F("[MASTER] Modbus bridge example"));

    pinMode(35, INPUT);

    // --- HC-12 ---
    HC12Config hcCfg;
    hcCfg.channel = 5;
    hcCfg.power = 8;
    hcCfg.mode = HC12Mode::FU3;
    hcCfg.baud = 19200;

    if (!radio.begin(&Serial1, HC12_SET_PIN, HC12_RX_PIN, HC12_TX_PIN, hcCfg)) {
        Serial.println(F("[MASTER] HC-12 FAILED"));
    }

    // --- Transport ---
    TransportConfig tCfg = TRANSPORT_DEFAULT_CONFIG;
    tCfg.localAddr = RADIO_ADDR_MASTER;
    tCfg.retries = 3;
    tCfg.ackTimeoutMs = 75;
    tCfg.autoPowerEnabled = true;
    tCfg.autoPowerIntervalMs = 5000;
    memcpy(tCfg.encryptionKey, SHARED_KEY, 16);

    transport.begin(&radio, tCfg);
    transport.startTask();

    // --- Modbus bridge ---
    bridge.begin(&transport, 500 /* rxTimeoutMs */);
    bridge.setMaxFailures(3);

#ifdef SERIAL_BRIDGE_MODE
    // Mode B: attach Serial2 as the Modbus UART bridge.
    // Any Modbus master connected to Serial2 will talk through the radio.
    Serial2.begin(9600, SERIAL_8N1, 5, 17);
    bridge.attachSerial(&Serial2, 5 /* silenceMs */, 4 /* rstPin */);
    bridge.startTask();
    Serial.println(F("[MASTER] Serial bridge mode active on Serial2"));
#else
    // Mode A: API mode -- bridge.startTask() drives route learning and
    // the receive() queue in the background.
    bridge.startTask();
    Serial.println(F("[MASTER] API mode active"));
#endif
}

void loop() {
#ifdef SERIAL_BRIDGE_MODE
    // In serial bridge mode the bridge task handles everything.
    // loop() is free for other application logic.
    vTaskDelay(pdMS_TO_TICKS(1000));

#else
    // --- API mode: poll each Modbus address every 1 second ---
    static uint8_t pollIdx = 0;
    static uint32_t lastPollMs = 0;

    if (millis() - lastPollMs < 1000) return;
    lastPollMs = millis();

    uint8_t devAddr = POLL_ADDRS[pollIdx];
    pollIdx = (pollIdx + 1) % POLL_COUNT;

    // Build and send an FC03 request for 2 registers starting at 0x0000.
    uint8_t frame[8];
    uint8_t flen = buildFC03(devAddr, 0x0000, 2, frame);

    uint8_t knownRadio = bridge.lookupRoute(devAddr);
    Serial.printf("[MASTER] Polling Modbus 0x%02X (radio %s 0x%02X) ... ",
                  devAddr,
                  knownRadio ? ">" : "BC",
                  knownRadio ? knownRadio : 0xFF);

    // send() blocks until response or timeout.
    bool ok = bridge.send(frame, flen);

    if (ok) {
        uint8_t resp[RADIO_MAX_PAYLOAD];
        uint8_t rlen;
        uint8_t radioSrc;

        if (bridge.receive(resp, &rlen, &radioSrc)) {
            Serial.printf("OK  radio=0x%02X  %d bytes:", radioSrc, rlen);
            for (uint8_t i = 0; i < rlen; i++) Serial.printf(" %02X", resp[i]);
            Serial.println();
        }
    } else {
        Serial.println(F("TIMEOUT / NO ROUTE"));
    }
#endif
    if (digitalRead(35)) {
        ESP.restart();
    }
}
