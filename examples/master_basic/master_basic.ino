/**
 * master_basic.ino — RadioTransport master example (no Modbus)
 *
 * Demonstrates:
 *  - HC12Driver configuration (channel, power, mode)
 *  - RadioTransport with auto-power enabled
 *  - Sending arbitrary data to a slave and receiving replies
 *
 * Wiring:
 *   ESP32 GPIO17 (TX1) -> HC-12 RX
 *   ESP32 GPIO16 (RX1) -> HC-12 TX
 *   ESP32 GPIO4        -> HC-12 SET
 */

#include <HC12Transporter.h>

// --- Pin assignments ---
static constexpr uint8_t HC12_SET_PIN = 12;
static constexpr uint8_t HC12_RX_PIN = 32;
static constexpr uint8_t HC12_TX_PIN = 33;

// --- Radio addresses ---
static constexpr uint8_t SLAVE_ADDR = 0x10;

// --- Objects ---
HC12Driver radio;
RadioTransport transport;

// --- Counter for demo payload ---
static uint32_t counter = 0;

void setup() {
    Serial.begin(115200);
    Serial.println(F("[MASTER] Initialising..."));

    pinMode(35, INPUT);

    // --- HC-12 configuration ---
    HC12Config hcCfg;
    hcCfg.channel = 5;  // RF channel 1-127
    hcCfg.power = 8;    // TX power 1-8 (8 = max, 20 dBm)
    hcCfg.mode = HC12Mode::FU3;
    hcCfg.baud = 19200;

    if (!radio.begin(&Serial1, HC12_SET_PIN, HC12_RX_PIN, HC12_TX_PIN, hcCfg)) {
        Serial.println(F("[MASTER] HC-12 init FAILED"));
        while (true) delay(1000);
    }
    Serial.printf("[MASTER] HC-12 OK  ch=%d  p=%d  FU%d  baud=%lu\n",
                  hcCfg.channel, hcCfg.power, (int)hcCfg.mode, hcCfg.baud);

    // --- Transport configuration ---
    TransportConfig tCfg = TRANSPORT_DEFAULT_CONFIG;
    tCfg.localAddr = RADIO_ADDR_MASTER;
    tCfg.retries = 3;
    tCfg.ackTimeoutMs = 75;
    tCfg.autoPowerEnabled = true;  // adaptive TX power
    tCfg.autoPowerMinP = 2;
    tCfg.autoPowerMaxP = 8;
    tCfg.autoPowerIntervalMs = 5000;

    transport.begin(&radio, tCfg);
    Serial.println(F("[MASTER] Transport ready"));
}

void loop() {
    // --- Send a payload every 2 seconds ---
    static uint32_t lastSendMs = 0;
    if (millis() - lastSendMs >= 2000) {
        lastSendMs = millis();

        uint8_t payload[8];
        payload[0] = (uint8_t)(counter >> 24);
        payload[1] = (uint8_t)(counter >> 16);
        payload[2] = (uint8_t)(counter >> 8);
        payload[3] = (uint8_t)(counter);
        counter++;

        Serial.printf("[MASTER] Sending counter=%lu to slave 0x%02X ... ", counter - 1, SLAVE_ADDR);
        bool ok = transport.send(SLAVE_ADDR, PacketType::DATA, payload, 4);
        Serial.println(ok ? F("ACK") : F("FAILED"));
    }

    // --- Check for replies ---
    transport.update();

    uint8_t src;
    PacketType type;
    uint8_t data[RADIO_MAX_PAYLOAD];
    uint8_t len;

    while (transport.receive(&src, &type, data, &len)) {
        if (type == PacketType::DATA) {
            Serial.printf("[MASTER] Got DATA from 0x%02X: ", src);
            for (uint8_t i = 0; i < len; i++) Serial.printf("%02X ", data[i]);
            Serial.println();
        } else if (type == PacketType::STATUS) {
            Serial.printf("[MASTER] STATUS from 0x%02X\n", src);
        }
    }

    // --- Print stats every 30 s ---
    static uint32_t lastStatsMs = 0;
    if (millis() - lastStatsMs >= 30000) {
        lastStatsMs = millis();
        const LinkStats& st = transport.stats();
        Serial.printf("[STATS] tx=%lu rx=%lu crcErr=%lu retries=%lu timeouts=%lu dup=%lu fail=%lu\n",
                      st.txPackets, st.rxPackets, st.crcErrors,
                      st.retries, st.timeouts, st.duplicates, st.txFailed);
        Serial.printf("[STATS] slave 0x%02X power level: %d\n",
                      SLAVE_ADDR, transport.slavePower(SLAVE_ADDR));
    }

    if (digitalRead(35)) {
        ESP.restart();
    }
}
