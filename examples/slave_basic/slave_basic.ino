/**
 * slave_basic.ino — RadioTransport slave example (no Modbus)
 *
 * Demonstrates:
 *  - HC12Driver configuration on slave side
 *  - Receiving arbitrary data from master and echoing it back
 *  - Responding to PING broadcasts with a PONG
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

// --- This slave's radio address (must be 0x10–0xFE, unique per network) ---
static constexpr uint8_t MY_RADIO_ADDR = 0x11;

// --- Objects ---
HC12Driver radio;
RadioTransport transport;

void setup() {
    Serial.begin(115200);
    Serial.printf("[SLAVE 0x%02X] Initialising...\n", MY_RADIO_ADDR);

    pinMode(35, INPUT);

    // --- HC-12 configuration (must match master channel + mode) ---
    HC12Config hcCfg;
    hcCfg.channel = 5;
    hcCfg.power = 8;
    hcCfg.mode = HC12Mode::FU3;
    hcCfg.baud = 19200;

    if (!radio.begin(&Serial1, HC12_SET_PIN, HC12_RX_PIN, HC12_TX_PIN, hcCfg)) {
        Serial.println(F("[SLAVE] HC-12 init FAILED"));
        while (true) delay(1000);
    }
    Serial.println(F("[SLAVE] HC-12 OK"));

    // --- Transport configuration ---
    TransportConfig tCfg = TRANSPORT_DEFAULT_CONFIG;
    tCfg.localAddr = MY_RADIO_ADDR;
    tCfg.retries = 3;
    tCfg.ackTimeoutMs = 75;
    tCfg.autoPowerEnabled = false;  // slaves typically don't auto-adjust

    transport.begin(&radio, tCfg);
    Serial.printf("[SLAVE 0x%02X] Transport ready\n", MY_RADIO_ADDR);
}

void loop() {
    transport.update();

    uint8_t src;
    PacketType type;
    uint8_t data[RADIO_MAX_PAYLOAD];
    uint8_t len;

    while (transport.receive(&src, &type, data, &len)) {
        switch (type) {
            case PacketType::DATA:
                // Echo the payload back to sender
                Serial.printf("[SLAVE] DATA from 0x%02X (%d bytes) -> echoing\n", src, len);
                transport.send(src, PacketType::DATA, data, len);
                break;

            case PacketType::PING:
                // Reply with a PONG (transport handles ACK; PING is delivered here)
                Serial.printf("[SLAVE] PING from 0x%02X -> PONG\n", src);
                {
                    // Payload: [my_radio_addr, 0 devices in basic example]
                    uint8_t pong[2] = {MY_RADIO_ADDR, 0};
                    transport.send(src, PacketType::PONG, pong, 2);
                }
                break;

            default:
                break;
        }
    }

    // Periodic status report to Serial
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
