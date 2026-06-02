/**
 * slave_basic.ino -- RadioTransport slave example (no Modbus)
 *
 * Demonstrates the event-driven transport API on the slave side:
 *  - HC12Driver configuration (must match master channel + mode)
 *  - transport.startTask() spawns the background FreeRTOS task
 *  - All packet handling done in onReceive() callback (fires from background task)
 *  - sendAsync() used inside the callback to reply -- safe, non-blocking
 *
 * Note: update() no longer exists. Never call it.
 *       Do NOT call send() from inside the onReceive callback -- use sendAsync().
 *       send() from inside a callback will deadlock.
 *
 * Wiring:
 *   ESP32 GPIO33 (TX1) -> HC-12 RX
 *   ESP32 GPIO32 (RX1) -> HC-12 TX
 *   ESP32 GPIO12       -> HC-12 SET
 */

#include <HC12Transporter.h>

// --- Pin assignments ---
static constexpr uint8_t HC12_SET_PIN = 12;
static constexpr uint8_t HC12_RX_PIN = 32;
static constexpr uint8_t HC12_TX_PIN = 33;

// --- This slave's radio address (must be 0x10-0xFE, unique per network) ---
static constexpr uint8_t MY_RADIO_ADDR = 0x10;

// --- Objects ---
HC12Driver radio;
RadioTransport transport;

// --- onReceive callback (fires from background task) ---
// sendAsync() is safe here. send() is NOT safe here -- it will deadlock.
static void onPacket(uint8_t src, PacketType type,
                     const uint8_t* data, uint8_t len) {
    switch (type) {
        case PacketType::DATA:
            // Echo the payload back to sender via sendAsync() -- non-blocking.
            Serial.printf("[SLAVE] DATA from 0x%02X (%d bytes) -> echoing\n", src, len);
            transport.sendAsync(src, PacketType::DATA, data, len);
            break;

        case PacketType::PING:
            // Reply with a PONG carrying this node's radio address.
            Serial.printf("[SLAVE] PING from 0x%02X -> PONG\n", src);
            {
                // Payload: [my_radio_addr, 0 local Modbus devices in basic example]
                uint8_t pong[2] = {MY_RADIO_ADDR, 0};
                transport.sendAsync(src, PacketType::PONG, pong, 2);
            }
            break;

        default:
            break;
    }
}

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
    tCfg.autoPowerEnabled = false;  // Enable autoPower here to also optimise the slave->master uplink

    transport.begin(&radio, tCfg);

    // Register callback -- fires from the background task on every packet.
    transport.onReceive(onPacket);

    // Spawn the background task. Must be called after begin().
    if (!transport.startTask()) {
        Serial.println(F("[SLAVE] startTask FAILED"));
        while (true) delay(1000);
    }
    Serial.printf("[SLAVE 0x%02X] Transport ready (event-driven)\n", MY_RADIO_ADDR);
}

void loop() {
    // Packet handling is fully event-driven via the onReceive() callback.
    // loop() only handles periodic diagnostics and the restart button.

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
