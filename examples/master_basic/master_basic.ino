/**
 * master_basic.ino -- RadioTransport master example (no Modbus)
 *
 * Demonstrates the event-driven transport API:
 *  - HC12Driver configuration (channel, power, mode)
 *  - RadioTransport with auto-power enabled
 *  - transport.startTask() spawns the background FreeRTOS task
 *  - Sending data via send() -- blocks the calling task efficiently via semaphore
 *  - Receiving replies via onReceive() callback (fires from background task)
 *
 * Note: update() no longer exists. Never call it.
 *       Do NOT call send() from inside the onReceive callback -- use sendAsync().
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

// --- Radio addresses ---
static constexpr uint8_t SLAVE_ADDR = 0x10;

// --- Shared AES-128 key (must be identical on every node in the network) ---
// Replace with your own 16 random bytes. Keep this secret.
static const uint8_t SHARED_KEY[16] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE,
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};

// --- Objects ---
HC12Driver radio;
RadioTransport transport;

// --- Counter for demo payload ---
static uint32_t counter = 0;

// --- onReceive callback (fires from background task) ---
static void onPacket(uint8_t src, PacketType type,
                     const uint8_t* data, uint8_t len) {
    if (type == PacketType::DATA) {
        Serial.printf("[MASTER] Got DATA from 0x%02X (%d bytes):", src, len);
        for (uint8_t i = 0; i < len; i++) Serial.printf(" %02X", data[i]);
        Serial.println();
    } else if (type == PacketType::STATUS) {
        Serial.printf("[MASTER] STATUS from 0x%02X\n", src);
    }
    // Use sendAsync() here if a reply is needed -- never send().
}

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
    }
    Serial.printf("[MASTER] HC-12 OK  ch=%d  p=%d  FU%d  baud=%lu\n",
                  hcCfg.channel, hcCfg.power, (int)hcCfg.mode, hcCfg.baud);

    // --- Transport configuration ---
    TransportConfig tCfg = TRANSPORT_DEFAULT_CONFIG;
    tCfg.localAddr = RADIO_ADDR_MASTER;
    tCfg.retries = 3;
    tCfg.ackTimeoutMs = 75;
    tCfg.autoPowerEnabled = true;
    tCfg.autoPowerMinP = 2;
    tCfg.autoPowerMaxP = 8;
    tCfg.autoPowerIntervalMs = 5000;
    // AES-128-CTR encryption is always active. Set your pre-shared key here.
    // All nodes in the network must use the same key.
    memcpy(tCfg.encryptionKey, SHARED_KEY, 16);

    transport.begin(&radio, tCfg);

    // Register callback -- fires from the background task on every packet.
    transport.onReceive(onPacket);

    // Spawn the background task. Must be called after begin().
    if (!transport.startTask()) {
        Serial.println(F("[MASTER] startTask FAILED"));
        while (true) delay(1000);
    }
    Serial.println(F("[MASTER] Transport ready (event-driven)"));
}

void loop() {
    // --- Send a payload every 2 seconds ---
    static uint32_t lastSendMs = 0;
    if (millis() - lastSendMs >= 2000) {
        lastSendMs = millis();

        uint8_t payload[4];
        payload[0] = (uint8_t)(counter >> 24);
        payload[1] = (uint8_t)(counter >> 16);
        payload[2] = (uint8_t)(counter >> 8);
        payload[3] = (uint8_t)(counter);
        counter++;

        Serial.printf("[MASTER] Sending counter=%lu to 0x%02X ... ", counter - 1, SLAVE_ADDR);
        // send() blocks this task via semaphore until ACK arrives or retries exhaust.
        // The background task processes the ACK and releases the semaphore.
        bool ok = transport.send(SLAVE_ADDR, PacketType::DATA, payload, 4);
        Serial.println(ok ? F("ACK") : F("FAILED"));
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
