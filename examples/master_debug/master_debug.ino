/**
 * master_debug.ino -- HC-12 Radio Network Diagnostic Tool
 *
 * Every 5 seconds this master:
 *  1. Sends a PKT_PING to each configured slave
 *  2. Measures full round-trip time (PING -> transport ACK -> PONG back)
 *  3. Tracks per-slave retries, success rate, and current TX power level
 *  4. Prints a formatted diagnostic table to the Serial Monitor
 *
 * Event-driven model:
 *  - transport.startTask() spawns the FreeRTOS background task in setup()
 *  - update() no longer exists and must not be called
 *  - probeSlave() sends the PING (blocking via semaphore), then waits for
 *    the PONG via a short vTaskDelay + receive() drain instead of spinning
 *
 * Slaves must run slave_basic.ino -- they reply with PKT_PONG on every PKT_PING.
 *
 * HC-12 does NOT expose an RSSI register, so link quality is estimated from
 * the retry-count heuristic used by the auto-power controller:
 *   0 retries  -> 100%  (excellent)
 *   1 retry    ->  75%  (good)
 *   2 retries  ->  50%  (fair)
 *   3 retries  ->  25%  (poor)
 *   timeout    ->   0%  (offline)
 *
 * TX power levels (AT+P1...AT+P8) map to approximate dBm:
 *   P1 = -1 dBm  |  P2 = 2   |  P3 = 5   |  P4 = 8
 *   P5 = 11 dBm  |  P6 = 14  |  P7 = 17  |  P8 = 20 dBm
 *
 * Wiring:
 *   ESP32 GPIO33 (TX1) -> HC-12 RX
 *   ESP32 GPIO32 (RX1) -> HC-12 TX
 *   ESP32 GPIO12       -> HC-12 SET
 *
 * ANSI colour codes are emitted by default. In Arduino IDE 2.x Serial Monitor
 * set "Carriage Return + Line Feed" and the colours will render.
 * Define NO_ANSI_COLOR to disable colour output.
 */

#include <HC12Transporter.h>

// --- User configuration ---

/** Radio addresses of the slaves to monitor. Add/remove as needed. */
static const uint8_t SLAVE_ADDRS[] = {0x10, 0x11};
static constexpr uint8_t SLAVE_COUNT = sizeof(SLAVE_ADDRS) / sizeof(SLAVE_ADDRS[0]);
static const uint8_t SHARED_KEY[16] = {0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30}; // "0000000000000000"

// HC-12 hardware pins
static constexpr uint8_t HC12_SET_PIN = 12;
static constexpr uint8_t HC12_RX_PIN = 32;
static constexpr uint8_t HC12_TX_PIN = 33;

// Timing
static constexpr uint32_t REPORT_INTERVAL_MS = 5000;  ///< Diagnostic print interval
static constexpr uint32_t PONG_WAIT_MS = 350;         ///< Max wait for PONG after PING ACK

// Max retries -- must match slave TransportConfig
static constexpr uint8_t MAX_RETRIES = 3;

// --- ANSI colour helpers ---

#ifndef NO_ANSI_COLOR
#define ANSI_RESET "\033[0m"
#define ANSI_BOLD "\033[1m"
#define ANSI_GREEN "\033[32m"
#define ANSI_YELLOW "\033[33m"
#define ANSI_RED "\033[31m"
#define ANSI_CYAN "\033[36m"
#define ANSI_WHITE "\033[37m"
#define ANSI_DIM "\033[2m"
#else
#define ANSI_RESET ""
#define ANSI_BOLD ""
#define ANSI_GREEN ""
#define ANSI_YELLOW ""
#define ANSI_RED ""
#define ANSI_CYAN ""
#define ANSI_WHITE ""
#define ANSI_DIM ""
#endif

// --- Power level lookup table ---

/** HC-12 P1-P8 in dBm (index 0 unused). */
static const int8_t POWER_DBM[9] = {0, -1, 2, 5, 8, 11, 14, 17, 20};

/** Bar representation of power P1-P8 (8 chars wide). */
static const char* POWER_BAR[9] = {
    "",
    "[=       ]",  // P1
    "[==      ]",  // P2
    "[===     ]",  // P3
    "[====    ]",  // P4
    "[=====   ]",  // P5
    "[======  ]",  // P6
    "[======= ]",  // P7
    "[========]",  // P8
};

// --- Per-slave diagnostic record ---

struct SlaveRecord {
    uint8_t addr;

    // Connectivity
    bool online;    ///< true if last PING got a PONG back
    bool everSeen;  ///< true once the slave has responded at least once

    // RTT (full round-trip: PING sent -> PONG received)
    uint32_t rttLastMs;
    uint32_t rttMinMs;
    uint32_t rttMaxMs;
    uint32_t rttSumMs;  ///< Accumulated for rolling average (resets at REPORT_INTERVAL)

    // Counters
    uint32_t pingsSent;
    uint32_t pongsReceived;
    uint32_t retriesThisCycle;  ///< Retries used in the most recent PING exchange
    uint32_t retriesTotal;      ///< All-time retries to this slave

    // Link quality
    uint8_t linkQuality;  ///< 0-100 estimate
    uint8_t powerLevel;   ///< Current auto-power level for this slave (1-8)

    // Interval snapshot for delta stats
    uint32_t prevPingsSent;
    uint32_t prevPongsReceived;
    uint32_t rttSumInterval;    ///< RTT sum since last report
    uint32_t rttCountInterval;  ///< PONG count since last report
};

static SlaveRecord records[SLAVE_COUNT];

// --- Global transport objects ---

static HC12Driver radio;
static RadioTransport transport;

// --- Snapshot of global stats at last report (for delta display) ---

static LinkStats prevStats = {};

// --- Helpers ---

static void printUptime() {
    uint32_t totalSec = millis() / 1000;
    uint32_t h = totalSec / 3600;
    uint32_t m = (totalSec % 3600) / 60;
    uint32_t s = totalSec % 60;
    Serial.printf("%02lu:%02lu:%02lu", (unsigned long)h,
                  (unsigned long)m, (unsigned long)s);
}

/** Estimate link quality 0-100 from retry count. */
static uint8_t estimateQuality(uint8_t retries, bool ok) {
    if (!ok) return 0;
    if (retries == 0) return 100;
    int q = 100 - (int)retries * 75 / MAX_RETRIES;
    return (q < 0) ? 0 : (uint8_t)q;
}

/** Colour-code a quality percentage. */
static const char* qualityColor(uint8_t q) {
    if (q >= 90) return ANSI_GREEN;
    if (q >= 60) return ANSI_YELLOW;
    return ANSI_RED;
}

/** Colour-code an RTT value (ms). */
static const char* rttColor(uint32_t rtt) {
    if (rtt == 0) return ANSI_DIM;
    if (rtt < 80) return ANSI_GREEN;
    if (rtt < 150) return ANSI_YELLOW;
    return ANSI_RED;
}

// --- Probe one slave (send PING, wait for PONG, measure RTT) ---

static void probeSlave(SlaveRecord& r) {
    r.pingsSent++;

    // Snapshot global retry counter to compute per-ping retries
    uint32_t retriesBefore = transport.stats().retries;

    // --- TX PING (blocking: send() waits via semaphore for transport-level ACK) ---
    uint32_t t0 = millis();
    bool ackOk = transport.send(r.addr, PacketType::PING, nullptr, 0);

    r.retriesThisCycle = transport.stats().retries - retriesBefore;
    r.retriesTotal += r.retriesThisCycle;
    r.powerLevel = transport.slavePower(r.addr);

    if (!ackOk) {
        r.online = false;
        r.linkQuality = 0;
        r.rttLastMs = 0;
        return;
    }

    // --- Wait for application-level PONG from the slave ---
    // Poll the RX queue in 1 ms slices so we break out the instant the PONG
    // arrives, giving an accurate RTT reading. vTaskDelay(1) yields the CPU
    // to the background task on every iteration -- no busy-spin.
    bool pongReceived = false;
    {
        uint32_t deadline = millis() + PONG_WAIT_MS;
        uint8_t src;
        PacketType type;
        uint8_t data[RADIO_MAX_PAYLOAD];
        uint8_t len;

        while (millis() < deadline && !pongReceived) {
            while (transport.receive(&src, &type, data, &len)) {
                if (src == r.addr && type == PacketType::PONG) {
                    pongReceived = true;
                    break;
                }
                // Discard unexpected packets that arrived during the wait window
            }
            if (!pongReceived) vTaskDelay(pdMS_TO_TICKS(1));  // yield, check again next ms
        }
    }

    uint32_t rtt = millis() - t0;

    r.online = pongReceived;
    r.rttLastMs = pongReceived ? rtt : 0;
    r.linkQuality = estimateQuality((uint8_t)r.retriesThisCycle, pongReceived);

    if (pongReceived) {
        r.everSeen = true;
        r.pongsReceived++;

        if (r.rttMinMs == 0 || rtt < r.rttMinMs) r.rttMinMs = rtt;
        if (rtt > r.rttMaxMs) r.rttMaxMs = rtt;
        r.rttSumMs += rtt;

        r.rttSumInterval += rtt;
        r.rttCountInterval++;
    }
}

// --- Print diagnostic report ---

static void printReport() {
    const LinkStats& st = transport.stats();
    const HC12Config& hcfg = radio.getConfig();

    uint32_t dTx = st.txPackets - prevStats.txPackets;
    uint32_t dRx = st.rxPackets - prevStats.rxPackets;
    uint32_t dCrc = st.crcErrors - prevStats.crcErrors;
    uint32_t dRetries = st.retries - prevStats.retries;
    uint32_t dTimeouts = st.timeouts - prevStats.timeouts;
    uint32_t dFailed = st.txFailed - prevStats.txFailed;
    prevStats = st;
    (void)dFailed;

    Serial.println();
    Serial.print(ANSI_BOLD ANSI_CYAN);
    Serial.print(F("HC-12 Diagnostics  "));
    printUptime();
    Serial.println(ANSI_RESET);

    Serial.print(ANSI_DIM);
    Serial.printf("  CH=%03d FU%d %lu baud  Global: P%d (%+d dBm)  Slaves=%d\r\n",
                  hcfg.channel, (int)hcfg.mode, (unsigned long)hcfg.baud,
                  hcfg.power, POWER_DBM[hcfg.power], SLAVE_COUNT);
    Serial.print(ANSI_RESET);

    for (uint8_t i = 0; i < SLAVE_COUNT; i++) {
        SlaveRecord& r = records[i];

        uint32_t avgRttInterval = (r.rttCountInterval > 0)
                                      ? r.rttSumInterval / r.rttCountInterval
                                      : 0;

        uint8_t p = (r.powerLevel >= 1 && r.powerLevel <= 8)
                        ? r.powerLevel
                        : hcfg.power;

        const char* statusColor = r.online ? ANSI_GREEN : r.everSeen ? ANSI_YELLOW
                                                                     : ANSI_RED;
        const char* statusStr = r.online ? "ONLINE" : r.everSeen ? "LOST"
                                                                 : "OFFLINE";

        Serial.printf("  0x%02X  %s%s" ANSI_RESET "  ", r.addr, statusColor, statusStr);

        const char* qColor = qualityColor(r.linkQuality);
        Serial.printf("q=%s%3d%%" ANSI_RESET "  ", qColor, r.linkQuality);

        if (r.online && r.rttLastMs > 0) {
            Serial.printf("rtt=%s%lums" ANSI_RESET " (avg %s%lums" ANSI_RESET ")  ",
                          rttColor(r.rttLastMs), (unsigned long)r.rttLastMs,
                          rttColor(avgRttInterval), (unsigned long)avgRttInterval);
        } else {
            Serial.print(ANSI_DIM "rtt=---  " ANSI_RESET);
        }

        Serial.printf("retries=%s%lu" ANSI_RESET "/%lu  ",
                      r.retriesThisCycle > 0 ? ANSI_YELLOW : ANSI_GREEN,
                      (unsigned long)r.retriesThisCycle,
                      (unsigned long)MAX_RETRIES);

        Serial.printf("P%d (%+d dBm) %s%s" ANSI_RESET "\r\n",
                      p, POWER_DBM[p], qColor, POWER_BAR[p]);
    }

    Serial.print(ANSI_DIM);
    Serial.printf("  TX +%lu/%lu  RX +%lu/%lu  CRC +%lu/%lu  Retries +%lu/%lu  Timeouts +%lu/%lu\r\n",
                  (unsigned long)dTx, (unsigned long)st.txPackets,
                  (unsigned long)dRx, (unsigned long)st.rxPackets,
                  (unsigned long)dCrc, (unsigned long)st.crcErrors,
                  (unsigned long)dRetries, (unsigned long)st.retries,
                  (unsigned long)dTimeouts, (unsigned long)st.timeouts);
    Serial.print(ANSI_RESET);

    for (uint8_t i = 0; i < SLAVE_COUNT; i++) {
        records[i].rttSumInterval = 0;
        records[i].rttCountInterval = 0;
    }
}

// --- setup() ---

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(35, INPUT);

    Serial.println(ANSI_BOLD ANSI_CYAN
                   "\n  HC-12 Radio Network Diagnostic -- master_debug.ino" ANSI_RESET);
    Serial.println(ANSI_DIM
                   "  Slaves must run slave_basic.ino on the same channel/mode" ANSI_RESET "\n");

    for (uint8_t i = 0; i < SLAVE_COUNT; i++) {
        records[i] = {};
        records[i].addr = SLAVE_ADDRS[i];
    }

    // --- HC-12 configuration ---
    HC12Config hcCfg;
    hcCfg.channel = 5;
    hcCfg.power = 8;
    hcCfg.mode = HC12Mode::FU3;
    hcCfg.baud = 19200;

    Serial.printf("  Initialising HC-12  CH=%03d  FU%d  %lu baud  P%d...\r\n",
                  hcCfg.channel, (int)hcCfg.mode,
                  (unsigned long)hcCfg.baud, hcCfg.power);

    if (!radio.begin(&Serial1, HC12_SET_PIN, HC12_RX_PIN, HC12_TX_PIN, hcCfg)) {
        Serial.println(ANSI_RED
                       "  HC-12 init FAILED -- check wiring and SET pin" ANSI_RESET);
    }
    Serial.println(ANSI_GREEN "  HC-12 OK" ANSI_RESET);

    // --- Transport configuration ---
    TransportConfig tCfg = TRANSPORT_DEFAULT_CONFIG;
    tCfg.localAddr = RADIO_ADDR_MASTER;
    tCfg.retries = MAX_RETRIES;
    tCfg.ackTimeoutMs = 75;
    tCfg.autoPowerEnabled = true;
    memcpy(tCfg.encryptionKey, SHARED_KEY, 16);
    tCfg.autoPowerMinP = 1;
    tCfg.autoPowerMaxP = 8;
    tCfg.autoPowerIntervalMs = REPORT_INTERVAL_MS;
    tCfg.autoPowerHighThresh = 1;
    tCfg.autoPowerCleanSteps = 4;

    transport.begin(&radio, tCfg);

    // Spawn the background task. No update() call is needed anywhere after this.
    if (!transport.startTask()) {
        Serial.println(ANSI_RED "  startTask FAILED" ANSI_RESET);
        while (true) delay(1000);
    }
    Serial.println(ANSI_GREEN "  Transport OK" ANSI_RESET);

    Serial.printf("  Monitoring %d slave(s): ", SLAVE_COUNT);
    for (uint8_t i = 0; i < SLAVE_COUNT; i++) {
        Serial.printf("0x%02X ", SLAVE_ADDRS[i]);
    }
    Serial.printf("\r\n  First report in %.1f s...\r\n\r\n",
                  REPORT_INTERVAL_MS / 1000.0f);
}

// --- loop() ---

void loop() {
    static uint32_t lastReportMs = 0;

    if (millis() - lastReportMs >= REPORT_INTERVAL_MS) {
        lastReportMs = millis();

        for (uint8_t i = 0; i < SLAVE_COUNT; i++) {
            probeSlave(records[i]);
            // Brief pause between probes to avoid collisions on the RF medium
            vTaskDelay(pdMS_TO_TICKS(30));
        }

        printReport();
    }

    if (digitalRead(35)) {
        ESP.restart();
    }
}
