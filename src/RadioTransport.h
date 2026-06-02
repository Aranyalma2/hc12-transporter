#pragma once

/**
 * @file RadioTransport.h
 * @brief Reliable, addressed, application-agnostic radio transport layer.
 *
 * Sits above HC12Driver and below any application protocol (Modbus, custom, ...).
 * The application supplies raw bytes in send(); the transport frames, CRCs,
 * sequences, ACKs, retries, and delivers clean payloads via receive().
 *
 * --- TX flow ---
 *  IDLE -> SEND -> WAIT_ACK - (ACK) -> IDLE (success)
 *                         -> (timeout) RETRY -> SEND (if retries left)
 *                                         -> FAILED -> IDLE
 *  Broadcast: IDLE -> SEND -> IDLE  (no ACK expected)
 *
 * --- RX flow ---
 *  Byte-by-byte state machine:
 *  WAIT_SOF -> RD_DEST -> RD_SRC -> RD_SEQ -> RD_TYPE -> RD_LEN
 *           -> RD_PAYLOAD(N) -> RD_CRC1 -> RD_CRC2 -> VALIDATE
 *
 * --- Duplicate detection ---
 *  uint8_t _lastSeq[256] indexed by sender address.
 *  If incoming seq == _lastSeq[src] -> discard payload, re-send ACK.
 *
 * --- Auto-power control ---
 *  Retry-count heuristic per slave:
 *   • too many retries -> increase power (AT+Px)
 *   • zero retries for N consecutive intervals -> decrease power
 *
 * --- FreeRTOS ---
 *  Define HC12_USE_RTOS before including this header to enable task spawning.
 *  Without it, call update() from your own loop — fully polling-safe.
 */

#include <Arduino.h>

#include "HC12Driver.h"
#include "RadioPacket.h"

// --- Types ---

/**
 * @brief Callback signature for asynchronous packet reception.
 * If registered, the transport layer will call this directly from update()
 * instead of queuing the packet.
 */
typedef void (*RadioRxCallback)(uint8_t src, PacketType type,
                                const uint8_t* data, uint8_t len);

// --- Compile-time limits ---

#ifndef HC12_MAX_SLAVES
#define HC12_MAX_SLAVES 32  /// Max slave nodes tracked for auto-power
#endif

#ifndef HC12_RX_QUEUE_DEPTH
#define HC12_RX_QUEUE_DEPTH 4  /// Number of received packets buffered
#endif

// --- Configuration ---

/**
 * @brief Per-transport configuration.
 * Pass to RadioTransport::begin().
 */
struct TransportConfig {
    uint8_t localAddr;           /// This node's radio address (RADIO_ADDR_MASTER or 0x10–0xFE)
    uint8_t retries;             /// TX retry count per packet (default 3)
    uint16_t ackTimeoutMs;       /// ms to wait for ACK before retry (default 75)
    uint16_t txRxSwitchDelayMs;  /// ms after TX before listening for ACK (default 10)
    uint16_t interPacketGapMs;   /// Minimum ms between consecutive TX (default 10)

    // Auto-power control
    bool autoPowerEnabled;         /// Enable adaptive TX power per slave
    uint8_t autoPowerMinP;         /// Minimum power level (1–8, default 1)
    uint8_t autoPowerMaxP;         /// Maximum power level (1–8, default 8)
    uint16_t autoPowerIntervalMs;  /// ms between power-adjustment evaluations (default 5000)
    uint8_t autoPowerHighThresh;   /// Retries/interval triggering a power increase (default 2)
    uint8_t autoPowerCleanSteps;   /// Clean intervals before power decrease (default 3)
};

/** @brief Sensible defaults. */
static constexpr TransportConfig TRANSPORT_DEFAULT_CONFIG = {
    .localAddr = RADIO_ADDR_MASTER,
    .retries = 3,
    .ackTimeoutMs = 75,
    .txRxSwitchDelayMs = 10,
    .interPacketGapMs = 10,
    .autoPowerEnabled = false,
    .autoPowerMinP = 1,
    .autoPowerMaxP = 8,
    .autoPowerIntervalMs = 5000,
    .autoPowerHighThresh = 2,
    .autoPowerCleanSteps = 12,
};

// --- Link Statistics ---

struct LinkStats {
    uint32_t txPackets;   /// Packets successfully delivered (ACK received)
    uint32_t rxPackets;   /// Valid packets received (after CRC + address filter)
    uint32_t crcErrors;   /// Frames discarded due to CRC mismatch
    uint32_t retries;     /// Total retry transmissions
    uint32_t timeouts;    /// ACK wait timeouts (may trigger retry or fail)
    uint32_t duplicates;  /// Incoming duplicates rejected (ACK re-sent)
    uint32_t acksSent;    /// ACK/NACK frames sent
    uint32_t txFailed;    /// Sends that exhausted all retries
    uint32_t broadcasts;  /// Broadcast frames transmitted (no ACK expected)
};

// --- RadioTransport class ---

class RadioTransport {
   public:
    // --- Lifecycle ---

    /**
     * @brief Initialise the transport.
     * @param driver  Initialised HC12Driver instance.
     * @param cfg     Transport configuration.
     * @return true always (driver validity is the caller's responsibility).
     */
    bool begin(HC12Driver* driver,
               const TransportConfig& cfg = TRANSPORT_DEFAULT_CONFIG);

    // --- TX API ---

    /**
     * @brief Send a payload to a destination radio address.
     *
     * For unicast (dest ≠ 0xFF): waits for ACK (blocking). Returns true only
     * if the remote transport layer acknowledged the packet.
     * For broadcast (dest == 0xFF): returns true immediately after TX.
     *
     * Internally calls update() in a spin loop while waiting for ACK —
     * safe to call from a single task / main loop.
     *
     * @param dest   Destination radio address.
     * @param type   Packet type (use PacketType::DATA for application data).
     * @param data   Payload bytes (max RADIO_MAX_PAYLOAD = 64).
     * @param len    Payload length.
     * @return true on success; false if delivery failed after all retries.
     */
    bool send(uint8_t dest, PacketType type,
              const uint8_t* data, uint8_t len);

    /**
     * @brief Non-blocking send — kicks off TX, returns immediately.
     * Call isBusy() / poll update() to track completion.
     * Prefer send() for simple use-cases.
     */
    bool sendAsync(uint8_t dest, PacketType type,
                   const uint8_t* data, uint8_t len);

    /** @brief Returns true while an async TX is in progress. */
    bool isBusy() const;

    /** @brief Result of the last completed async send (valid when !isBusy()). */
    bool lastSendOk() const { return _txLastResult; }

    // --- RX API ---

    /**
     * @brief Set a callback for asynchronous packet reception.
     * If a callback is set, it will be automatically invoked by update() when a
     * packet arrives. The standard receive queue (available() / receive()) will
     * be bypassed to prevent mixing paradigms.
     * @param cb Callback function pointer, or nullptr to disable.
     */
    void onReceive(RadioRxCallback cb);

    /**
     * @brief Returns true if at least one received packet is queued.
     * Note: always returns false if an onReceive() callback is registered.
     */
    bool available() const;

    /**
     * @brief Pop the oldest received packet.
     *
     * @param[out] src   Radio address of the sender.
     * @param[out] type  Packet type.
     * @param[out] data  Payload bytes (caller must provide ≥ RADIO_MAX_PAYLOAD bytes).
     * @param[out] len   Payload length.
     * @return true if a packet was available and copied.
     */
    bool receive(uint8_t* src, PacketType* type,
                 uint8_t* data, uint8_t* len);

    // --- Engine ---

    /**
     * @brief Drive the TX and RX state machines.
     * Must be called frequently (every few ms).
     * In polling mode, call from loop(). In RTOS mode, call from a task.
     */
    void update();

    // --- Auto-power ---

    /**
     * @brief Evaluate and optionally adjust TX power for one slave.
     * Called automatically by update() when autoPowerEnabled = true.
     * Can also be called manually at any time.
     */
    void runAutoPower(uint8_t slaveAddr);

    // --- Diagnostics ---

    const LinkStats& stats() const { return _stats; }
    void resetStats() { memset(&_stats, 0, sizeof(_stats)); }

    /** @brief Current TX power level (1–8) used for the given slave. */
    uint8_t slavePower(uint8_t slaveAddr) const;

   private:
    // --- TX state machine ---

    enum class TxState : uint8_t {
        IDLE,
        SENDING,
        WAIT_ACK,
        SUCCESS,
        RETRY,
        FAILED,
    };

    void _updateTx();
    void _doSend(const RadioPacket& pkt);

    // --- RX state machine ---

    enum class RxState : uint8_t {
        WAIT_SOF,
        RD_DEST,
        RD_SRC,
        RD_SEQ,
        RD_TYPE,
        RD_LEN,
        RD_PAYLOAD,
        RD_CRC1,
        RD_CRC2,
    };

    void _updateRx();
    void _processComplete();
    void _sendAck(uint8_t dest, uint8_t seq, bool ack);

    // --- TX state ---
    TxState _txState = TxState::IDLE;
    RadioPacket _txPacket;
    uint8_t _txRetriesLeft = 0;
    uint32_t _txSentMs = 0;
    uint32_t _txLastTxMs = 0;  /// millis() when last TX ended (inter-packet gap)
    bool _txBroadcast = false;
    bool _txLastResult = false;
    uint8_t _txSeq = 0;  /// Rolling sequence counter

    // --- RX state ---
    RxState _rxState = RxState::WAIT_SOF;
    RadioPacket _rxPacket;
    uint8_t _rxPayloadIdx = 0;
    uint8_t _rxCrc1 = 0;

    // --- RX receive queue (static) ---
    RadioPacket _rxQueue[HC12_RX_QUEUE_DEPTH];
    uint8_t _rxQHead = 0;
    uint8_t _rxQTail = 0;
    RadioRxCallback _rxCallback = nullptr;

    /** Push a completed packet into the queue. Returns false if full. */
    bool _rxEnqueue(const RadioPacket& pkt);

    // --- Duplicate detection ---
    uint8_t _lastSeq[256];  /// Last seen SEQ per source address
    bool _seqSeen[256];     /// Whether we've received anything from this src

    // --- Per-slave auto-power state ---
    struct SlaveState {
        uint8_t addr;
        uint8_t currentPower;
        uint32_t retryAccum;         /// Retries accumulated since last check
        uint8_t cleanIntervalCount;  /// Consecutive clean (zero-retry) intervals
        uint32_t lastCheckMs;
        bool used;
    };
    SlaveState _slaves[HC12_MAX_SLAVES];

    SlaveState* _findOrAllocSlave(uint8_t addr);

    // --- Dependencies and config ---
    HC12Driver* _driver = nullptr;
    TransportConfig _cfg = TRANSPORT_DEFAULT_CONFIG;
    LinkStats _stats = {};
};
