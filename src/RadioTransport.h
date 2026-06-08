#pragma once

/**
 * @file RadioTransport.h
 * @brief Reliable, addressed, application-agnostic radio transport layer.
 *
 * Sits above HC12Driver and below any application protocol (Modbus, custom, ...).
 * The application supplies raw bytes in send(); the transport frames, CRCs,
 * sequences, ACKs, retries, and delivers clean payloads via onReceive() callback
 * or the receive() queue.
 *
 * --- TX flow ---
 *  IDLE -> SENDING -> WAIT_ACK - (ACK) -> IDLE (success, unblocks send())
 *                             -> (timeout) -> SENDING (if retries left)
 *                                          -> IDLE (failed, unblocks send())
 *  Broadcast: IDLE -> SENDING -> IDLE  (no ACK expected)
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
 *   too many retries -> increase power (AT+Px)  [applied deferred, TX idle only]
 *   zero retries for N consecutive intervals -> decrease power
 *
 * --- Background task ---
 *  Call startTask() once after begin(). The library spawns a FreeRTOS task
 *  that wakes on UART ISR events and drives both state machines. Never call
 *  update() -- it no longer exists.
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "HC12Driver.h"
#include "RadioCipher.h"
#include "RadioPacket.h"

// --- Types ---

/**
 * @brief Callback signature for asynchronous packet reception.
 *
 * Invoked from the background task when a valid, addressed packet arrives.
 * It is safe to call sendAsync() from inside this callback.
 *
 * @warning Do NOT call send() from inside the callback. send() blocks waiting
 *          for an ACK that the background task would process -- calling it from
 *          the callback will deadlock. Use sendAsync() instead.
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
    uint8_t localAddr;           /// This node's radio address (RADIO_ADDR_MASTER or 0x10-0xFE)
    uint8_t retries;             /// TX retry count per packet (default 3)
    uint16_t ackTimeoutMs;       /// ms to wait for ACK before retry (default 75)
    uint16_t txRxSwitchDelayMs;  /// ms after TX before listening for ACK (default 10)
    uint16_t interPacketGapMs;   /// Minimum ms between consecutive TX (default 10)

    // Auto-power control
    bool autoPowerEnabled;         /// Enable adaptive TX power per slave
    uint8_t autoPowerMinP;         /// Minimum power level (1-8, default 1)
    uint8_t autoPowerMaxP;         /// Maximum power level (1-8, default 8)
    uint16_t autoPowerIntervalMs;  /// ms between power-adjustment evaluations (default 5000)
    uint8_t autoPowerHighThresh;   /// Retries/interval triggering a power increase (default 2)
    uint8_t autoPowerCleanSteps;   /// Clean intervals before power decrease (default 3)

    // Encryption (AES-128-CTR, payload only - always active)
    uint8_t encryptionKey[16];  /// Pre-shared 128-bit key. Set before begin(); immutable after.
                                /// Defaults to all-zero bytes (functional but not secret).
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
    .encryptionKey = {},
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

    /**
     * @brief Spawn the FreeRTOS background task that drives the transport engine.
     *
     * Call once from setup() after begin(). After this returns the transport is
     * fully event-driven. send(), sendAsync(), onReceive(), and receive() are
     * all thread-safe.
     *
     * @param stackDepth  Task stack in words (default 3072).
     * @param priority    FreeRTOS task priority (default 5, above Arduino loop()).
     * @param core        CPU core to pin the task to (0 or 1, default 1).
     * @return true if the task was created successfully.
     */
    bool startTask(uint32_t stackDepth = 3072,
                   UBaseType_t priority = 5,
                   BaseType_t core = 1);

    /** @brief Stop and delete the background task. Safe to call at any time. */
    void stopTask();

    // --- TX API ---

    /**
     * @brief Send a payload to a destination radio address.
     *
     * For unicast (dest != 0xFF): blocks the calling task until the remote
     * transport layer acknowledges the packet (or all retries are exhausted).
     * The CPU is released to other tasks while waiting -- no busy spin.
     * For broadcast (dest == 0xFF): returns immediately after TX.
     *
     * @warning Do NOT call send() from inside an onReceive() callback.
     *          Use sendAsync() instead to avoid deadlock.
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
     * @brief Non-blocking send -- queues the packet and returns immediately.
     *
     * Safe to call from inside an onReceive() callback.
     * Call isBusy() to check whether the previous async TX completed.
     *
     * @return true if the packet was queued; false if a TX is already in progress.
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
     *
     * The callback fires from the background task on every valid incoming packet.
     * It is safe to call sendAsync() from within the callback.
     *
     * @warning Do NOT call send() from inside the callback -- deadlock will occur.
     *          Use sendAsync() instead.
     *
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
     * Thread-safe. May be called from loop() concurrently with the background task.
     *
     * @param[out] src   Radio address of the sender.
     * @param[out] type  Packet type.
     * @param[out] data  Payload bytes (caller must provide >= RADIO_MAX_PAYLOAD bytes).
     * @param[out] len   Payload length.
     * @return true if a packet was available and copied.
     */
    bool receive(uint8_t* src, PacketType* type,
                 uint8_t* data, uint8_t* len);

    // --- Auto-power ---

    /**
     * @brief Evaluate and optionally schedule a TX power adjustment for one slave.
     * Called automatically by the background task when autoPowerEnabled = true.
     * Power changes are deferred and applied only when the TX machine is idle.
     */
    void runAutoPower(uint8_t slaveAddr);

    // --- Diagnostics ---

    const LinkStats& stats() const { return _stats; }
    void resetStats() { memset(&_stats, 0, sizeof(_stats)); }

    /** @brief Current TX power level (1-8) used for the given slave. */
    uint8_t slavePower(uint8_t slaveAddr) const;

   private:
    // --- TX state machine ---

    enum class TxState : uint8_t {
        IDLE,
        SENDING,
        WAIT_ACK,
        SUCCESS,  // kept for state completeness; resolved inline in _processComplete()
        RETRY,
        FAILED,  // kept for state completeness; resolved inline in _processComplete()
    };

    void _updateTx();
    void _doSend(const RadioPacket& pkt);

    /** @brief Inner sendAsync logic, called with _txMutex already held. */
    bool _sendAsyncInternal(uint8_t dest, PacketType type,
                            const uint8_t* data, uint8_t len);

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

    // --- Background task ---

    static void _taskFunc(void* arg);

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
    bool _seqSeen[256];     /// Whether we have received anything from this src

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

    // --- FreeRTOS handles ---
    TaskHandle_t _taskHandle = nullptr;
    SemaphoreHandle_t _txMutex = nullptr;    /// Protects TX state machine
    SemaphoreHandle_t _rxMutex = nullptr;    /// Protects _rxQueue head/tail
    SemaphoreHandle_t _ackEvent = nullptr;   /// Given when ACK/NACK resolves send()
    SemaphoreHandle_t _txTrigger = nullptr;  /// Given by sendAsync() to wake task

    uint8_t _pendingPower = 0;  /// Deferred setPower() target (0 = no change pending)

    // --- Encryption ---
    RadioCipher _cipher;  /// AES-128-CTR context

    // --- Dependencies and config ---
    HC12Driver* _driver = nullptr;
    TransportConfig _cfg = TRANSPORT_DEFAULT_CONFIG;
    LinkStats _stats = {};
};
