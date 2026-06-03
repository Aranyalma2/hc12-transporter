#pragma once

/**
 * @file ModbusMaster.h
 * @brief Modbus RTU over radio -- master side.
 *
 * Sits above RadioTransport. Wraps raw Modbus RTU frames as PacketType::DATA
 * payloads and delivers responses back to the caller.
 *
 * --- Routing ---
 *  The master maintains a route table (uint8_t _route[256]) that maps each
 *  Modbus device address to the radio slave address that serves it.
 *  Routes are learned automatically from incoming responses (the source radio
 *  address of a response is mapped to the Modbus address in byte 0 of the frame).
 *  Unknown addresses are sent to RADIO_ADDR_BROADCAST; when the response arrives
 *  the route is learned and subsequent requests go unicast.
 *  If a Modbus address fails to respond _maxFailures times in a row its route
 *  entry is evicted and it is treated as unknown again.
 *
 * --- TX modes (mutually exclusive, choose one) ---
 *  Serial bridge: call attachSerial() then startTask(). The task reads Modbus
 *  RTU frames from the attached Stream (silence-based framing) and writes
 *  responses back automatically.
 *  API: call send() directly. Response is delivered via onResponse() callback
 *  or the receive() queue.
 *
 * --- Threading ---
 *  The internal FreeRTOS task polls transport.receive() and drives route
 *  learning and response delivery. No transport.onReceive() callback is
 *  registered so the user may still attach their own onReceive on the transport.
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "../RadioTransport.h"

// --- Compile-time limits ---

#ifndef MODBUS_MASTER_RESP_QUEUE
#define MODBUS_MASTER_RESP_QUEUE 4  /// Depth of the response queue (API mode)
#endif

#ifndef MODBUS_MASTER_MAX_FAIL
#define MODBUS_MASTER_MAX_FAIL 3  /// Default consecutive failures before route eviction
#endif

// --- ModbusMaster ---

class ModbusMaster {
   public:
    // --- Callback type ---

    /**
     * @brief Response callback (API mode).
     * Fires from the internal task when a Modbus response arrives.
     * Do NOT call send() from inside this callback -- use the receive() queue
     * or a separate task for back-to-back requests.
     */
    typedef void (*ResponseCallback)(uint8_t radioSrc,
                                     const uint8_t* frame, uint8_t len);

    // --- Lifecycle ---

    /**
     * @brief Attach to a started RadioTransport instance.
     * Must be called after transport.startTask().
     * @param transport      Pointer to a begun + started RadioTransport.
     * @param rxTimeoutMs    Max ms to wait for a Modbus-level response (default 500).
     */
    bool begin(RadioTransport* transport, uint32_t rxTimeoutMs = 500);

    // --- Serial bridge mode ---

    /**
     * @brief Attach a serial port for automatic UART bridging.
     * Modbus RTU frames arriving on this port are routed over radio.
     * Responses are written back to this port automatically.
     * Call startTask() after attachSerial() to run the bridge in the background.
     * @param serial      Stream to read from and write to (e.g. Serial2, SoftwareSerial).
     * @param silenceMs   Inter-frame gap to detect end of Modbus RTU frame (default 5 ms).
     * @param rstPin      GPIO connected to RS-485 driver DE/RE pin for half-duplex control.
     *                    Asserted HIGH before writing, released LOW after flush.
     *                    Pass -1 (default) when not used (full-duplex or hardware auto-dir).
     */
    void attachSerial(Stream* serial, uint32_t silenceMs = 5, int rstPin = -1);

    // --- API mode ---

    /**
     * @brief Send a raw Modbus RTU frame over radio and wait for the response.
     * Routing is automatic. Blocks the calling task until a response arrives
     * or rxTimeoutMs elapses. Only one send() may be in progress at a time.
     * @param frame   Raw Modbus RTU bytes; byte 0 is the Modbus device address.
     * @param len     Frame length (max RADIO_MAX_PAYLOAD).
     * @return true if a response was received before timeout.
     */
    bool send(const uint8_t* frame, uint8_t len);

    /**
     * @brief Register a callback for incoming Modbus responses (API mode).
     * If set, the receive() queue is bypassed.
     */
    void onResponse(ResponseCallback cb);

    /** @brief True if a response frame is queued (only when no onResponse callback). */
    bool available() const;

    /**
     * @brief Pop the oldest response frame from the queue.
     * @param[out] frame      Buffer for the raw Modbus RTU response (>= RADIO_MAX_PAYLOAD).
     * @param[out] len        Number of bytes written into frame.
     * @param[out] radioSrc   Radio address of the slave that replied (optional).
     * @return true if a frame was dequeued.
     */
    bool receive(uint8_t* frame, uint8_t* len, uint8_t* radioSrc = nullptr);

    // --- Background task ---

    /**
     * @brief Spawn the FreeRTOS task that drives route learning and response delivery.
     * Required for serial bridge mode. Optional for API mode (send() works without it,
     * but onResponse() callback and receive() queue will not be updated without the task).
     */
    bool startTask(uint32_t stackDepth = 3072,
                   UBaseType_t priority = 4,
                   BaseType_t core = 1);

    /** @brief Stop and delete the background task. */
    void stopTask();

    // --- Route management ---

    /** @brief Number of consecutive Modbus-level failures before a route is evicted. */
    void setMaxFailures(uint8_t n) { _maxFailures = n; }

    /** @brief Remove all learned routes. */
    void clearRoutes();

    /** @brief Manually add or override a Modbus->radio route. */
    void addRoute(uint8_t modbusAddr, uint8_t radioAddr);

    /** @brief Current radio address for a Modbus device (0 = not learned). */
    uint8_t lookupRoute(uint8_t modbusAddr) const { return _route[modbusAddr]; }

   private:
    // --- Internal send + wait ---
    bool _sendFrame(const uint8_t* frame, uint8_t len);

    // --- Silence-based Modbus RTU frame reader ---
    // Reads bytes from _serial until silenceMs with no new byte.
    // Returns number of bytes read (0 = nothing arrived yet).
    uint8_t _readSerialFrame(uint8_t* buf, uint8_t maxLen);

    // --- Task ---
    static void _taskFunc(void* arg);
    void _runTask();  // called from _taskFunc
    void _processReceivedPacket(uint8_t radioSrc, const uint8_t* data, uint8_t len);

    // --- Members ---
    RadioTransport* _transport = nullptr;
    Stream* _serial = nullptr;
    uint32_t _rxTimeoutMs = 500;
    uint32_t _silenceMs = 5;
    int _rstPin = -1;  // RS-485 DE/RE direction pin (-1 = not used)
    uint8_t _maxFailures = MODBUS_MASTER_MAX_FAIL;
    ResponseCallback _responseCb = nullptr;
    TaskHandle_t _taskHandle = nullptr;

    // Route table: index = Modbus addr, value = radio addr (0 = unknown)
    uint8_t _route[256];
    uint8_t _failCount[256];  // consecutive failures per Modbus addr

    // Pending request sync (one in-flight request at a time)
    SemaphoreHandle_t _pendingMutex = nullptr;  // serialises concurrent send() callers
    SemaphoreHandle_t _responseSem = nullptr;   // given when the response lands
    volatile bool _pendingActive = false;
    volatile uint8_t _pendingMbAddr = 0;  // Modbus addr we are waiting a response for

    // Pending response scratch area (filled by task, read by send())
    uint8_t _pendingRespData[RADIO_MAX_PAYLOAD];
    uint8_t _pendingRespLen = 0;
    uint8_t _pendingRespRadio = 0;

    // Response queue (API mode without callback)
    struct RespEntry {
        uint8_t radioSrc;
        uint8_t data[RADIO_MAX_PAYLOAD];
        uint8_t len;
    };
    RespEntry _respQueue[MODBUS_MASTER_RESP_QUEUE];
    uint8_t _respQHead = 0;
    uint8_t _respQTail = 0;
    SemaphoreHandle_t _respMutex = nullptr;
};
