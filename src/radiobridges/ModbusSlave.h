#pragma once

/**
 * @file ModbusSlave.h
 * @brief Modbus RTU over radio -- slave side.
 *
 * Sits above RadioTransport on the slave node. Receives any PacketType::DATA
 * frame from the master and either:
 *  a) Forwards it to an attached serial port (RS-485 / RS-232), reads the
 *     response back, and sends it to the master via transport.sendAsync().
 *  b) Delivers it to a user-supplied onRequest() callback that fills a
 *     response buffer; the response is then sent via transport.sendAsync().
 *
 * The slave does NOT inspect or filter Modbus device addresses. It passes
 * all bytes through transparently. Address filtering (if needed) is the
 * responsibility of the attached RS-485 bus or the user callback.
 *
 * --- Threading ---
 *  An internal FreeRTOS task polls transport.receive() and handles each
 *  incoming request. For serial bridge mode the task performs the blocking
 *  serial read (waiting for the RS-485 device response) without stalling
 *  the transport background task.
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "../RadioTransport.h"

// --- Compile-time limits ---

#ifndef MODBUS_SLAVE_REQ_QUEUE
#define MODBUS_SLAVE_REQ_QUEUE 4  /// Depth of the incoming request queue
#endif

// --- ModbusSlave ---

class ModbusSlave {
   public:
    // --- Callback type ---

    /**
     * @brief Request callback (local handler mode).
     *
     * Called from the internal task when a Modbus request arrives.
     * Fill response[] with the raw Modbus RTU response bytes and set *responseLen.
     * Set *responseLen = 0 to send no response for this request.
     *
     * @param radioSrc      Radio address of the master that sent the request.
     * @param request       Raw Modbus RTU request bytes.
     * @param requestLen    Number of request bytes.
     * @param response      Buffer to fill with the response (>= RADIO_MAX_PAYLOAD bytes).
     * @param responseLen   Set to the number of response bytes to send (0 = no reply).
     */
    typedef void (*RequestCallback)(uint8_t radioSrc,
                                    const uint8_t* request, uint8_t requestLen,
                                    uint8_t* response, uint8_t* responseLen);

    // --- Lifecycle ---

    /**
     * @brief Attach to a started RadioTransport instance.
     * Must be called after transport.startTask().
     * @param transport   Pointer to a begun + started RadioTransport.
     */
    bool begin(RadioTransport* transport);

    // --- Serial bridge mode ---

    /**
     * @brief Attach a serial port (RS-485 / RS-232) for transparent bridging.
     * Incoming Modbus requests are forwarded to this port byte-for-byte.
     * The response is read back (silence-based framing) and sent to the master.
     * Call startTask() to run the bridge in a background FreeRTOS task.
     *
     * @param serial             Stream to write requests to and read responses from.
     * @param responseTimeoutMs  Max ms to wait for a response from the RS-485 device.
     * @param silenceMs          Inter-frame silence to detect end of RS-485 response.
     * @param rstPin             GPIO connected to RS-485 driver DE/RE pin for half-duplex
     *                           control. Asserted HIGH before writing, released LOW after
     *                           flush. Pass -1 (default) when not used.
     */
    void attachSerial(Stream* serial,
                      uint32_t responseTimeoutMs = 500,
                      uint32_t silenceMs = 5,
                      int rstPin = -1);

    // --- Local handler mode ---

    /**
     * @brief Register a callback to handle incoming requests locally.
     * The callback runs inside the internal task. It must be fast or yield
     * explicitly for heavy processing. Do NOT call transport.send() from within
     * the callback -- use transport.sendAsync() if a transport send is needed.
     */
    void onRequest(RequestCallback cb);

    // --- Background task ---

    /**
     * @brief Spawn the FreeRTOS task that drives the slave bridge.
     * Required for serial bridge mode. Also recommended for callback mode to
     * keep request processing off the transport background task.
     */
    bool startTask(uint32_t stackDepth = 3072,
                   UBaseType_t priority = 4,
                   BaseType_t core = 1);

    /** @brief Stop and delete the background task. */
    void stopTask();

   private:
    // --- Internal request queue ---
    struct ReqEntry {
        uint8_t radioSrc;
        uint8_t data[RADIO_MAX_PAYLOAD];
        uint8_t len;
    };
    ReqEntry _reqQueue[MODBUS_SLAVE_REQ_QUEUE];
    uint8_t _reqQHead = 0;
    uint8_t _reqQTail = 0;
    SemaphoreHandle_t _reqMutex = nullptr;
    SemaphoreHandle_t _reqNotify = nullptr;  // given when a new entry is enqueued

    // --- Serial response reader ---
    uint8_t _readSerialResponse(uint8_t* buf, uint8_t maxLen);

    // --- Task ---
    static void _taskFunc(void* arg);
    void _runTask();
    void _handleRequest(uint8_t radioSrc, const uint8_t* data, uint8_t len);

    // --- Members ---
    RadioTransport* _transport = nullptr;
    Stream* _serial = nullptr;
    uint32_t _responseTimeoutMs = 500;
    uint32_t _silenceMs = 5;
    int _rstPin = -1;  // RS-485 DE/RE direction pin (-1 = not used)
    RequestCallback _requestCb = nullptr;
    TaskHandle_t _taskHandle = nullptr;
};
