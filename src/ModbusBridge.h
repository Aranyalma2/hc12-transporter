#pragma once

/**
 * @file ModbusBridge.h
 * @brief Optional Modbus RTU mapping layer above RadioTransport.
 *
 * This is NOT part of the transport. It is an application-layer helper
 * that maps Modbus device addresses to radio slave addresses and wraps
 * Modbus RTU frames as PKT_DATA payloads.
 *
 * --- Architecture ---
 *
 *  [Modbus RTU Master / Application]
 *          |
 *    ModbusBridgeMaster::sendModbus(modbusAddr, frame, len)
 *          |  looks up radio address from routing table
 *          v
 *    RadioTransport::send(radioAddr, PKT_DATA, frame, len)
 *          |
 *          v RF link
 *          |
 *    RadioTransport::receive() on slave
 *          |
 *    ModbusBridgeSlave::update() delivers to local RS-485
 *          |
 *          v
 *  [Local Modbus RTU Slave / RS-485]
 *
 * --- One master / multi-slave routing ---
 *
 *  Master maintains a ModbusRouteEntry[] table:
 *    Modbus device addr 0x01 -> radio slave 0x10
 *    Modbus device addr 0x02 -> radio slave 0x10  (same radio node!)
 *    Modbus device addr 0x05 -> radio slave 0x11
 *    ...
 *
 *  Routes can be added:
 *   a) Statically: addRoute(modbusAddr, radioAddr)
 *   b) Dynamically: discover() broadcasts a PKT_PING; each slave replies
 *      with PKT_PONG listing its served Modbus addresses.
 *
 * --- Broadcast discovery flow ---
 *
 *  Master -> BROADCAST PKT_PING (empty payload)
 *  Each slave -> PKT_PONG to RADIO_ADDR_MASTER
 *    payload = [radio_addr, count, modbusAddr0, modbusAddr1, ...]
 *  Master collects PONGs within timeoutMs, builds route table.
 *
 * --- Slave registration ---
 *
 *  On the slave side, call:
 *    bridge.registerLocalDevice(modbusAddr);
 *  The bridge will respond to PKT_PING broadcasts with its device list,
 *  and deliver incoming PKT_DATA payloads matching those addresses.
 */

#include <Arduino.h>

#include "RadioTransport.h"

// --- Compile-time limits ---

#ifndef MODBUS_MAX_ROUTES
#define MODBUS_MAX_ROUTES 64  /// Master: maximum routing table entries
#endif

#ifndef MODBUS_MAX_LOCAL_DEVICES
#define MODBUS_MAX_LOCAL_DEVICES 16  /// Slave: max Modbus devices on this node
#endif

// --- Route entry ---

struct ModbusRouteEntry {
    uint8_t modbusAddr;  /// Modbus device address (1–247)
    uint8_t radioAddr;   /// Radio slave address (0x10–0xFE)
};

// --- Received Modbus frame ---

struct ModbusFrame {
    uint8_t radioSrc;                 /// Radio address of the slave that sent this
    uint8_t data[RADIO_MAX_PAYLOAD];  /// Raw Modbus RTU bytes
    uint8_t len;
};

// --- ModbusBridge class ---

/**
 * @brief Modbus RTU / RadioTransport bridge for both master and slave roles.
 *
 * Role is determined by the localAddr in the TransportConfig passed to begin():
 *  - RADIO_ADDR_MASTER (0x01) -> master bridge behaviour
 *  - Any other value          -> slave bridge behaviour
 *
 * Both roles coexist in a single class to share the routing helpers.
 */
class ModbusBridge {
   public:
    // --- Lifecycle ---

    /**
     * @brief Attach to an initialised RadioTransport.
     * @param transport  Pointer to a begun RadioTransport instance.
     */
    bool begin(RadioTransport* transport, uint8_t localRadioAddr);

    // --- MASTER — Route table ---

    /**
     * @brief Add a static Modbus address -> radio address mapping.
     * @return false if the table is full.
     */
    bool addRoute(uint8_t modbusAddr, uint8_t radioAddr);

    /**
     * @brief Remove a route entry.
     */
    void removeRoute(uint8_t modbusAddr);

    /**
     * @brief Clear all routes.
     */
    void clearRoutes();

    /**
     * @brief Look up the radio address for a Modbus device.
     * @return Radio address, or 0 if not found.
     */
    uint8_t lookupRadioAddr(uint8_t modbusAddr) const;

    // --- MASTER — Discovery ---

    /**
     * @brief Broadcast a PKT_PING and collect slave responses.
     *
     * Blocks for up to timeoutMs. For each PKT_PONG received, adds the
     * reported Modbus addresses to the route table.
     *
     * @param timeoutMs  How long to collect PONG replies (e.g. 500 ms).
     * @return Number of slave nodes that responded.
     */
    uint8_t discover(uint32_t timeoutMs = 500);

    // --- MASTER — TX ---

    /**
     * @brief Send a Modbus RTU frame to the appropriate radio slave.
     *
     * The destination radio address is looked up from the route table.
     * The frame is sent as PKT_DATA. Waits for transport-level ACK.
     *
     * @param modbusAddr  Modbus device address (used for routing).
     * @param frame       Raw Modbus RTU bytes.
     * @param len         Frame length.
     * @return true if transport ACK received; false on timeout/no route.
     */
    bool sendModbus(uint8_t modbusAddr, const uint8_t* frame, uint8_t len);

    /**
     * @brief Send a Modbus RTU frame to a specific radio address (bypasses routing).
     */
    bool sendModbusDirect(uint8_t radioAddr, const uint8_t* frame, uint8_t len);

    // --- MASTER — RX ---

    /**
     * @brief Returns true if a Modbus response frame is available.
     */
    bool modbusAvailable() const;

    /**
     * @brief Pop the oldest received Modbus frame.
     * @param[out] frame  Filled with source radio address and RTU bytes.
     * @return true if a frame was available.
     */
    bool receiveModbus(ModbusFrame* frame);

    // --- SLAVE — Registration ---

    /**
     * @brief Register a Modbus device address that this slave serves.
     * These addresses are broadcast in PONG responses to discovery pings.
     * @return false if the local device list is full.
     */
    bool registerLocalDevice(uint8_t modbusAddr);

    /**
     * @brief Unregister a local Modbus device.
     */
    void unregisterLocalDevice(uint8_t modbusAddr);

    // --- SLAVE — RX (application calls this to get incoming requests) ---

    /**
     * @brief Returns true if an incoming Modbus request frame is queued.
     * (Slave side only — frames addressed to registered local devices.)
     */
    bool requestAvailable() const;

    /**
     * @brief Pop the oldest incoming Modbus request.
     * @param[out] frame  Source radio address + RTU frame.
     * @return true if a frame was available.
     */
    bool receiveRequest(ModbusFrame* frame);

    // --- SLAVE — TX (respond to master) ---

    /**
     * @brief Send a Modbus RTU response frame back to the master.
     *
     * @param masterRadioAddr  Radio address of the master (usually RADIO_ADDR_MASTER).
     * @param frame            Raw Modbus RTU response bytes.
     * @param len              Frame length.
     * @return true if transport ACK received.
     */
    bool sendResponse(uint8_t masterRadioAddr, const uint8_t* frame, uint8_t len);

    // --- Common ---

    /**
     * @brief Must be called frequently to process incoming transport packets.
     * Handles PING/PONG internally; delivers DATA frames to the appropriate queue.
     */
    void update();

   private:
    void _handlePing(const RadioPacket& pkt);
    void _handlePong(const RadioPacket& pkt);
    void _handleData(const RadioPacket& pkt);

    bool _isLocalDevice(uint8_t modbusAddr) const;

    RadioTransport* _transport = nullptr;
    uint8_t _localRadioAddr = 0;

    // --- Master route table ---
    ModbusRouteEntry _routes[MODBUS_MAX_ROUTES];
    uint8_t _routeCount = 0;

    // --- Slave local device list ---
    uint8_t _localDevices[MODBUS_MAX_LOCAL_DEVICES];
    uint8_t _localDeviceCount = 0;

    // --- RX queues ---
    // Master: incoming responses from slaves
    static constexpr uint8_t FRAME_QUEUE_DEPTH = 4;
    ModbusFrame _responseQueue[FRAME_QUEUE_DEPTH];
    uint8_t _respQHead = 0;
    uint8_t _respQTail = 0;

    // Slave: incoming requests from master
    ModbusFrame _requestQueue[FRAME_QUEUE_DEPTH];
    uint8_t _reqQHead = 0;
    uint8_t _reqQTail = 0;

    bool _enqueueFrame(ModbusFrame* queue, uint8_t& head, uint8_t tail,
                       const ModbusFrame& f);
    bool _dequeueFrame(ModbusFrame* queue, uint8_t head, uint8_t& tail,
                       ModbusFrame* out);
};
