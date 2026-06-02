#pragma once

/**
 * @file RingBuffer.h
 * @brief Lock-free, statically-allocated ring buffer.
 *
 * SIZE must be a power of 2 (8, 16, 32, 64, 128, 256, 512).
 * The power-of-2 constraint allows fast modulo via bitwise AND.
 *
 * Thread safety: single-producer / single-consumer safe on ESP32
 * (head written only by producer, tail written only by consumer).
 */

#include <stdint.h>

template <uint16_t SIZE>
class RingBuffer {
    static_assert((SIZE & (SIZE - 1)) == 0, "RingBuffer SIZE must be a power of 2");
    static constexpr uint16_t MASK = SIZE - 1;

   public:
    RingBuffer() : _head(0), _tail(0) {}

    /**
     * @brief Push one byte. Returns false if full (byte is dropped).
     */
    bool push(uint8_t b) {
        uint16_t next = (_head + 1u) & MASK;
        if (next == _tail) return false;  // full
        _buf[_head] = b;
        _head = next;
        return true;
    }

    /**
     * @brief Pop one byte. Returns false if empty.
     */
    bool pop(uint8_t& b) {
        if (_head == _tail) return false;  // empty
        b = _buf[_tail];
        _tail = (_tail + 1u) & MASK;
        return true;
    }

    /**
     * @brief Peek at the next byte without consuming it.
     */
    bool peek(uint8_t& b) const {
        if (_head == _tail) return false;
        b = _buf[_tail];
        return true;
    }

    /**
     * @brief Number of bytes available to read.
     */
    uint16_t available() const {
        return (_head - _tail) & MASK;
    }

    /**
     * @brief Free space (bytes that can still be pushed).
     */
    uint16_t free() const {
        return MASK - available();
    }

    bool empty() const { return _head == _tail; }

    /**
     * @brief Discard all data.
     */
    void clear() { _head = _tail = 0; }

    /**
     * @brief Read up to maxLen bytes into buf. Returns bytes actually read.
     */
    uint16_t read(uint8_t* buf, uint16_t maxLen) {
        uint16_t count = 0;
        while (count < maxLen) {
            uint8_t b;
            if (!pop(b)) break;
            buf[count++] = b;
        }
        return count;
    }

   private:
    uint8_t _buf[SIZE];
    volatile uint16_t _head;  // written by producer
    volatile uint16_t _tail;  // written by consumer
};
