# HC-12 Radio Transport Library

Application-agnostic reliable transport layer for **HC-12 433 MHz radios** on **ESP32**.

Behaves like a wireless **UART** link with reliability features: CRC, ACK/retry, sequence numbers, duplicate detection, and adaptive TX power - completely independent of any application protocol.

---

## Features

| Feature        | Details                                               |
| -------------- | ----------------------------------------------------- |
| **Transport**  | CRC16-CCITT, ACK/NACK, 3 retries, seq-number dedup    |
| **Addressing** | 1 master + up to 238 slaves, broadcast 0xFF           |
| **Auto-power** | Per-slave adaptive TX power via retry-count heuristic |
| **Protocol**   | Payload-agnostic - send any bytes                     |
| **Modbus**     | Optional `ModbusBridge` layer (not required)          |
| **RTOS**       | Uses FreeRTOS architecture                            |

---

## Packet Wire Format

```
┌─────┬──────┬─────┬─────┬──────┬─────┬─────────┬────────┐
│ SOF │ DEST │ SRC │ SEQ │ TYPE │ LEN │ PAYLOAD │ CRC16  │
└─────┴──────┴─────┴─────┴──────┴─────┴─────────┴────────┘
   1      1     1     1     1     1      0–64       2
```

- **SOF** = `0xAA`
- **DEST / SRC**: `0x01` = master, `0x10–0xFE` = slaves, `0xFF` = broadcast
- **TYPE**: `DATA=0x00`, `ACK=0x01`, `NACK=0x02`, `PING=0x03`, `PONG=0x04`, `STATUS=0x05`
- **CRC16-CCITT** covers `DEST..PAYLOAD` (not SOF or CRC bytes themselves)
- Max frame: 72 bytes -> comfortable within HC-12 FU3 throughput at 19200 baud

```

---

## HC-12 Configuration

| Parameter | Range | Default | AT Command |
|---|---|---|---|
| Channel | 1–127 | 5 | `AT+C005` |
| TX Power | 1–8 | 8 (20 dBm) | `AT+P8` |
| FU Mode | FU1–FU4 | FU3 | `AT+FU3` |
| Baud | 1200–115200 | 19200 | `AT+B19200` |

AT commands are typically processed at **9600 baud**, but the library dynamically scans all standard baud rates during `begin()` to safely auto-detect and sync AT-mode parameters regardless of the module's unknown prior state.

Runtime change: `radio.setChannel(7)`, `radio.setPower(4)`, `radio.setMode(HC12Mode::FU3)`.

---

## Auto-Power Control

When `autoPowerEnabled = true`, the master tracks retry counts per slave:

- **Retries ≥ threshold** in an interval -> requires power increase by one step
- **Zero retries** for N consecutive intervals -> allows power decrease by one step
- Power limits clamp to `[autoPowerMinP ... autoPowerMaxP]`
- The transport calculates the highest required power across **all** active slaves and dynamically applies that single `globalMaxP` level to the shared HC-12 hardware. This safely guarantees link stability for the furthest slave without dropping connections.

Check current logical power for a slave: `transport.slavePower(0x10)` -> `1–8`.

---

## Timing Defaults

| Parameter | Default |
|---|---|
| ACK timeout | 75 ms |
| Retries per send | 3 |
| TX->RX switch delay | 10 ms |
| Inter-packet gap | 10 ms |
| Auto-power interval | 5000 ms |
| AT entry delay | 40 ms |
| AT exit delay | 80 ms |

---

## Broadcast & Discovery

- Send to `0xFF` -> delivered to all slaves, **no ACK expected**
- `ModbusBridge::discover(ms)` broadcasts `PKT_PING`, collects `PKT_PONG` replies
- Each slave's `PKT_PONG` lists its registered Modbus device addresses
- Master builds routing table automatically

---

## Recommended Limits

| Parameter | Value |
|---|---|
| Max payload | 64 bytes |
| UART baud | 19200 |
| Retry count | 3 |
| ACK timeout | 75 ms |
| Max slaves tracked | 32 (`HC12_MAX_SLAVES`) |
| RX queue depth | 4 (`HC12_RX_QUEUE_DEPTH`) |
| Master route table | 64 entries (`MODBUS_MAX_ROUTES`) |

All are `#define` overridable before including the headers.
```
