# 📊 Data Logging & File Format

The Five Channel Hub is built for high-throughput, robust data logging. This section covers how data is buffered, logged, and stored, along with details for parsing and analyzing log files.

---

## 1. Data Flow Overview

```mermaid
flowchart LR
    Sensors --samples--> [Acquire]
    [Acquire] --raw data--> [PSRAM Buffer]
    [PSRAM Buffer] --blocks--> [Logging (Core 1)]
    [Logging] --files--> [SD Card]
    [SD Card] --removal--> [PC/Host Analysis]
```

---

## 2. PSRAM Buffering

- **Purpose:** Absorbs high-rate sensor samples, decoupling acquisition from SD write latency.
- **Size:** Default 8MB (configurable via `SRAM_SIZE`)
- **Chunk Size:** Data is read/written in blocks (`BYTES_READ_AT_ONCE`), default 15 samples/block for efficiency.
- **Wrap-around:** Circular buffer; firmware tracks write and read pointers for overflow protection.

---

## 3. Log File Structure

### File Naming

- **Pattern:** `HIDxxSIDxx_yyyy.hex`
    - `HID`: Hardware ID (from EEPROM/config)
    - `SID`: Session ID (from EEPROM/config)
    - `yyyy`: File counter/index

### File Format: Binary Hex

- **Extension:** `.hex` (but not Intel HEX; raw binary, optionally hex-dumped)
- **Record Layout:**  
  Each sample record (per sensor reading) typically contains:
    - Timestamp (e.g., micros since boot, 4 bytes)
    - Sensor Channel ID (1 byte)
    - Sensor Data (variable, e.g., 2–8 bytes depending on sensor)
    - Status/Flags (1 byte)
    - Optional: Error codes, calibration info, etc.

#### Example: EGP Mode Sample (14 bytes, typical)

| Offset | Size | Field            | Example Value      |
|--------|------|------------------|-------------------|
| 0      | 4    | Timestamp        | 0x5F3D6A01        |
| 4      | 1    | Channel ID       | 0x01              |
| 5      | 8    | Sensor Data      | 0xAB CD EF ...    |
| 13     | 1    | Status Flags     | 0x00              |

#### MFL Mode: 44 bytes/sample (see code for details)

---

## 4. Logging Operation

- **Buffering:** Data is collected in RAM until a full block (`LOG_BUFF_SIZE`, e.g., 32KB) is ready.
- **Preallocation:** Log files are pre-sized for speed.
- **Rotation:** When a file reaches `FILE_SIZE_LIMIT` (default 128MB), a new file is created.
- **Closing:** Files are safely closed with truncation of unused space.

---

## 5. Performance

- **Write speed:** ~34μs per 160-byte write (PSRAM to SD)
- **Read speed:**  ~37μs per 160-byte read (for diagnostics)
- **Max sustainable rate:** Determined by SD card speed and sensor count; PSRAM buffer prevents overruns.

---

## 6. Data Integrity & Error Handling

- **Watchdog:** Ensures logging loop is alive.
- **File Close & Ack:** Logging loop signals file closed via FIFO; Core 0 waits for confirmation.
- **Log Markers:** Error codes/status can be injected as special records or status flags.

---

## 7. Log Parsing Example (Python)

```python
import struct

RECORD_SIZE = 14
with open("HID01SID01_0001.hex", "rb") as f:
    while True:
        record = f.read(RECORD_SIZE)
        if len(record) < RECORD_SIZE:
            break
        timestamp, chan, *data = struct.unpack("<IB8sB", record)
        print(f"Time: {timestamp}, Channel: {chan}, Data: {data[:-1]}, Status: {data[-1]}")
```
> Adjust struct format for MFL or custom modes.

---

## 8. Advanced: Customizing Log Format

- Change buffer size, file size, or record layout in `include/function.h` and related source.
- For new sensors, update the packing/unpacking routines in `Acquire` and `Logging`.

---

**Next Steps:**  
- For protocol and command usage, see [API-Reference.md](API-Reference.md).
- For troubleshooting logging issues, see [Troubleshooting.md](Troubleshooting.md).