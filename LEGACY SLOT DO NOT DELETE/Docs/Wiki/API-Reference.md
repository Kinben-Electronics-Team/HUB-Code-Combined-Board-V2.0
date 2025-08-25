# 📖 API & Command Reference

This section provides a comprehensive reference for all configuration, control, and diagnostic commands supported by the Five Channel Hub, including I2C, Serial, EEPROM, and diagnostic/test APIs.

---

## 1. I2C Command Protocol

- **Address:** Default slave address is `0x01` (configurable)
- **Speed:** 100kHz recommended (can be increased as needed)
- **Format:**  
  - **Write:** `[register][data]` (set value)
  - **Read:** `[register]` (followed by read request to get value)

### Supported I2C Registers & Commands

| Name                | Hex   | R/W | Description                                 |
|---------------------|-------|-----|---------------------------------------------|
| I2C_CMD_SD_CON      | 0x04  | W   | Connect SD card (switch to MCU)             |
| I2C_CMD_SD_DISCON   | 0x05  | W   | Disconnect SD card (release for reader)     |
| SID_REG             | 0x06  | R/W | Session ID (for file naming, etc.)          |
| HID_REG             | 0x07  | R/W | Hardware ID (for file naming, etc.)         |
| SENSOR_TYPE_REG     | 0x08  | R/W | Select sensor type (EGP/MFL)                |
| NUM_EGP_REG         | 0x09  | R/W | Number of EGP sensors                       |
| ANGLE_AXIS_REG      | 0x0A  | R/W | Axis for angle calculation                  |
| MAG_AXIS_REG        | 0x0B  | R/W | Axis for magnetic sensor                    |
| AVG_SAMPLE_REG      | 0x0C  | R/W | Number of samples for averaging (max 25)    |
| GET_SENSOR_DATA     | 0x10  | R   | Get last acquired sensor data (block read)  |
| GET_CONFIG          | 0x11  | R   | Get current configuration block             |
| DIAG_SD_TEST        | 0x20  | W   | Run SD card write/read test                 |
| DIAG_PSRAM_TEST     | 0x21  | W   | Run PSRAM speed/integrity test              |
| DIAG_SENSOR_TEST    | 0x22  | W   | Re-init and check all sensors               |

*See `function.h` for the latest, full list of supported registers.*

---

## 2. Serial Commands

- **Interface:** UART, default 115200 baud (see `BAUDRATE`)
- **Usage:**  
  - Human-readable commands for debugging/config
  - Diagnostic output and sensor data streaming

### Example Serial Commands

| Command           | Description                    |
|-------------------|-------------------------------|
| `get_status`      | Print current status/config    |
| `test_sd_card`    | SD card write/read test       |
| `test_psram`      | PSRAM test                    |
| `check_sensors`   | Sensor init/status check      |
| `set_sid [val]`   | Set Session ID                |
| `set_hid [val]`   | Set Hardware ID               |
| `set_mode [EGP/MFL]`| Set sensor type             |
| `set_avg [val]`   | Set averaging sample count    |

---

## 3. EEPROM Settings

Data stored persistently in EEPROM:

- `SID` (Session ID)
- `HID` (Hardware ID)
- `Sensor Type` (EGP/MFL)
- `Angle Sensor Cal` (high/low averages)
- `Averaging Count` (samples)

EEPROM is used to restore configuration at boot and for file naming/log partitioning.

---

## 4. Diagnostic Commands

- **SD Card Test:**  
  - Command: `test_sd_card` (serial) or `DIAG_SD_TEST` (I2C)
  - Function: Writes and reads test blocks, checks for errors, prints result

- **PSRAM Test:**  
  - Command: `test_psram` or `DIAG_PSRAM_TEST`
  - Function: Write/read speed, data integrity (prints to serial)

- **Sensor Test:**  
  - Command: `check_sensors` or `DIAG_SENSOR_TEST`
  - Function: Re-initializes and verifies all sensor channels

---

## 5. Sensor Data Access

- For block reads (e.g., via I2C/serial), data is returned as packed binary records.
    - EGP: Each record is `numEGP * 14` bytes
    - MFL: Each record is 44 bytes
- Use the `GET_SENSOR_DATA` register to retrieve

### Example I2C Read Sequence

1. Master writes `[GET_SENSOR_DATA]`
2. Master reads N bytes (N = record size)

---

## 6. Error Codes & Status

- Errors are reported:
    - Via status flags in sensor data records
    - By LED (Red = Error)
    - On Serial console (detailed message)
    - As special marker records in log file (if critical)

Common error sources:
- SD card not present, write failure
- PSRAM test fail
- Sensor init fail

---

## 7. Example I2C Transaction

**Set averaging sample count to 10:**
- Write: `[AVG_SAMPLE_REG][0x0A]`

**Get current configuration:**
- Write: `[GET_CONFIG]`
- Read: (returns packed config block)

---

## 8. Custom Extensions

- Add new commands by extending the command handler in `Acquire`.
- Document in this file for user/developer reference.

---

**See [Developer-Guide.md](Developer-Guide.md) for adding new command support.**