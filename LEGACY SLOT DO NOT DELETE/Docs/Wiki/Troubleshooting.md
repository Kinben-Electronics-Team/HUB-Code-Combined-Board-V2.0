# 🛠️ Troubleshooting

This guide helps you quickly diagnose and resolve common issues with the Five Channel Hub, from deployment to long-term field use.

---

## 1. LED Status Codes

| Color | State         | Meaning                                  |
|-------|--------------|------------------------------------------|
| Blue  | Idle/Ready   | Board powered, waiting for trigger       |
| Green | Logging      | Data acquisition and SD logging active   |
| Red   | Error        | Hardware or software error detected      |

---

## 2. Common Problems & Solutions

### SD Card Not Detected or Logging Fails

- **Symptoms:** Red LED, "SD card not found" or write errors on serial.
- **Causes:**
    - Card not formatted FAT32 or not inserted properly
    - Card detect pin not wired or not pulled low
    - SD card power not enabled
    - Wiring error on SDIO/SPI lines
- **Fixes:**
    - Reformat card as FAT32 on a PC
    - Reseat card, check card detect pin
    - Confirm SD power switch pin logic
    - Test with known good card

---

### No Data Logging When Triggered

- **Symptoms:** NeoPixel stays blue, no new files on SD.
- **Causes:**
    - MODE_pin not set HIGH (not in acquisition mode)
    - TRIG_pin not pulsed, or trigger logic inverted
    - Sensor init failed
- **Fixes:**
    - Set MODE_pin HIGH before triggering
    - Check trigger wiring and logic level
    - Review serial debug output for errors

---

### Watchdog Resets or Device Randomly Reboots

- **Symptoms:** Periodic resets; "Watchdog reset" messages (if enabled).
- **Causes:**
    - Firmware hang (e.g., blocked SD card access)
    - Sensor bus conflict or hardware fault
    - Power supply instability
- **Fixes:**
    - Check PSRAM and SD card wiring
    - Ensure sensors are not shorted or unresponsive
    - Use a stable, adequate power source

---

### Sensor Failures

- **Symptoms:** "Sensor init failed" or missing channels in data.
- **Causes:**
    - Sensor wiring error (SPI/I2C/shift register)
    - Power to sensors not enabled
    - Incorrect sensor type/config
- **Fixes:**
    - Double-check wiring, use service mode for diagnostics
    - Enable SENSOR_PS_EN_pin
    - Reconfigure sensor type via command/API

---

### File Write Errors or Corrupted Files

- **Symptoms:** Incomplete/corrupted logs, write errors.
- **Causes:**
    - Removing SD card while powered/logging
    - SD card wear/failure
    - Buffer overrun not handled (rare)
- **Fixes:**
    - Only remove SD card when NeoPixel is blue (idle)
    - Replace card if recurring
    - Extend PSRAM buffer for higher rates

---

## 3. Diagnostic & Test Commands

- `test_sd_card` / I2C `DIAG_SD_TEST` — SD card speed/integrity test
- `test_psram` / I2C `DIAG_PSRAM_TEST` — PSRAM test
- `check_sensors` / I2C `DIAG_SENSOR_TEST` — Sensor hardware test

Run these via serial console or master I2C as needed.

---

## 4. Field Recovery Checklist

- Power cycle board (wait for full reset)
- Reflash firmware if persistent errors remain
- Restore factory configuration via command
- Swap SD card and retest
- Contact support or file a [GitHub Issue](https://github.com/Kinben-Electronics-Team/five-channel-hub/issues) with logs and hardware details

---

## 5. Advanced Debug Tips

- Use a logic analyzer or oscilloscope to check SD/SPI/trigger lines
- Watch serial debug output for error messages
- Use service mode to test sensors one-by-one

---

## 6. Getting More Help

- See [FAQ.md](FAQ.md) for common questions and advanced use
- Open a GitHub issue with a detailed description and log snippets

---