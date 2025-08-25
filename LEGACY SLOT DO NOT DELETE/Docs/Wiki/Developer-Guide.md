# 🧑‍💻 Developer Guide

This guide is for contributors, maintainers, and advanced users who want to extend, debug, or adapt the Five Channel Hub firmware and hardware.

---

## 1. Project Structure

| Folder/File            | Purpose                                     |
|----------------------- |---------------------------------------------|
| `/src/`                | Main source code (C++, C)                   |
| `/include/`            | Header files, configuration, pin defs       |
| `/lib/`                | Optional third-party or custom libraries    |
| `/docs/wiki/`          | Documentation files (for in-repo wikis)     |
| `platformio.ini`       | PlatformIO build configuration (if used)    |
| `README.md`            | Project summary, features, quick start      |

---

## 2. Key Classes & Modules

### **Sensors**
- **Purpose:** Handles all sensor init, configuration, and raw data reading.
- **Extending:** Add new sensor models by implementing new functions and updating the sensor enumeration/type logic.

### **Acquire**
- **Purpose:** Orchestrates acquisition, buffering, and command parsing.
- **Extending:** Add new acquisition modes or sample-packing strategies here.

### **Logging**
- **Purpose:** Manages SD card, file system, log formatting, and buffer reads.
- **Extending:** Add new logging formats, cloud upload, or alternative storage.

### **psram_spi**
- **Purpose:** Low-level PSRAM buffer management.
- **Extending:** Port to different PSRAM chips or alternative RAM.

---

## 3. Adding a New Sensor

1. **Add hardware wiring** and update pin definitions in `function.h`.
2. **Implement initialization and read functions** in `Sensors` and/or `Acquire`.
3. **Update sample packing/unpacking** to include new data fields.
4. **Document changes** in [Data-Logging.md](Data-Logging.md) and [API-Reference.md](API-Reference.md).

---

## 4. Adding a New Command/API

1. **Define register/command macro** in `function.h`.  
   Example:
   ```cpp
   #define I2C_CMD_MYNEWCOMMAND 0x30
   ```
2. **Handle the command** in `Acquire::ExecuteWireCmd()` or Serial handler:
   ```cpp
   case I2C_CMD_MYNEWCOMMAND:
       // Implement your custom logic
       break;
   ```
3. **Test via I2C/Serial** and update documentation.

---

## 5. Debugging & Testing

- **Serial Monitor:** Use at 115200 baud for live logs and troubleshooting.
- **Diagnostic Functions:**  
  - `test_sd_card()` – SD write/read integrity and speed.
  - `test_psram()` – RAM performance and correctness.
  - `checkSensors()` – Sensor status and communication.
- **Service Mode:** Configure and test without logging active.
- **Field Upgrade:** Safe to re-flash firmware without corrupting logs.

---

## 6. Coding Standards & Style

- **Languages:** C++ (main logic), C (low-level, e.g., PSRAM)
- **Style:** Use clear, descriptive variable and function names.
- **Documentation:**  
  - Use Doxygen-style comments for functions/classes.
  - Inline comments for hardware-specific logic.
- **Performance:** Keep acquisition and logging fast; avoid blocking calls in time-critical paths.

---

## 7. Unit & Integration Testing

- Write hardware-independent code in modules where possible.
- Simulate sensor data or use stub classes for unit tests.
- Use PlatformIO test framework or custom scripts for automation.

---

## 8. Porting to New Hardware

- Update pin definitions in `function.h`.
- Confirm compatibility of PSRAM, SD, and sensor interface.
- Adjust power supply and level shifting as required.

---

## 9. Contributing

- Fork the repo, create a feature branch.
- Follow the existing file/folder structure.
- Submit a Pull Request (PR) with a detailed description.
- Reference updated documentation in your PR.
- Engage in code review and respond to feedback.

---

## 10. References

- [RP2040 Datasheet](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html)
- [earlephilhower/arduino-pico](https://github.com/earlephilhower/arduino-pico)
- [SdFat Library](https://github.com/greiman/SdFat)
- [Adafruit NeoPixel Library](https://github.com/adafruit/Adafruit_NeoPixel)

---

**For troubleshooting and error recovery, see [Troubleshooting.md](Troubleshooting.md).**