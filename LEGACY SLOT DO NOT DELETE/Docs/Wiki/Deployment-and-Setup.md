# 🚦 Deployment & Setup

This guide covers everything you need to get your Five Channel Hub up and running, from hardware assembly to first data logs and field configuration.

---

## 1. Hardware Preparation

### Required Components

| Component                | Notes                                             |
|--------------------------|--------------------------------------------------|
| RP2040-based main board  | Five Channel Hub custom or compatible PCB         |
| PSRAM (8MB+)             | Confirm correct wiring to SPI                     |
| SD Card (FAT32, 2–128GB) | UHS-I/SDHC recommended for speed                  |
| Up to 5 EGP/MFL sensors  | Connect as per pinout                             |
| EEPROM (optional)        | For persistent IDs/calibration                    |
| NeoPixel LED             | Status indicator                                  |
| Power Supply/USB         | 5V input, 3.3V logic everywhere                   |
| Cables and connectors    | As required                                       |

### Wiring and Assembly

- Double-check all wiring against [Hardware-Overview.md](Hardware-Overview.md) and your board’s schematic.
- Pay special attention to:
    - SD card detect and power switch pins
    - PSRAM SPI connections (CS, SCK, MOSI, MISO)
    - Correct orientation of sensors and NeoPixel
    - Pullups on I2C lines (if needed)

---

## 2. Software Environment

### Option A: Arduino IDE

1. Install [Arduino IDE](https://www.arduino.cc/en/software).
2. Add the [earlephilhower/arduino-pico](https://github.com/earlephilhower/arduino-pico) board package.
    - **Boards Manager URL:** `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`
3. Install required libraries:
    - SdFat
    - Adafruit NeoPixel
    - EasyTransfer
    - Any sensor-specific libraries (e.g., MFLSensorHead/LDC1101)
4. Download or clone this repository.

### Option B: PlatformIO

1. Install [VS Code](https://code.visualstudio.com/) and the [PlatformIO extension](https://platformio.org/).
2. Open the project folder.
3. PlatformIO auto-detects dependencies from `platformio.ini`.

---

## 3. Building and Flashing

### Arduino IDE

- Open `src/main.cpp` or the main sketch.
- Set board to **Raspberry Pi Pico** or your custom RP2040 target.
- Connect to PC via USB (hold BOOTSEL if required).
- Click **Upload**.

### PlatformIO

- Connect board via USB.
- Click “Upload” in the PlatformIO toolbar or run:
    ```
    platformio run --target upload
    ```

---

## 4. First Boot and Verification

1. **Insert SD card** (FAT32 formatted).
2. **Power board via USB or external 5V**.
3. **Observe NeoPixel**:
    - **Blue** = Idle/Ready
    - **Red** = Error (check serial for details)
4. **Open Serial Monitor** (115200 baud) for debug output.
5. **Check for**:  
    - “Card successfully initialized.”  
    - “Sensors initialised.”

---

## 5. Field Configuration

### Changing IDs, Sensor Types, or Calibration

- Use I2C or Serial commands (see [API-Reference.md](API-Reference.md)).
- For persistent settings, ensure EEPROM is present and detected.

#### Example: Setting Hardware ID (HID) via I2C

- Send HID register address (`0x07`) and new value.
- Read back for confirmation.

---

## 6. Triggering Data Acquisition

- Set `MODE_pin` **HIGH** (Acquisition Mode).
- Pulse `TRIG_pin` (hardware trigger) or send trigger command (software).
- NeoPixel turns **Green** during logging.

---

## 7. Retrieving Data

- **Safely power down** before removing SD card.
- Copy `.hex` log files to your PC for analysis.

---

## 8. Common Build/Deployment Errors

| Symptom           | Likely Cause                         | Fix                            |
|-------------------|--------------------------------------|-------------------------------|
| No serial output  | Wrong board selected / bad USB cable | Re-check settings/cable        |
| No SD detected    | Miswired SD, wrong format            | Check wiring, reformat card    |
| NeoPixel Red      | SD/sensor/PSRAM init failed          | Check serial debug, connections|
| Watchdog resets   | Firmware hang, miswired sensor       | Check wiring, reboot           |

---

## 9. Updating Firmware

- Repeat the build/flash steps.
- Configuration and log files remain on EEPROM/SD unless erased.

---

## 10. Advanced: PlatformIO Project Layout

```plaintext
project/
│-- include/            # Header files, config, pin definitions
│-- lib/                # Custom libraries (optional)
│-- src/
│   ├─ main.cpp         # Main firmware logic
│   └─ function.cpp     # Logging and system functions
│-- platformio.ini      # PlatformIO build config
```

---

**Next Steps:**  
- See [Data-Logging.md](Data-Logging.md) for log file structure and parsing.
- For field troubleshooting, consult [Troubleshooting.md](Troubleshooting.md).