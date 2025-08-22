# 🛠️ Hardware Overview

The **Five Channel Hub** is a professional-grade, RP2040-based data acquisition and logging system. It is designed for flexibility, reliability, and high throughput in demanding data environments.

---

## System Block Diagram

```mermaid
flowchart TD
    MCU[RP2040 Microcontroller]
    PSRAM[8MB PSRAM<br>(External, SPI)]
    SDCARD[SD Card<br>(SPI/SDIO)]
    SENSORS[Up to 5x EGP/MFL Sensors]
    EEPROM[EEPROM<br>(I2C)]
    NEOPIXEL[NeoPixel LED]
    I2C[I2C Bus<br>(to Master)]
    SERIAL[UART/Serial<br>(to Master)]
    TRIG[Trigger In<br>(HW or SW)]
    WDT[Watchdog<br>(on-chip)]
    
    MCU --> PSRAM
    MCU --> SDCARD
    MCU --> SENSORS
    MCU --> EEPROM
    MCU --> NEOPIXEL
    MCU --> I2C
    MCU --> SERIAL
    TRIG --> MCU
    WDT --> MCU
```

---

## Pinout and Wiring Summary

| Name              | Pin Number | Function                  | Direction | Notes                         |
|-------------------|------------|---------------------------|-----------|-------------------------------|
| MODE_pin          | 18         | Mode Select (Acq/Service) | Input     | High = Acquisition            |
| TRIG_pin          | 26         | Data Acquisition Trigger  | Input     | Rising edge triggers logging  |
| LED_pin           | 15         | NeoPixel Status           | Output    | RGB status indicator          |
| SENSOR_PS_EN_pin  | 9          | Sensor Power Enable       | Output    | High = sensors powered        |
| STCLK_pin         | 14         | Shift Register Clock      | Output    | For sensor multiplexing       |
| SRDA_pin          | 13         | Shift Register Data       | Output    |                               |
| SHCLK_pin         | (custom)   | Shift Register Shift Clk  | Output    | Sometimes unused              |
| Tx_Data_pin       | 16         | Serial TX (to master)     | Output    | UART/RS232 option             |
| Rx_Data_pin       | 17         | Serial RX (from master)   | Input     |                               |
| SDA_pin           | 20         | I2C Data                  | I/O       | EEPROM, external master       |
| SCL_pin           | 21         | I2C Clock                 | I/O       |                               |
| SD Card Pins      | 2–7, 8     | SDIO/SPI                  | I/O       | CLK, CMD, D0–D3, Detect, etc. |
| PSRAM Pins        | 22–25      | SPI to PSRAM              | I/O       | CS, SCK, MOSI, MISO           |

> See `include/function.h` for the definitive, up-to-date list.

---

## Sensor Interface

- **Supported Sensors:**  
  - EGP (Electro-Galvanic Probe)  
  - MFL (Magnetic Flux Leakage)
- **Channels:** Up to 5 (expandable)
- **Connection:** SPI, with shift register multiplexing for addressable expansion
- **Power Control:** Dedicated enable pin for low-power operation

---

## Storage

- **PSRAM:**  
  - 8MB external, high-speed SPI RAM  
  - Used as a circular buffer for high-rate data prior to SD logging
- **SD Card:**  
  - Standard SD/SDHC up to 128GB+  
  - SDIO or SPI interface  
  - Card detect pin for hot-swap safety

---

## Visual & Status Feedback

- **NeoPixel LED:**  
  - Single RGB LED for status (idle, logging, error)
- **Watchdog:**  
  - Hardware watchdog with multi-core health signaling

---

## Expansion & Debug

- **Serial (UART):**  
  - Full debug and live data streaming support
- **I2C Master/Slave:**  
  - Configuration and data access from host controller
- **EEPROM:**  
  - Stores persistent IDs, calibration, and user settings

---

## Example Wiring Table

| Peripheral         | Board Pin(s) | Signal Name       | Notes                        |
|--------------------|--------------|-------------------|------------------------------|
| SD Card CLK        | 2            | GP2               | SDIO/SPI clock               |
| SD Card CMD        | 3            | GP3               | SDIO/SPI command             |
| SD Card D0         | 4            | GP4               | SDIO/SPI data                |
| SD Card Detect     | 8/9          | GP8/GP9           | Card present switch          |
| PSRAM CS           | 22           | GP22              | Chip select                  |
| PSRAM SCK          | 23           | GP23              | SPI clock                    |
| PSRAM MOSI         | 24           | GP24              | SPI MOSI                     |
| PSRAM MISO         | 25           | GP25              | SPI MISO                     |
| NeoPixel           | 15           | GP15              | Status LED                   |
| Sensors SPI        | (see code)   | SPI1              | Sensor array                 |
| Shift Register     | 13, 14       | SRDA, STCLK       | Multiplex sensor channels    |

---

## Design Notes

- **Power:** All digital I/O is 3.3V; use logic level shifters if interfacing with 5V sensors.
- **Hot-swappable SD:** Card detect and power switch pins allow safe insertion/removal during operation.
- **Robustness:** Watchdog and explicit error signaling (LED, serial) for reliability in field deployments.

---

## Board Photos & Schematics

> *(Insert board photos and annotated schematic diagrams here for visual reference.)*

---

**Next Steps:**  
- See [Firmware-Architecture.md](Firmware-Architecture.md) for a deep-dive into software structure and operation.
- For wiring help, refer to `include/function.h` and your board’s schematic.

---