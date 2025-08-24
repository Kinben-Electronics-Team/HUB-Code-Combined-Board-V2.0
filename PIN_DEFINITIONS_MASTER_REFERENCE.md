# CORRECTED PIN DEFINITIONS
## HUB System V1.0 - Verified Working Pin Mappings
### Updated: 2025-08-24

---

## MASTER BOARD PINS

### Digital I/O Pins (RP2040 GPIO)

| GPIO | Pin Name | Direction | Function | Init State | Notes |
|------|----------|-----------|----------|------------|-------|
| 0 | DATA_TX_PIN | OUTPUT | TX to CSB | - | Serial TX (not digital I/O) |
| 1 | DATA_RX_PIN | INPUT | RX from CSB | - | Serial RX (not digital I/O) |
| 2 | TX_EN_PIN | OUTPUT | Enable TX to CSB | LOW | Disabled by default |
| 3 | MODE_PIN | INPUT | Mode selection from CSB | INPUT_PULLUP | Digital signal |
| 4 | TRIG_PIN | INPUT | Trigger from CSB | INPUT_PULLUP | Digital signal |
| 6 | IOEX_SDA_PIN | I2C | IO Expander data | - | Wire1 SDA |
| 7 | IOEX_SCL_PIN | I2C | IO Expander clock | - | Wire1 SCL |
| 8 | SLOT_TX_PIN | OUTPUT | TX to Slots | - | Serial1 TX |
| 9 | SLOT_RX_PIN | INPUT | RX from Slots | - | Serial1 RX |
| 10 | SENSOR_SUPPLY_SEL0 | OUTPUT | Sensor power select | LOW | Always LOW for control |
| 11 | MUX_C2_PIN | OUTPUT | USB MUX bit 2 | LOW | Channel select |
| 12 | MUX_C1_PIN | OUTPUT | USB MUX bit 1 | LOW | Channel select |
| 13 | MUX_C0_PIN | OUTPUT | USB MUX bit 0 | LOW | Channel select |
| 14 | MUX_EN_PIN | OUTPUT | USB MUX enable | HIGH | HIGH = disabled |
| 15 | SENSOR_SUPPLY_SEL1 | OUTPUT | 5V buck enable | LOW | LOW = 3.3V, HIGH = 5V |
| 16 | IOEX_INT_PIN | INPUT | IO Expander interrupt | INPUT_PULLUP | |
| 17 | IOEX_RST_PIN | OUTPUT | IO Expander reset | HIGH | HIGH = not reset |
| 18 | SLOT_MODE_PIN | OUTPUT | Slot mode signal | LOW | Common to all slots |
| 19 | SLOT_TRIG_PIN | OUTPUT | Trigger to Slots | LOW | Common to all slots |
| 20 | SLOT_I2C_SDA_PIN | I2C | Slot I2C data | - | Wire SDA |
| 21 | SLOT_I2C_SCL_PIN | I2C | Slot I2C clock | - | Wire SCL |
| 22 | LED_PIN | OUTPUT | NeoPixel LED | LOW | WS2812B control |
| 28 | SDCARD_HUB_DET_PIN | OUTPUT | SD card detect | HIGH | HIGH = no card |
| 29 | USB_RPBUS_PWR_PIN | INPUT | USB hub power monitor | INPUT | Monitor USB power |

### IO Expander Pin Mappings (TCAL9539 at I2C 0x74)

| IOEX Pin | Function | Slot | Init State | Notes |
|----------|----------|------|------------|-------|
| **Power Control Pins** |
| P00 | Slot5 Power Enable | 5 | HIGH (OFF) | LOW = ON, HIGH = OFF |
| P03 | Slot4 Power Enable | 4 | HIGH (OFF) | LOW = ON, HIGH = OFF |
| P06 | Slot3 Power Enable | 3 | HIGH (OFF) | LOW = ON, HIGH = OFF |
| P09 | Slot2 Power Enable | 2 | HIGH (OFF) | LOW = ON, HIGH = OFF |
| P15 | Slot1 Power Enable | 1 | HIGH (OFF) | LOW = ON, HIGH = OFF |
| **Boot Control Pins** |
| P02 | Slot5 Boot Control | 5 | HIGH (Normal) | LOW = BOOTSEL mode |
| P05 | Slot4 Boot Control | 4 | HIGH (Normal) | LOW = BOOTSEL mode |
| P08 | Slot3 Boot Control | 3 | HIGH (Normal) | LOW = BOOTSEL mode |
| P11 | Slot2 Boot Control | 2 | HIGH (Normal) | LOW = BOOTSEL mode |
| P14 | Slot1 Boot Control | 1 | HIGH (Normal) | LOW = BOOTSEL mode |
| **I2C/Serial Selection Pins** |
| P01 | Slot5 Selection | 5 | HIGH (Not Selected) | LOW = Selected |
| P04 | Slot4 Selection | 4 | HIGH (Not Selected) | LOW = Selected |
| P07 | Slot3 Selection | 3 | HIGH (Not Selected) | LOW = Selected |
| P10 | Slot2 Selection | 2 | HIGH (Not Selected) | LOW = Selected |
| P13 | Slot1 Selection | 1 | HIGH (Not Selected) | LOW = Selected |

### Sensor Power Control Truth Table

| SENSOR_SUPPLY_SEL0 | SENSOR_SUPPLY_SEL1 | Output |
|--------------------|--------------------|---------| 
| HIGH | HIGH | Hi-Z (Off) |
| LOW | HIGH | 5V (EGP Mode) |
| LOW | LOW | 3.3V (MFL Mode) |

---

## SLOT BOARD PINS

### Digital I/O Pins (RP2040 GPIO)

| GPIO | Pin Name | Direction | Function | Init State | Notes |
|------|----------|-----------|----------|------------|-------|
| 1 | SD_SW_PIN | OUTPUT | SD card switch | LOW | LOW = disconnected, HIGH = connected |
| 2 | SD_CLK_PIN | SD | SD card clock | - | SD interface |
| 3 | SD_CMD_PIN | SD | SD card command | - | SD interface |
| 4 | SD_D0_PIN | SD | SD card data 0 | - | SD interface |
| 5 | SD_D1_PIN | SD | SD card data 1 | - | SD interface |
| 6 | SD_D2_PIN | SD | SD card data 2 | - | SD interface |
| 7 | SD_D3_PIN | SD | SD card data 3 | - | SD interface |
| 8 | SD_VCC_EN_PIN | OUTPUT | SD card power | LOW (OFF) | Controls SD card power |
| 9 | SENSOR_PS_EN_PIN | OUTPUT | Sensor power | LOW (OFF) | Sensor power switch |
| 10 | SPI_SCK_PIN | SPI | SPI clock | - | SPI1 |
| 11 | SPI_MOSI_PIN | SPI | SPI MOSI | - | SPI1 |
| 12 | SPI_MISO_PIN | SPI | SPI MISO | - | SPI1 |
| 13 | SRDA_PIN | OUTPUT | Shift register data | LOW | Serial data to SR |
| 14 | STCLK_PIN | OUTPUT | Shift register clock | LOW | Storage clock for SR |
| 15 | LED_PIN | OUTPUT | NeoPixel LED | LOW | WS2812B control |
| 16 | TX_DATA_PIN | OUTPUT | TX to Master | - | Serial1 TX |
| 17 | RX_DATA_PIN | INPUT | RX from Master | - | Serial1 RX |
| 18 | MODE_PIN | INPUT | Mode from Master | INPUT_PULLUP | Digital signal |
| 19 | I2C_IO_PIN | INPUT | Selection from Master | INPUT_PULLUP | LOW = Selected |
| 20 | SDA_PIN | I2C | I2C data | - | Wire SDA |
| 21 | SCL_PIN | I2C | I2C clock | - | Wire SCL |
| 26 | TRIG_PIN | INPUT | Trigger from Master | INPUT_PULLUP | Digital signal |

---

## IMPORTANT NOTES

### 1. IO Expander Configuration
- I2C Address: 0x74 (TCAL9539)
- Connected via Wire1 on Master (GPIO 6/7)
- All 16 pins configured as OUTPUT on master side

### 2. I2C_IO_PIN Direction Clarification
- **On Master (IO Expander)**: Configured as OUTPUT to drive selection signal
- **On Slot (GPIO19)**: Configured as INPUT_PULLUP to read selection state
- When Master pulls LOW via IO Expander, the selected slot detects LOW on its I2C_IO_PIN

### 3. Power Control Logic
- Slot power pins are ACTIVE LOW (LOW = ON, HIGH = OFF)
- Boot pins: Hold LOW during power-on to enter BOOTSEL mode
- Sensor supply: Keep SENSOR_SUPPLY_SEL0 always LOW for proper operation

### 4. USB MUX Channels
- Channel 0: MUX disabled (Master USB active)
- Channels 1-5: Map to Slots 1-5 respectively
- MUX_EN_PIN: HIGH = disabled, LOW = enabled

### 5. Critical Safety Notes
- All slots powered OFF by default
- USB MUX disabled by default
- Sensor power in 3.3V mode by default
- SD cards disconnected by default

---

## CORRECTIONS FROM ORIGINAL DOCUMENTATION

1. **Fixed IO Expander Pin Mappings**: Using verified working mappings from old code
2. **Standardized Naming**: Using old code naming conventions (SRDA_PIN, STCLK_PIN, etc.)
3. **Clarified I2C_IO_PIN**: Documented as INPUT on slot side with pull-up
4. **Updated Sensor Supply Pins**: Using SENSOR_SUPPLY_SEL0/SEL1 naming
5. **USB MUX Pins**: Using MUX_xxx naming convention

---

## VERIFICATION STATUS
- [x] Pin mappings verified against working old code
- [x] IO Expander pins corrected to avoid conflicts
- [x] Pin directions clarified
- [x] Naming conventions standardized
- [x] Safe initialization states defined
- [ ] Hardware testing pending