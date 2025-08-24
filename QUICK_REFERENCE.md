# HUB MASTER QUICK REFERENCE CARD

## 🚀 Upload & Run
```bash
# Upload to Master (COM13):
pio run -e master -t upload

# Monitor serial output:
pio device monitor -p COM13 -b 115200
```

## 📋 Command Reference

### USB MUX Control
- `0` - Disable MUX (no slot USB active)
- `1` - Switch to Slot 1 (MUX Ch2)
- `2` - Switch to Slot 2 (MUX Ch3)
- `3` - Switch to Slot 3 (MUX Ch4)
- `4` - Switch to Slot 4 (MUX Ch5)
- `5` - Switch to Slot 5 (MUX Ch6)

### Power Control
- `P1` to `P5` - Power ON Slot 1-5
- `O1` to `O5` - Power OFF Slot 1-5
- `PA` - Power ON all slots
- `OA` - Power OFF all slots

### Boot Control
- `B1` to `B5` - Boot Slot 1-5 to BOOTSEL mode
- `R1` to `R5` - Reset Slot 1-5

### Communication
- `S0` - Select ALL slots for broadcast
- `S1` to `S5` - Select specific slot for comms

### System
- `?` - Show menu
- `I` - Show system status

---

## 🎯 Programming a Slot - Step by Step

### Example: Program Slot 1
1. **Power on the slot**: `P1`
2. **Boot to BOOTSEL**: `B1`
3. **Switch USB MUX**: `1`
4. **Check Windows**: Should see RPI-RP2 drive
5. **Upload firmware**: Drag .uf2 file or use PlatformIO
6. **Reset slot**: `R1`
7. **Disable MUX**: `0` (optional, to disconnect USB)

---

## 💡 LED Status Colors (NeoPixel)
- 🔵 **Blue** - System initializing
- 🟢 **Green** - System ready
- 🔴 **Red** - IO Expander error
- 🟣 **Cyan breathing** - System idle/running

---

## ⚠️ Important Notes

1. **Master USB**: Always available on COM13 (doesn't go through MUX)
2. **Slot USB**: Only accessible when MUX is enabled for that slot
3. **MUX Channels**: 
   - Channel 1 = Hub (not used)
   - Channel 2-6 = Slots 1-5
4. **Power**: Slots are powered OFF by default for safety
5. **BOOTSEL**: Hold BOOT low during power-on sequence

---

## 🔧 Troubleshooting

### IO Expander Not Found
- Check Wire1 connections (GPIO 6/7)
- Verify I2C address 0x74
- Check IOEX_RST_PIN (GPIO 17)

### Slot Won't Enter BOOTSEL
- Ensure slot is powered off first
- Check boot pin mapping in IO Expander
- Try longer delays in boot sequence

### USB MUX Not Working
- Verify MUX_EN_PIN is LOW when active
- Check channel select bits (C0, C1, C2)
- Ensure Windows has time to enumerate

---

## 📊 Pin Mappings Reference

### IO Expander (TCAL9539)
| Function | Slot 1 | Slot 2 | Slot 3 | Slot 4 | Slot 5 |
|----------|--------|--------|--------|--------|--------|
| Power    | P15    | P09    | P06    | P03    | P00    |
| Boot     | P14    | P11    | P08    | P05    | P02    |
| Select   | P13    | P10    | P07    | P04    | P01    |

*Power pins are ACTIVE LOW (LOW = ON, HIGH = OFF)*