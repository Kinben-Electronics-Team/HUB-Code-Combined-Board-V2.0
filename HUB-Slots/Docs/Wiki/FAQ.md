# ❓ FAQ

**Q: How do I add a new sensor type?**  
A: Extend the `Sensors` class and update acquisition logic. See [Developer Guide](Developer-Guide.md).

**Q: How do I change the SD file size limit?**  
A: Update `FILE_SIZE_LIMIT` in `include/function.h`.

**Q: What does each LED color mean?**  
A:  
- Blue = Idle/Ready  
- Green = Logging  
- Red = Error/Stopped

**Q: How do I reset device IDs (SID/HID)?**  
A: Use I2C/serial commands or update via EEPROM.

**Q: Can I use this firmware on a regular Raspberry Pi Pico?**  
A: Yes, if you add the required peripherals (PSRAM, SD, NeoPixel) and adjust pin mapping.

**Q: Can I live stream sensor data?**  
A: Yes, via serial or I2C in service mode.

---

See [API-Reference.md](API-Reference.md) for more technical details.