#ifndef MyI2CDeviceKT_h
#define MyI2CDeviceKT_h

#include <Arduino.h>
#include <Wire.h>
class MyI2CDevice
{
public:
    MyI2CDevice(TwoWire *wirePort, uint8_t deviceAddress, Stream *userStream = nullptr); // Constructor with default stream
    /**
     * @brief Function to check if the device is present at the specified I2C address
     * @return true if the device is present, false otherwise
     */
    bool check_comm();

    bool writeRegister(uint8_t reg, uint8_t value); // Function to write to a register
    uint8_t readRegister(uint8_t reg);
    void setpinMode(int pin, int pMode = INPUT); // Set PinMode as INPUT or OUTPUT
    bool digitalRead(int pin);
    void digitalWrite(int pin, bool pState = LOW);

private:
    uint16_t portModeConfig, UPdownConfig, OutputStat;
    TwoWire *_wirePort;                                 // Pointer to the I2C port
    uint8_t _deviceAddress;                             // I2C address of the device
    Stream *SerialStream;                               // Pointer to the stream for debugging or communication// Helper function for debug printing
    static const uint8_t CONFIG_REG_PORT_0 = 0x06;      // Configuration register for port 0
    static const uint8_t CONFIG_REG_PORT_1 = 0x07;      // Configuration register for port 1
    static const uint8_t PULL_SEL_REG_PORT_0 = 0x0E;    // Pull-up/down selection register for port 0
    static const uint8_t PULL_SEL_REG_PORT_1 = 0x0F;    // Pull-up/down selection register for port 1
    static const uint8_t PULL_EN_REG_PORT_0 = 0x0C;     // Pull-up/down enable register for port 0
    static const uint8_t PULL_EN_REG_PORT_1 = 0x0D;     // Pull-up/down enable register for port 1
    static const uint8_t INPUT_REG_PORT_0 = 0x00;       // Input register for port 0
    static const uint8_t INPUT_REG_PORT_1 = 0x01;       // Input register for port 1
    static const uint8_t OUTPUT_REG_PORT_0 = 0x02;      // Output register for port 0
    static const uint8_t OUTPUT_REG_PORT_1 = 0x03;      // Output register for port 1
    static const uint8_t OUTPUT_TYPE_REG_PORT_0 = 0x04; // Output type register for port 0
    static const uint8_t OUTPUT_TYPE_REG_PORT_1 = 0x05; // Output type register for port 1
};
#endif