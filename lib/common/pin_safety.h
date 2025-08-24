#ifndef PIN_SAFETY_H
#define PIN_SAFETY_H

#include <Arduino.h>

// Master critical pins
#ifdef BUILD_MASTER
#define CSB_TX_PIN 0
#define CSB_RX_PIN 1
#define TX_EN_PIN 2
#define SLOT_TX_PIN 8
#define SLOT_RX_PIN 9
#define SLOT_MODE_PIN 18
#define SLOT_TRIG_PIN 19
#define IOEX_SDA_PIN 6
#define IOEX_SCL_PIN 7
#define IOEX_RST_PIN 17
#define USB_MUX_EN_PIN 14
#define LED_PIN 22

// IO Expander addresses and pins
#define IOEX_ADDR 0x74
#define IOEX_PORT0 0x02
#define IOEX_PORT1 0x03
#define IOEX_CONFIG0 0x06
#define IOEX_CONFIG1 0x07

// Slot power control pins on IO Expander (active LOW)
#define SLOT1_POWER_PIN 0  // Port 0, Pin 0
#define SLOT2_POWER_PIN 1  // Port 0, Pin 1
#define SLOT3_POWER_PIN 2  // Port 0, Pin 2
#define SLOT4_POWER_PIN 3  // Port 0, Pin 3
#define SLOT5_POWER_PIN 4  // Port 0, Pin 4

// USB MUX control
#define MUX_ADDR 0x70
#define MUX_CHANNEL_MASK 0x3F  // Channels 2-6 for slots 1-5

inline void initMasterPinsSafe() {
    // Set all control pins to safe states
    pinMode(TX_EN_PIN, OUTPUT);
    digitalWrite(TX_EN_PIN, LOW);  // Disable transmitter initially
    
    pinMode(SLOT_MODE_PIN, OUTPUT);
    digitalWrite(SLOT_MODE_PIN, LOW);  // Normal mode
    
    pinMode(SLOT_TRIG_PIN, OUTPUT);
    digitalWrite(SLOT_TRIG_PIN, LOW);  // No trigger
    
    pinMode(USB_MUX_EN_PIN, OUTPUT);
    digitalWrite(USB_MUX_EN_PIN, LOW);  // USB MUX disabled
    
    pinMode(IOEX_RST_PIN, OUTPUT);
    digitalWrite(IOEX_RST_PIN, HIGH);  // IO Expander not in reset
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);  // LED off
}

#endif // BUILD_MASTER

// Slot critical pins
#ifdef BUILD_SLOT
#define TX_DATA_PIN 16
#define RX_DATA_PIN 17
#define MODE_PIN 18
#define I2C_IO_PIN 19
#define SDA_PIN 20
#define SCL_PIN 21
#define TRIG_PIN 26
#define LED_PIN 15
#define SD_SW_PIN 1
#define SD_VCC_EN_PIN 8
#define SENSOR_PS_EN_PIN 9

inline void initSlotPinsSafe() {
    // Set all control pins to safe states
    pinMode(MODE_PIN, INPUT);  // Read mode from master
    
    pinMode(I2C_IO_PIN, INPUT);  // I2C address pin
    
    pinMode(TRIG_PIN, INPUT);  // Trigger input from master
    
    pinMode(SD_SW_PIN, OUTPUT);
    digitalWrite(SD_SW_PIN, LOW);  // SD card disconnected
    
    pinMode(SD_VCC_EN_PIN, OUTPUT);
    digitalWrite(SD_VCC_EN_PIN, LOW);  // SD card powered off
    
    pinMode(SENSOR_PS_EN_PIN, OUTPUT);
    digitalWrite(SENSOR_PS_EN_PIN, LOW);  // Sensors powered off
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);  // LED off
}

#endif // BUILD_SLOT

// LED status colors (using built-in LED for simplicity)
enum LEDStatus {
    LED_OFF = 0,
    LED_BLUE,      // Idle/Initialized
    LED_GREEN,     // Communication success
    LED_YELLOW,    // Retrying
    LED_RED,       // Error/Timeout
    LED_PURPLE     // Processing command
};

// Forward declaration - implemented in main.cpp
void setLEDStatus(LEDStatus status);

#endif // PIN_SAFETY_H