#ifndef HUB_MASTER_h
#define HUB_MASTER_h
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Wire.h>
// #include <shift_register/LVC595.h>
// #include <High_Speed_ADC/high_speed_adc.h>
#include "EasyTransfer.h"
#include "EEPROM.h"
#include <IOEX/MyI2CDeviceKT.h>
#include <common_config.h>

/* Slot serial pins*/
#define SLOT_RX_pin 9        // receive serial data from slot
#define SLOT_TX_pin 8        // transmit and trigger pin to slot
#define SLOT_TRIG_pin 19     // transmit serial data to slot
#define SLOT_MODE_pin 18     // slot board mode pin
#define SLOT_Serial Serial2  // Serial port for slot communication
#define SLOT_BR COM_BAUDRATE // Baud rate for slot communication

/*HUB and  MUX pins*/
#define MUX_C0_pin 13
#define MUX_C1_pin 12
#define MUX_C2_pin 11
#define MUX_EN_pin 14
const uint8_t hubChannels[] = {2, 3, 4, 5, 6}; // MUX channels for respective hub slots

/*Card reader pins */
#define USB_RPBUS_PWR_pin 29  // input  monitor rpbus port power
#define SDCARD_HUB_DET_pin 28 // drive low to detect sd card by card reader

// pins from CSB
#define DATA_TX_pin 0
#define DATA_RX_pin 1
#define TRIG_pin 4
#define MODE_pin 3
#define CSB_Serial Serial1  // Serial port for CSB communication
#define CSB_BR COM_BAUDRATE // Baud rate for CSB communication
#define TX_EN_pin 2         // drive high to enable data Tx to CSB

/*IOEX pins */

#define IOEX_SCL_pin 7  // I2C clock pin for IO expander
#define IOEX_SDA_pin 6  // I2C data pin for IO expander
#define IOEX_Wire Wire1 // I2C bus for IO expander
#define IOEX_ADD 0x74   // I2C address of TCAL9539 (change A0/A1 pins accordingly)
#define IOEX_INT_pin 16 // interrupt pin for IO expander
#define IOEX_RST_pin 17 // Reset pin for IO expander

/*IOEX pin mapping */
const uint8_t IOEX_ioPins[] = {13, 10, 7, 4, 1};   // IOEX pins for I2C selection
const uint8_t IOEX_bootPins[] = {14, 11, 8, 5, 2}; // pins to boot respective hub boards
const uint8_t IOEX_pwrPins[] = {15, 9, 6, 3, 0};   // pins to power respective hub boards, set low to power on the slot

#define TX_EN 1
#define TX_DISEN !TX_EN

#define ACQ_MODE 1           // mode pin status for acquisition
#define OTHER_MODE !ACQ_MODE // mode pin status for no acquisition

// Master I2C pins
#define I2C_CLOCK_SPEED 100000 // 3500000
#define I2C_SDA_pin 20         // I2C data pin
#define I2C_SCL_pin 21         // I2C clock pin
#define I2C_Wire Wire          // I2C bus for master communication

// LED Colours

/**
 * SENSOR_SUPPLY_SEL0  SENSOR_SUPPLY_SEL2     OUTPUT
 * HIGH               HIGH             Hi-z
 * LOW                HIGH             5V
 * LOW                LOW              3.3V
 */
#define SENSOR_SUPPLY_SEL0 10 // pin to select sensor supply voltage keep always low
#define SENSOR_SUPPLY_SEL1 15 // pull high to enable EGP power (5V) low for 3.3V
#define LED_pin 22            // GPIO pin for LED DIN
#define NUM_LEDS 1            // Number of LEDs in the strip
// Adafruit_NeoPixel pixel(1, 18, NEO_GRB + NEO_KHZ800);

#define BAUDRATE 152000 // usb baud rate for communication

#define EEPROM_SIZE 512 // EEPROM size in bytes

#define Broadcast_ADDR 0x00 // I2C address for broadcast

/*I2C commands for hub MCUs*/
#define I2C_CMD_SD_CON 0x04    // command to connect sd card to card reader
#define I2C_CMD_SD_DISCON 0x05 // command to disconect sd card from sd card reader ic
#define SID_REG 0x06
#define HID_REG 0x07

// Bidirectional communication query commands
#define I2C_CMD_GET_STATUS 0x10    // Query slot status and health
#define I2C_CMD_GET_SENSORS 0x11   // Query sensor readings  
#define I2C_CMD_GET_CONFIG 0x12    // Query slot configuration
#define I2C_CMD_PING 0x13          // Ping slot for connectivity test

enum sensorpwrsel
{
  pwr_off = 0, // sensor power off
  pwr_3v3,     // sensor power 3.3V
  pwr_5v       // sensor power 5V
};
// Serial commands
typedef enum
{
  /*commands to bootup respective hub board mcu*/
  Hub1_boot = 0,
  Hub2_boot,
  Hub3_boot,
  Hub4_boot,
  Hub5_boot,
  Hub6_boot,
  Hub7_boot,
  Hub8_boot,
  /*commands to connect respective hub board sd card with card reader*/
  Hub1_SD_conn,
  Hub2_SD_conn,
  Hub3_SD_conn,
  Hub4_SD_conn,
  Hub5_SD_conn,
  Hub6_SD_conn,
  Hub7_SD_conn,
  Hub8_SD_conn,
  /*command to disconnect sd card from sd card reader*/
  disconnect_sd,
  /*command to select any hub board*/
  Hub1_sel,
  Hub2_sel,
  Hub3_sel,
  Hub4_sel,
  Hub5_sel,
  Hub6_sel,
  Hub7_sel,
  Hub8_sel,
  MUX_disconnect,

  /*command to set number of egp sensor*/
  hub1_egp_set,
  hub2_egp_set,
  hub3_egp_set,
  hub4_egp_set,
  hub5_egp_set,
  hub6_egp_set,
  hub7_egp_set,
  hub8_egp_set,
  all_hub_egp_set,

  /*HID SID configuration command*/
  config_hid,
  config_sid,

  dummy_log_start,
  dummy_log_end

} SerialCMD;

typedef enum
{
  SLOT1 = 0,
  SLOT2,
  SLOT3,
  SLOT4,
  SLOT5,

} slot_t;

// select hub to communicate with

struct configData
{
  uint8_t hid;         // hub id
  uint8_t ns;          // number of sensors connected to the hub board
  uint8_t mag_axis;    // magnetic axis of the sensor
  uint8_t ang_axis;    // angular axis of the sensor
  uint8_t sensor_Type; // sensor type EGP/MFL
};

struct MasterRXdata
{
  uint8_t setBoardID = 0;
  // uint8_t data[100];
};


void setSensorPwr(sensorpwrsel sel);

// Function prototypes for main.cpp
void setup_master();
void loop_master();

#endif