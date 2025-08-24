#ifndef FUNCTION_H
#define FUNCTION_H

#include "EEPROM.h"
#include "hardware/structs/scb.h"
#include <Adafruit_NeoPixel.h>
#include "psram_spi.h"
#include <Arduino.h>
#include <MFLSensorHead/LDC1101.h>
#include <SdFat.h>
#include "hardware/irq.h"
#include <Wire.h>
#include <EasyTransfer.h>
#include <common_config.h>

#define BAUDRATE 115200 // serial port baudrate

#define LOG_BUFF_SIZE 32 * 1024             // number of bytes to be logged into sd card at once
#define FILE_SIZE_LIMIT (128 * 1024 * 1024) // size of each file

#define NUM_ADC_BYTES 3  // number of ADC bytes
#define NUM_COUNT_BYTE 8 // number of sample count bytes

#if defined(EGP)
#define NUM_START_BYTES 16     // number of start bytes
#define SAMPLE_BUFFER_SIZE 112 //(MAX_SAMPLE_SIZE + NUM_ADC_BYTES + NUM_START_BYTES + NUM_COUNT_BYTE+1) // buffer size of 1 sample
#define SPI_FREQ 2500000       // SPI frequency for EGP sensor
#elif defined(MFL)
#define NUM_START_BYTES 12    // number of start bytes
#define SAMPLE_BUFFER_SIZE 64 //(MAX_SAMPLE_SIZE + NUM_ADC_BYTES + NUM_START_BYTES + NUM_COUNT_BYTE+1) // buffer size of 1 sample
#define SPI_FREQ 10000000     // SPI frequency for MFL sensor
#else
#error "Please define either EGP or MFL"
#endif

#define SRAM_SIZE (8 * 1024 * 1024)                                           // Memory size of Sram (8Mbytes)
#define BYTES_READ_AT_ONCE 15 * SAMPLE_BUFFER_SIZE                            // 2560//128*20//1920                                               // Max number of bytes to read at once from PSRAM due to time constraint
#define MAX_WRITE_ADD ((SRAM_SIZE / SAMPLE_BUFFER_SIZE) * SAMPLE_BUFFER_SIZE) // this is the max address value to which both cores will write and read data
#define PSRAM_CHUNK_SIZE 16
#define WDT_RESET_TIME 3000

#define TRIG_PRIORITY 0x00
#define WIRE_PRIORITY 0x01

#define LED_pin 15 // general purpose led for debugging

/*Pins for I2C communication with master*/
#define I2C_IO_pin 19
#define SDA_pin 20
#define SCL_pin 21
#define MSTR_Wire Wire // I2C instance for communication with master
#define I2C_SLAVE_ADDRESS 0x01
#define I2C_CLOCK_SPEED 100000 // 3500000

/*I2C commands for hub MCUs*/
#define I2C_CMD_SD_CON 0x04    // command to connect sd card to card reader
#define I2C_CMD_SD_DISCON 0x05 // command to disconect sd card from sd card reader ic
#define SID_REG 0x06
#define HID_REG 0x07

/*CSB pin define*/
#define MODE_pin 18
#define TRIG_pin 26 // trigger pin for acquisition
#define Tx_Data_pin 16
#define Rx_Data_pin 17
#define MSTR_Serial Serial1 // CSB pin for serial communication with master

/*Sensor pins */
#define SENSOR_PS_EN_pin 9 // sensor power enable pin
#define STCLK_pin 14       // sensor shift register storage clock pin
#define SRDA_pin 13        // sensor shift register data pin
#define SHCLK_pin NO_pin   // sensor shift register shift clock pin
#define SENSOR_SPI SPI1    // sensor spi instance

/*sensor spi pins*/
#define SPI_SCK_PIN 10
#define SPI_MOSI_PIN 11
#define SPI_MISO_PIN 12

#define SPI_CS_PIN NO_pin

/*SD card pins*/
#define SD_SW_pin 1  // drive high to connenct sd card to card reader IC
#define SD_CLK_pin 2 // sd card clock pin
#define SD_CMD_pin 3 // sd card CMD pin
#define SD_D0_pin 4  // sd card data0 pin

#define SD_VCC_EN_pin 8 // sd card power enable pin

/*FIFO CMD*/
#define LOG_DONE 1         // data logged to sd card
#define CLOSE_FILE 2       // cmd to close file, complete logging
#define UNMOUNT_SD 3       // cmd to unmount sd card
#define LOG_START 4        // cmd to initiate data logging (open a file and wait for data)
#define MOUNT_SD 5         // cmd to re mount sd card
#define DATA_READY 6       // a sample is written to psram, psram available to read
#define TEST_SD 7          // cmd to test sd card
#define CLOSE_FILE_ACK 8   // ack flag for cmd close file command received
#define UPDATE_NUM_EGP 9   // cmd to update the number of egp
#define UPDATE_SID 10      // cmd to update SID
#define UPDATE_HID 11      // cmd to update HID
#define HEALTH_FLAG 12     // cmd to reset watchdog timer
#define RESET_WDT_EVENT 23 // cmd to reset watchdog timer

#define SIDadd 0
#define HIDadd 1

void receiveEventCallback(int numBytes);
void requestEventCallback();
void trigger();

class Logging
{
public:
    // constructor
    Logging(FsFile *file,  psram_spi_inst_t *psramSPI);

    /*initialise sd card */
    void sd_card_init();

    /**
     * @brief create new file and open it.
     * new filename is also generated after creating a file
     * @param isopen true if any a file is already open
     */
    void create_file(bool isopen);

    /**
     * @brief check the fifo commands from other core and take action
     * @param fifo_val fifo cmd or data
     */
    void check_fifo(uint32_t fifo_val);

    /**
     * analyse number of bytes to be read and log_buff
     * when buffer is full, data is logged to sd card
     */
    void psram_read_data();

    /**
     * @brief read data from psram in chunks
     * @param read_bytes number of bytes to be read from psram
     */
    void psram_collect_sample(unsigned int read_bytes);

    /**
     * @brief log buffer to sd card
     * @param buff buffer to be logged
     * @param buff_size number of bytes to be written
     */
    void log_data(uint8_t *buff, uint32_t buff_size);
    /**
     * @brief read data from psram byte by byte
     * @param num_bytes number of bytes to be read
     */
    void psram_sample_collect_bytes(unsigned int num_bytes);

private:
    psram_spi_inst_t *psram_spi;
    FsFile *f;
    SdFs sd;
    char filename[32];
    uint32_t fileSize = 0; // size of current file

    uint8_t log_buff[LOG_BUFF_SIZE]; // buffer to log data
    unsigned int log_buff_index = 0;
    // uint16_t fileCount = 0;
    uint8_t Sample_size_to_read = SAMPLE_BUFFER_SIZE;                               //(8 * 14) + NUM_START_BYTES + NUM_COUNT_BYTE + NUM_ADC_BYTES; // number of bytes in a sample including ADC bytes for core1,initialsed for 3 egp
    uint32_t max_read_add = ((SRAM_SIZE / Sample_size_to_read) * Sample_size_to_read); // this is the max address value to which both cores will write and read data;
    uint32_t psram_bytes_read_add = 0;                                              // psram address upto which reading is done
    uint32_t psram_bytes_to_be_read = 0;                                            // number of remaining bytes that needs to read from psram
    unsigned long psram_sample_read_count = 0;                                      // number of samples read from psram by core1
    unsigned long sample_read_bytes = 0;                                            // number of bytes (which is not complete sample like 1 sample + 2 bytes) read from psram by core1
    unsigned long psram_cycle_read_count = 0;                                       // number of times psram is read completely

    /* generate  a file name. filename format - HIDxxSIDxx_xxxx*/
    void getFilename();

    /*function to test sd card. it would be used in check fifo*/
    bool test_Sd_card(SerialUSB *_testSerial);

    /**
     * @brief update the number of sample and number of bytes of incomplete sample(if any) read from psram
     * @param read_bytes number of bytes read from psram
     */
    void update_sample_read_count(unsigned int read_bytes);

    // FRESULT log_error(uint8_t error);
};

class Acquire
{
public:
    uint64_t sc = 0; // sample count

    Acquire(psram_spi_inst_t *psramSPI, Sensors *sens, TwoWire *_wire = &Wire, SerialUSB *serial = &Serial);

    /* initialise all pins and peripherals of core0*/
    void begin();

    /**
     * @brief sensors including powering them
     * @param debugEn true enable printing sensor data false to disable
     * @param numEgp number of egp sensor
     * */
    void Sensor_begin(bool debugEN, uint8_t numEgp = 0);

    /*check mode pin */
    bool check_mode();

    /**
     * @brief update the number of EGP
     * @param numEGP number of EGP. pass 0 if no EGP sensor is used
     */
    void update_EGP_count(uint8_t numEGP = 0);

    /**
     * @brief get data from sensors and adc and write to psram
     * @param  startBytes buffer of start bytes to log
     * @param ADC_data array of adc data
     * */
    void get_data();

    /*function for wire request event. not a callback function*/
    void requestEvent();

    /**
     * @brief to test psram read/write time in chunk
     * @param num_bytes number of bytes to be read/write for testing
     * @param chunk_size number of bytes to be read/write at once
     * */
    void psram_test(unsigned int num_bytes, uint8_t chunk_size);

    /**
     * @brief the commands received by wire
     * @param cmd cmd to be executed
     * */
    void ExecuteWireCmd(uint8_t cmd, uint8_t data = 0);

private:
    Sensors *sensor;
    psram_spi_inst_t *psram_spi;
    TwoWire *_myWire;
    SerialUSB *_mySerial;
    unsigned long psram_sample_write_count = 0; // number of samples written in psram by core0
    unsigned long psram_cycle_write_count = 0;  // psram memory writting cycle
    uint32_t psram_bytes_write_add = 0;         // psram address upto which writting is done                                                               // number of egp sensor
    uint8_t Data_size = (8 * 14);               // number of sensor bytes, initialsed for 3 egp
    uint8_t Sample_size = SAMPLE_BUFFER_SIZE;   // NUM_START_BYTES + Data_size + NUM_COUNT_BYTE + NUM_ADC_BYTES; // number of bytes in a sample including ADC bytes

    uint8_t buffer[SAMPLE_BUFFER_SIZE];
    bool mode_stat = NOT_ACQ_MODE, adc_data_received = 0, sd_con_stat = 0;
    bool log_started;
};

// Function prototypes for main.cpp
void setup_slot();
void loop_slot();
void setup1_slot();
void loop1_slot();

#endif