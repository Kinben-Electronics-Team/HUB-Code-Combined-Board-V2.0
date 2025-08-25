#ifdef BUILD_SLOT
#include "function.h"

// Beta feature: Sample count debugging
bool beta_sample_debug = true; // Enable sample count debugging
uint64_t last_received_sample = 0;
unsigned long sample_receive_count = 0;

/*varible for psram*/
psram_spi_inst_t psram_spi = psram_spi_init(pio0, -1, false);

/*ISR variables */
volatile bool trig_status = 0;
uint8_t wireData[4];
volatile bool wire_received = 0;

EasyTransfer EscRX;

SPISettings SPIset(SPI_FREQ, MSBFIRST, SPI_MODE0);
ShiftRegister sr(2, SRDA_pin, STCLK_pin, -1);
Sensors Sens(&sr, SPIset, 2, &SENSOR_SPI);
Acquire core(&psram_spi, &Sens);

void setup_slot()
{
    set_sys_clock_khz(133000, true);
    if (rp2040.getResetReason() == 4)
    {
        rp2040.fifo.push_nb(RESET_WDT_EVENT); // reset watchdog timer
    }

    Serial.begin(BAUDRATE);

    MSTR_Serial.setRX(Rx_Data_pin);
    MSTR_Serial.setTX(Tx_Data_pin);
    MSTR_Serial.begin(COM_BAUDRATE); // begin serial communication with master

    core.begin();
    core.Sensor_begin(true, 0); // initialise sensors and power them on
    Serial.println("Sensors initialised");

    pinMode(SD_SW_pin, OUTPUT);
    digitalWrite(SD_SW_pin, LOW);                // CONNECT SD CARD TO uC
    
    Serial.println("Requesting SD card initialization...");
    rp2040.fifo.push_nb(MOUNT_SD);               // mount sd card

    // Wait for Core 1 to acknowledge SD initialization
    uint32_t timeout = millis() + 10000; // 10 second timeout
    bool sdInitialized = false;
    
    Serial.println("Waiting for SD initialization...");
    while (millis() < timeout && !sdInitialized) {
        if (rp2040.fifo.available()) {
            uint32_t response = rp2040.fifo.pop();
            if (response == MOUNT_SD_ACK) {
                sdInitialized = true;
                Serial.println("SD card initialized successfully!");
            } else {
                // Put other FIFO messages back for later processing
                rp2040.fifo.push_nb(response);
                Serial.printf("Got FIFO message: %lu (not SD ACK)\n", response);
            }
        }
        delay(10); // Small delay to prevent busy waiting
    }
    
    if (!sdInitialized) {
        Serial.println("ERROR: SD card initialization timeout!");
        Serial.println("Continuing with setup but SD may not work...");
    }

    EscRX.begin(details(core.sc), &MSTR_Serial); // begin EasyTransfer for receiving data from master
    rp2040.wdt_begin(WDT_RESET_TIME);            // start watchdog timer
    Serial.println("Setup complete");
}

void loop_slot()
{

    if (core.check_mode() == ACQ_MODE)
    {
        if (trig_status)
        {
            trig_status = 0;     // reset trigger flag
            EscRX.receiveData(); // receive data from master
            
            // CLEAN: Only track sample count for post-acquisition statistics
            if (beta_sample_debug && core.sc != last_received_sample) {
                sample_receive_count++;
                last_received_sample = core.sc;
            }
            
            core.get_data();     // get data from sensor and log into psram
        }
    }

    else
    {
        // Beta feature: Display sample count statistics (non-intrusive)
        static unsigned long last_stats_time = 0;
        if (beta_sample_debug && millis() - last_stats_time > 5000) { // Every 5 seconds
            if (sample_receive_count > 0) {
                Serial.printf("BETA Stats: Last sample ID: %llu, Total received: %lu\n", 
                             last_received_sample, sample_receive_count);
            }
            last_stats_time = millis();
        }

        if (wire_received)
        {
            Serial.println("Wire data received");
            wire_received = false;
            core.ExecuteWireCmd(wireData[0], wireData[1]);
            memset(wireData, 0, sizeof(wireData));
        }
#if defined(MFL)
        Sens.livereadMFL(X_CH_RESULT, true, core.getSDCardStatus());
#else
        Sens.liveReadEGP(X_CH_RESULT, true, core.getSDCardStatus());
#endif
        delay(100);
        rp2040.fifo.push_nb(HEALTH_FLAG); // push health flag to reset watchdog timer
    }
}

/***core1 varibles***/

FsFile file;

Logging l(&file, &psram_spi);

void setup1_slot()
{
    pinMode(SD_VCC_EN_pin, OUTPUT);
    digitalWrite(SD_VCC_EN_pin, HIGH); // power on sd card
    
    // Process any pending FIFO messages during setup
    delay(100); // Let Core 0 send MOUNT_SD
    
    // Check for MOUNT_SD during setup phase
    Serial.println("Core 1 setup checking for MOUNT_SD...");
    if (rp2040.fifo.available()) {
        uint32_t cmd = rp2040.fifo.pop();
        if (cmd == MOUNT_SD) {
            Serial.println("Processing early SD mount request");
            l.sd_card_init();
            rp2040.fifo.push_nb(MOUNT_SD_ACK);
            Serial.println("SD initialization complete, ACK sent");
        } else {
            // Put it back for loop processing
            rp2040.fifo.push_nb(cmd);
            Serial.printf("Got non-MOUNT_SD command in setup1: %lu\n", cmd);
        }
    } else {
        Serial.println("No FIFO messages in Core 1 setup");
    }
}

void loop1_slot()
{

    if (rp2040.fifo.available())
    {
        l.check_fifo(rp2040.fifo.pop()); // check multicore fifo value and execute required action
    }
}

/**
 * \brief interrupt callback function for Wire receive event
 * \param numBytes number of bytes received
 */
void receiveEventCallback(int numBytes)
{
    int i = 0;
    memset(wireData, 0, sizeof(wireData)); // reset wire buffer
    while (Wire.available())
    {
        if (i < 4)
        {
            wireData[i] = Wire.read();
            i++;
        }
        else
        {
            char trash = Wire.read();
        }
    }
    Serial.printf("Wire data received:%d \n", i);
    wire_received = 1;
}

/**
 * \brief Interrupt callback function for Wire request event
 */
void requestEventCallback()
{
    core.requestEvent();
}

/**
 * \brief Interrupt callback function for Interrupt on Trigger pin
 */
void trigger()
{
    trig_status = 1;
}

#endif // BUILD_SLOT