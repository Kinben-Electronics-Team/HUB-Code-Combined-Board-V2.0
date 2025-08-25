
#include "function.h"

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

void setup()
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

    // while (!Serial)
    //     ;
    // delay(1000);
    // Serial.println("Serial started");
    core.begin();
    core.Sensor_begin(true, 0); // initialise sensors and power them on
                                /*debug purpose*/
    Serial.println("Sensors initialised");
    //  core.psram_test(BYTES_READ_AT_ONCE,16);

    // core.psram_test(BYTES_READ_AT_ONCE, 16);
    // rp2040.fifo.push_nb(TEST_SD); // push test sd card command to fifo
    // delay(2000); // wait for sd card to initialise
    /*************** */
    pinMode(SD_SW_pin, OUTPUT);
    digitalWrite(SD_SW_pin, LOW);                // CONNECT SD CARD TO uC
    rp2040.fifo.push_nb(MOUNT_SD);               // mount sd card
    delay(2000);                                 // wait for sd card to initialise
    EscRX.begin(details(core.sc), &MSTR_Serial); // begin EasyTransfer for receiving data from master
    rp2040.wdt_begin(WDT_RESET_TIME);            // start watchdog timer
}

void loop()
{

    if (core.check_mode() == ACQ_MODE)
    {
        if (trig_status)
        {
            // detachInterrupt(TRIG_pin);
            trig_status = 0;     // reset trigger flag
            EscRX.receiveData(); // receive data from master
            core.get_data();     // get data from sensor and log into psram
            // attachInterrupt(TRIG_pin,trigger,RISING);
        }
    }

    else
    {

        if (wire_received)
        {
            Serial.println("Wire data received");
            wire_received = false;
            core.ExecuteWireCmd(wireData[0], wireData[1]);
            memset(wireData, 0, sizeof(wireData));
        }
#if defined(MFL)
        Sens.livereadMFL();
#else
        Sens.liveReadEGP();
#endif
        delay(500);
        rp2040.fifo.push_nb(HEALTH_FLAG); // push health flag to reset watchdog timer
    }
}

/***core1 varibles***/

FsFile file;

Logging l(&file, &psram_spi);

void setup1()
{
    pinMode(SD_VCC_EN_pin, OUTPUT);
    digitalWrite(SD_VCC_EN_pin, HIGH); // power on sd card

    // l.sd_card_init(); // initialise sd card
    // rp2040.wdt_reset();
    // rp2040.wdt_begin(WDT_RESET_TIME); // start watchdog timer
}
void loop1()
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
    // wireData[3] = i;
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