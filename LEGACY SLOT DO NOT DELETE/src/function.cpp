#include "function.h"

uint8_t SID = 0, HID = 0;
Adafruit_NeoPixel pixels(1, LED_pin, NEO_GRB + NEO_KHZ800);

Logging::Logging(FsFile *file, psram_spi_inst_t *psramSPI)
    : f(file), psram_spi(psramSPI)
{
}

void Logging::getFilename()
{
    static unsigned int fileCount;
    snprintf(filename, sizeof(filename), "HID%02uSID%02u_%04u.hex", HID, SID, fileCount);
    fileCount++;
}

void Logging::sd_card_init()
{

    /*sd pins init */
    digitalWrite(SD_SW_pin, LOW);      // CONNECT SD CARD TO uC
    digitalWrite(SD_VCC_EN_pin, HIGH); // power on sd card

    delay(100); // wait for sd card to power on

    if (!sd.begin(SdioConfig(SD_CLK_pin, SD_CMD_pin, SD_D0_pin)))
    {
        sd.initErrorHalt(&Serial);
    }
    else
    {
        Serial.println("Card successfully initialized.");
        getFilename();
        while (f->open(filename, O_RDONLY))
        {
            f->close(); // close the file if it exists
            getFilename();
            rp2040.wdt_reset();
        }
    }
}

void Logging::create_file(bool isopen)
{
    /*if a file is open then close is first*/
    if (f->isOpen())
        f->close();
    if (!f->open(filename, O_RDWR | O_CREAT | O_TRUNC)) // open file for writing, create if not exists, truncate if exists
        Serial.println("file open failed");

    f->preAllocate(fileSize); // preallocate memory block for the file
    getFilename();
    fileSize = 0;
}

void Logging::check_fifo(uint32_t fifo_val)
{
    uint32_t TotalSampleCollected = 0;
    rp2040.wdt_reset();
    switch (fifo_val)
    {
    case DATA_READY:
        
        // update bytes to be read and start reading from psram
        if (rp2040.fifo.available())
        {
            rp2040.fifo.pop_nb(&TotalSampleCollected);
            psram_bytes_to_be_read = ((TotalSampleCollected - psram_sample_read_count) * Sample_size_to_read) - sample_read_bytes; // update bytes to be read
            // Serial.println(psram_bytes_to_be_read);
            psram_read_data();
            
        }

        break;

    case CLOSE_FILE:

        rp2040.fifo.pop_nb(&TotalSampleCollected);                                                                             // send acknowledgement flag to core0
        psram_bytes_to_be_read = ((TotalSampleCollected - psram_sample_read_count) * Sample_size_to_read) - sample_read_bytes; // update bytes to be read
        while (psram_bytes_to_be_read)
        {
            psram_read_data();                                                                                                     // read data
            psram_bytes_to_be_read = ((TotalSampleCollected - psram_sample_read_count) * Sample_size_to_read) - sample_read_bytes; // update bytes to be read
        }
        if (log_buff_index) // incomplete buffer
        {
            log_data(log_buff, log_buff_index); // log remaining data
            log_buff_index = 0;
        }
        f->truncate(); // reduce the file size by releasing unused clusters
        f->close();    // close file
        rp2040.fifo.push_nb(CLOSE_FILE_ACK);
        //_mySerial->->println("file closed");

        break;

    case MOUNT_SD: // Opens a file for new logging
        // delay(1000);
        sd_card_init();
        //_mySerial->->println("sd mounted");
        break;

    case UNMOUNT_SD:
        // f_unmount("");
        //_mySerial->->println("unmount");
        break;

    case LOG_START: // start logging
        //_mySerial->->printf("file name %d\n", fileCount);
        create_file(false);
        log_buff_index = 0;
        break;

    case TEST_SD:
        test_Sd_card(&Serial);
        break;

    case UPDATE_NUM_EGP:
        // rp2040.fifo.pop_nb(&Sample_size_to_read);
        break;

        // case UPDATE_SID:
        //     rp2040.fifo.pop_nb(&temp);
        //     if (SID != static_cast<uint8_t>(temp))
        //     {
        //         SID = static_cast<uint8_t>(temp);
        //         EEPROM.write(SIDadd, SID);
        //         if (!EEPROM.commit())
        //             Serial.println("EEPROM write fail");
        //     }
        //     Serial.printf("received SID is %x\n", temp);
        //     break;

        // case UPDATE_HID:
        //     rp2040.fifo.pop_nb(&temp);
        //     if (HID != static_cast<uint8_t>(temp))
        //     {
        //         HID = static_cast<uint8_t>(temp);
        //         EEPROM.write(HIDadd, HID);
        //         if (!EEPROM.commit())
        //             Serial.println("EEPROM write fail");
        //     }
        //     Serial.printf("received HID is %x\n", temp);
        //     break;

    case HEALTH_FLAG:
        rp2040.wdt_reset();
        break;

    case RESET_WDT_EVENT:
        rp2040.wdt_reset();
        // log_error(RESET_WDT_EVENT);
        break;

    default:
        break;
    }
}

void Logging::psram_read_data()
{

    int available_buff_space = LOG_BUFF_SIZE - log_buff_index;
    /* evaluate the availability of space in log buffer and bytes to be read to read data */
    if (available_buff_space > psram_bytes_to_be_read) // check if space avaiblable in log buffer for maximum bytes that can be read at once
    {
        if (psram_bytes_to_be_read > BYTES_READ_AT_ONCE) // if yes then the bytes avaiblable to be read is greater than maximum bytes that can be read at once
        {
            psram_collect_sample(BYTES_READ_AT_ONCE); // if yes then read max bytes
        }
        else // if no then read only available  bytes  in the multiple of PSRAM chunk size
        {
            psram_collect_sample((psram_bytes_to_be_read / PSRAM_CHUNK_SIZE) * PSRAM_CHUNK_SIZE);
        }
    }
    // else if (available_buff_space < psram_bytes_to_be_read)
    // {
    //     psram_collect_sample((psram_bytes_to_be_read / PSRAM_CHUNK_SIZE) * PSRAM_CHUNK_SIZE);
    // }
    else
    {
        // if (available_buff_space % 16)
        // {
        //     _mySerial->printf("available buff space is not multiple of 16 space = %d", available_buff_space);
        // }
        psram_collect_sample(available_buff_space);
    }
    /*if buffer is full then log data*/
    if (LOG_BUFF_SIZE <= log_buff_index)
    {
        log_buff_index = 0;
        log_data(log_buff, LOG_BUFF_SIZE);
        memset(log_buff, 0, LOG_BUFF_SIZE);
    }
}

void Logging::psram_collect_sample(unsigned int read_bytes)
{
    /* check if the psram address is near max readable address*/
    if ((psram_bytes_read_add + read_bytes) >= max_read_add)
    {
        /* reading ending address of this cycle */
        unsigned int bytes_to_read = max_read_add - psram_bytes_read_add;

        /*read in chunks*/
        for (int i = 0; i + PSRAM_CHUNK_SIZE <= bytes_to_read; i += PSRAM_CHUNK_SIZE)
        {
            psram_read(psram_spi, psram_bytes_read_add + i, log_buff + log_buff_index + i, PSRAM_CHUNK_SIZE);
        }
        /*read remaining bytes*/
        for (int i = (bytes_to_read / PSRAM_CHUNK_SIZE) * PSRAM_CHUNK_SIZE; i < bytes_to_read; i++)
        {
            log_buff[log_buff_index + i] = psram_read8(psram_spi, psram_bytes_read_add + i);
        }
        // /*read in chunks */
        // if (bytes_to_read / PSRAM_CHUNK_SIZE)
        // {

        //     for (int i = 0; i < bytes_to_read / PSRAM_CHUNK_SIZE; i++)
        //     {
        //         increement = i * PSRAM_CHUNK_SIZE;
        //         psram_read(psram_spi, psram_bytes_read_add + increement, log_buff + log_buff_index + increement, PSRAM_CHUNK_SIZE);
        //     }
        //     /*read the remaining bytes byte by byte if bytes to be read here was not a multiple of chunk size*/

        //     increement += PSRAM_CHUNK_SIZE;
        // }
        // if (bytes_to_read % PSRAM_CHUNK_SIZE)
        // {
        //     for (int i = increement; i < bytes_to_read; i++)
        //     {
        //         log_buff[log_buff_index + i] = psram_read8(psram_spi, psram_bytes_read_add + i);
        //     }
        // }

        psram_bytes_read_add = 0;                // now psram read address will start from zero
        psram_cycle_read_count++;                // read cycle count will increement
        log_buff_index += bytes_to_read;         // bytes_to_read;         // increement log buffer index
        update_sample_read_count(bytes_to_read); // update sample count which read from psram
    }

    /*read bytes in in chunks as read address is not close to end*/
    else
    {

        /*read in chunks*/
        for (int i = 0; i + PSRAM_CHUNK_SIZE <= read_bytes; i += PSRAM_CHUNK_SIZE)
        {
            psram_read(psram_spi, psram_bytes_read_add + i, log_buff + log_buff_index + i, PSRAM_CHUNK_SIZE);
        }
        /*read remaining bytes*/
        for (int i = (read_bytes / PSRAM_CHUNK_SIZE) * PSRAM_CHUNK_SIZE; i < read_bytes; i++)
        {
            log_buff[log_buff_index + i] = psram_read8(psram_spi, psram_bytes_read_add + i);
        }

        psram_bytes_read_add += read_bytes; // increement the psram read address that has been read
        log_buff_index += read_bytes;       // increement log buffer index
        update_sample_read_count(read_bytes);
    }
}

void Logging::psram_sample_collect_bytes(unsigned int num_bytes)
{
    if ((psram_bytes_read_add + num_bytes) >= max_read_add)
    {
        /* reading ending address of this cycle */
        unsigned int bytes_to_read = max_read_add - psram_bytes_read_add;
        for (unsigned int i = 0; i < bytes_to_read; i++)
        {
            log_buff[log_buff_index + i] = psram_read8(psram_spi, psram_bytes_read_add);
        }
        psram_bytes_read_add = 0;                // now psram read address will start from zero
        psram_cycle_read_count++;                // read cycle count will increement
        log_buff_index += bytes_to_read;         // increement log buffer index
        update_sample_read_count(bytes_to_read); // update sample count which read from psram
    }
    else
    {
        for (int i = 0; i < num_bytes; i++)
        {
            log_buff[log_buff_index + i] = psram_read8(psram_spi, psram_bytes_read_add);
        }
        psram_bytes_read_add += num_bytes; // increement the psram read address that has been read
        log_buff_index += num_bytes;       // increement log buffer index
        update_sample_read_count(num_bytes);
    }
}

void Logging::update_sample_read_count(unsigned int read_bytes)
{
    psram_sample_read_count += read_bytes / Sample_size_to_read;
    sample_read_bytes += read_bytes % Sample_size_to_read;
    if (sample_read_bytes >= Sample_size_to_read)
    {
        psram_sample_read_count += sample_read_bytes / Sample_size_to_read;
        sample_read_bytes = sample_read_bytes % Sample_size_to_read;
    }
    // _mySerial->printf("sample read %lu \n", psram_sample_read_count);
}

void Logging::log_data(uint8_t *buff, uint32_t buff_size)
{
    // Serial.printf("log start %u\n", micros());
    size_t bw = f->write(buff, buff_size); // write data to file
    // Serial.printf("log end %u\n", micros());
    if (f->sync()) // sync the file to ensure data is written to disk
    {
        //  Serial.printf("log end %u\n", micros());
        Serial.printf("File written successfully, size: %u bytes\n", bw);
        fileSize += bw;
        // memset(buff, 0, buff_size);
        if (fileSize >= FILE_SIZE_LIMIT)
        {
            create_file(true);
        }
        // Serial.println("log done");
    }
    else
    {
        Serial.printf("Error writing to file: %s\n", f->getError());
        // log_error(FR_DISK_ERR);
    }
    rp2040.fifo.clear();
    // rp2040.fifo.push_nb(LOG_DONE);
}

// FRESULT Logging::log_error(uint8_t error)
// {
//     // f_close(&file);
//     if (FR_OK != f_open(&file, "Error_log.txt", FA_OPEN_APPEND | FA_WRITE))
//     {
//         f_close(&file);
//         f_unmount("");
//         f_mount(&ptr->state.fatfs, "", 1);
//         return FR_DISK_ERR;
//     }
//     switch (error)
//     {
//     case FR_DISK_ERR:
//         f_printf(&file, "XXXXX_FR_DISK_ERROR_XXXXX\n");
//         break;
//     // case FR_INT_ERR:
//     //     Serial.println("Internal error");
//     //     break;
//     // case FR_NOT_READY:
//     //     Serial.println("Not ready");
//     //     break;
//     // case FR_NO_FILE:
//     //     Serial.println("No file");
//     //     break;
//     // case FR_NO_PATH:
//     //     Serial.println("No path");
//     //     break;
//     // case FR_INVALID_NAME:
//     //     Serial.println("Invalid name");
//     //     break;
//     // case FR_DENIED:
//     //     Serial.println("Denied");
//     //     break;
//     // case FR_EXIST:
//     //     Serial.println("Exist");
//     //     break;
//     case FR_INVALID_OBJECT:
//         f_printf(&file, "XXXXX_FR_INVALID_OBJECT_XXXXX\n");
//         break;
//     case RESET_WDT_EVENT:
//         f_printf(&file, "XXXXX_MCU_RESET_XXXXX\n");
//         break;
//     default:
//         f_printf(&file, "XXXXX_FR_ERROR_XXXXX\n");
//         break;
//     }

//     f_sync(&file);
//     f_close(&file);
//     return fr;
// }

bool Logging::test_Sd_card(SerialUSB *_testSerial)
{
    digitalWrite(SD_VCC_EN_pin, LOW); // power on sd card
    digitalWrite(SD_SW_pin, LOW);     // CONNECT SD CARD TO uC
    delay(100);
    digitalWrite(SD_VCC_EN_pin, HIGH); // power on sd card
    delay(100);
    Serial.println("Testing SD card...");
    if (!sd.begin(SdioConfig(SD_CLK_pin, SD_CMD_pin, SD_D0_pin)))
    {
        Serial.println("SD card initialization failed!");
        sd.printSdError(_testSerial);
        return false;
    }
    Serial.println("Card successfully initialized.");
    Serial.println("\nls:");
    sd.ls(LS_A | LS_DATE | LS_SIZE); // Add LS_R for recursive list.
    return true;
}

Acquire::Acquire(psram_spi_inst_t *psramSPI, Sensors *sens, TwoWire *_wire, SerialUSB *serial)
    : psram_spi(psramSPI), sensor(sens), _myWire(_wire), _mySerial(serial)
{
    memset(buffer, 0, SAMPLE_BUFFER_SIZE); // clear old data
}

void Acquire::begin()
{
    /*eeprom initialise */
    EEPROM.begin(256);
    SID = EEPROM.read(SIDadd);
    HID = EEPROM.read(HIDadd);
    Serial.printf("SID: %d, HID: %d\n", SID, HID);

    /*initialise wire for communication with master */
    pinMode(I2C_IO_pin, INPUT);
    _myWire->setClock(I2C_CLOCK_SPEED);
    _myWire->setSDA(SDA_pin);
    _myWire->setSCL(SCL_pin);
    _myWire->begin(I2C_SLAVE_ADDRESS);
    _myWire->onReceive(receiveEventCallback);
    _myWire->onRequest(requestEventCallback);

    /*initialise ws led */
    pixels.begin();
    pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // set white color at beginning
    pixels.show();
    /*initialise sensor SPI */
    SENSOR_SPI.setRX(SPI_MISO_PIN);
    SENSOR_SPI.setSCK(SPI_SCK_PIN);
    SENSOR_SPI.setTX(SPI_MOSI_PIN);
    SENSOR_SPI.begin(false);

    pinMode(SENSOR_PS_EN_pin, OUTPUT);

    // Set priorities (lower number = higher priority)
    irq_set_priority(IO_IRQ_BANK0, TRIG_PRIORITY); // 0x00 (highest) to 0xFF (lowest)
    irq_set_priority(I2C0_IRQ, WIRE_PRIORITY);

    pinMode(TRIG_pin, INPUT);
    pinMode(MODE_pin, INPUT);
}

void Acquire::Sensor_begin(bool debugEN, uint8_t numEgp)
{
    if (numEgp)
        update_EGP_count(numEgp);
    digitalWrite(SENSOR_PS_EN_pin, HIGH);
    delay(10);
    sensor->enableDebugging(debugEN);
#if defined(EGP)

    Serial.println("EGP mode");
    sensor->CheckEGPSensors();
    sensor->configEGP();
    sensor->runEGP();
    Data_size = 14 * numEgp; // 14 bytes for each EGP sensor

#elif defined(MFL)

    Serial.println("MFL mode");
    sensor->CheckMFLSensors();
    sensor->configMFL();
    sensor->runMFL();
    Data_size = 44;
#else
#error "Please define either EGP or MFL"
#endif
}

void Acquire::update_EGP_count(uint8_t numEGP)
{
    if (numEGP)
    {
        Data_size = numEGP * 14;
        // Sample_size = NUM_START_BYTES + Data_size + NUM_COUNT_BYTE + NUM_ADC_BYTES;
        sensor->updateEGPCount(numEGP);
        // rp2040.fifo.push_nb(UPDATE_NUM_EGP);
        // rp2040.fifo.push_nb(Sample_size);
    }
}

bool Acquire::check_mode()
{
    bool current_mode = digitalRead(MODE_pin);
    if (mode_stat != current_mode)
    {
        if (current_mode == ACQ_MODE)
        {

            sensor->enableDebugging(false);
            rp2040.fifo.push_nb(LOG_START); // send cmd to core 1 to create and open a file
            _mySerial->println("logging started");
            _mySerial->end();
            pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // set white color at beginning
            pixels.show();
            attachInterrupt(TRIG_pin, trigger, FALLING);
            log_started = 1;
            sc = 0;
        }
        else
        {
            pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // set white color at beginning
            pixels.show();
            if (log_started)
            {
                uint32_t ack_val = 0;
                detachInterrupt(TRIG_pin);
                _mySerial->begin(BAUDRATE);

                log_started = 0;
                rp2040.fifo.push_nb(CLOSE_FILE);
                rp2040.fifo.push_nb(psram_sample_write_count);
                delay(1000);
                rp2040.fifo.pop_nb(&ack_val);
                while (ack_val != CLOSE_FILE_ACK)
                {
                    rp2040.fifo.push_nb(CLOSE_FILE);
                    rp2040.fifo.push_nb(psram_sample_write_count);
                    delay(1000);
                    rp2040.fifo.pop_nb(&ack_val);
                    _mySerial->print("a");
                }

                _mySerial->println("logging stopped");
            }
            sensor->enableDebugging(true);
        }
        mode_stat = current_mode;
    }
    return mode_stat;
}

void Acquire::get_data()
{
    /*check if memory is not full in psram*/
    if ((SRAM_SIZE - psram_bytes_write_add) < Sample_size)
    {
        psram_bytes_write_add = 0; // start from address zero again as psram is full
        psram_cycle_write_count++; // upgrade psram write cycle
    }

    memcpy(buffer, startBytes, NUM_START_BYTES); // copy start bytes
#if defined(EGP)
    sensor->getEGPdata(buffer + NUM_START_BYTES); // acquire sensor data
#elif defined(MFL)
    sensor->getMFLdata(buffer + NUM_START_BYTES);
#else
#error "Please define either EGP or MFL"
#endif

    memcpy(buffer + (NUM_START_BYTES + Data_size), &sc, sizeof(sc)); // copy sample count value

    /*write data to psram in chunks of 16 bytes*/
    for (int i = 0; i + PSRAM_CHUNK_SIZE <= Sample_size; i += PSRAM_CHUNK_SIZE)
    {
        psram_write(psram_spi, psram_bytes_write_add + i, buffer + i, PSRAM_CHUNK_SIZE);
    }

    /*write the remaining bytes*/
    for (int i = (Sample_size / PSRAM_CHUNK_SIZE) * PSRAM_CHUNK_SIZE; i < Sample_size; i++)
    {
        psram_write8(psram_spi, psram_bytes_write_add + i, buffer[i]);
    }

    psram_bytes_write_add += Sample_size;          // increement psram address
    psram_sample_write_count++;                    // increement sample write val to match when reading in core1
    rp2040.fifo.push_nb(DATA_READY);               // send data ready and psram available flag to core0
    rp2040.fifo.push_nb(psram_sample_write_count); // send the value of address upto which psram is written
    memset(buffer, 0, SAMPLE_BUFFER_SIZE);         // clear old data
    // memset(var.wireData, 0, 10);            // clear old adc data
    // interrupts();                                  // enable interrupts
    // var.trig_status = 0;
}

void Acquire::requestEvent()
{
    _myWire->write(0x1); // Send acknowledgment message
}

void Acquire::ExecuteWireCmd(uint8_t cmd, uint8_t data)
{
    Serial.println("executing wire cmd");
    if (digitalRead(I2C_IO_pin))
    {
        /*it must be a command for MCU*/
        switch (cmd)
        {
        case I2C_CMD_SD_CON: // connect sd card to card reader
            _mySerial->printf("cmd executing sdcon %x\n", cmd);
            digitalWrite(SD_VCC_EN_pin, LOW); // power OFF sd card
            delay(10);
            // digitalWrite(SD_VCC_EN_pin, HIGH); // power ON sd card
            digitalWrite(SD_SW_pin, HIGH);
            rp2040.fifo.push_nb(UNMOUNT_SD);
            sd_con_stat = 1;
            break;
        case I2C_CMD_SD_DISCON: // disconnect sd card from card reader

            if (sd_con_stat) // disconnect only if sd card us connected to card reader
            {
                _mySerial->printf("cmd executing sd discon %x\n", cmd);
                sd_con_stat = 0;
                digitalWrite(SD_VCC_EN_pin, HIGH); // power OFF sd card
                digitalWrite(SD_SW_pin, LOW);
                rp2040.fifo.push_nb(MOUNT_SD);
            }
            else
            {
                _mySerial->println("card is already disconnected");
            }
            break;

        case SID_REG:
            _mySerial->printf("received SID is %x\n", data);
            if (SID != data)
            {
                SID = data;
                EEPROM.write(SIDadd, SID);
                if (!EEPROM.commit())
                    _mySerial->println("EEPROM write fail");
                else
                    scb_hw->aircr = (0x05FA << M0PLUS_AIRCR_VECTKEY_LSB) | M0PLUS_AIRCR_SYSRESETREQ_BITS; // reset the mcu
            }

            // rp2040.fifo.push_nb(UPDATE_SID);
            // rp2040.fifo.push_nb(data);
            break;
        case HID_REG:
            _mySerial->printf("received HID is %x\n", data);
            if (HID != data)
            {
                HID = data;
                EEPROM.write(HIDadd, HID);
                if (!EEPROM.commit())
                    _mySerial->println("EEPROM write fail");
            }
            // rp2040.fifo.push_nb(UPDATE_HID);
            // rp2040.fifo.push_nb(data);
            break;
        default:
            break;
        }
    }
    else
        Serial.println("I2C IO pin is low, not a command for MCU");
}

void Acquire::psram_test(unsigned int num_bytes, uint8_t chunk_size)
{
    uint8_t write_data[num_bytes], read_data[num_bytes];
    uint32_t x, y;
    const size_t totalBytes = num_bytes - (num_bytes % chunk_size);
    for (int i = 0; i < num_bytes; i++)
    {
        write_data[i] = i;
    }
    _mySerial->printf("test begin. writting %d bytes at once and total bytes are %d\n", chunk_size, num_bytes);
    x = micros();
    for (int i = 0; i < totalBytes; i += chunk_size)
    {
        psram_write(psram_spi, psram_bytes_write_add + i, write_data + i, chunk_size);
    }
    for (int i = ((num_bytes / chunk_size) * chunk_size); i < num_bytes; i++)
    {
        psram_write8(psram_spi, i, write_data[i]); // buffer[i]
    }
    y = micros() - x;
    _mySerial->printf("time to write %luus\n", y);
    delay(100);
    // for(int i = ((num_bytes / chunk_size) * chunk_size); i < num_bytes; i++)
    // {
    //     _mySerial->printf("%d   ", psram_read8(&psram_spi, i));
    // }

    x = micros();
    for (int i = 0; i < num_bytes; i += chunk_size)
    {
        psram_read(psram_spi, psram_bytes_write_add + i, read_data + i, chunk_size);

        // read_data[i]= psram_read8(&psram_spi, i);
    }
    for (int i = ((num_bytes / chunk_size) * chunk_size); i < num_bytes; i++)
    {
        read_data[i] = psram_read8(psram_spi, i); // buffer[i]
    }
    // for (int i = 0; i < num_bytes; i++)
    // {
    //     read_data[i] = psram_read8(&psram_spi, i);
    // }
    // for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    // {
    //     if (write_data[i] != psram_read8(&psram_spi, psram_bytes_write_count + i))
    //     {
    //         _mySerial->printf("wrong read %d\n", i);
    //         break;
    //     }
    //     // buffer[i]
    // }
    y = micros() - x;
    if (memcmp(read_data, write_data, num_bytes))
    {
        _mySerial->println("false data");
        for (int i = 0; i < num_bytes; i++)
        {
            _mySerial->printf("%d, ", read_data[i]);
        }
    }
    // for (int i = 0; i < 128; i++)
    // {
    //     _mySerial->printf("%d, ", read_data[i]);
    // }
    _mySerial->printf("\ntime to read %luus\n", y);
}
