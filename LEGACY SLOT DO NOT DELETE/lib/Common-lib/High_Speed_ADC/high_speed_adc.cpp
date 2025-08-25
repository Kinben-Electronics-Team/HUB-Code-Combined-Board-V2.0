

#include "high_speed_adc.h"

//SPISettings ADCSPI(1000000, MSBFIRST, SPI_MODE0);

HighSpeedADC::HighSpeedADC(ADCvar adcSet)
    : adc(adcSet)
{
    pinMode(adc.rvsPin, INPUT);
    pinMode(adc.convstPin, OUTPUT);
    pinMode(adc.rstPin, OUTPUT);
    pinMode(adc.csPin, OUTPUT);


    

    // adc.spi->begin();
    // adc_config();
    // Serial.println("ADC initialised");
    // Serial.println(read_adc_data_1wire(), HEX); // read SDO CNTL register as cmd is send while configuring
}


void HighSpeedADC ::adc_config()
{
    uint32_t set3v_OFST_CAL_cmd = (SET_BITS << 17) | (OFST_CNTL_Reg << 8) | (0xff & 0x4);
    uint32_t get_OFST_CAL_data = (RD_REG << 17) | (OFST_CNTL_Reg << 8);
    uint32_t data = 0;

    digitalWrite(adc.csPin, HIGH);
    delay(1);
    digitalWrite(adc.rstPin, LOW);
    delay(100);
    digitalWrite(adc.rstPin, HIGH);

    digitalWrite(adc.convstPin, LOW);
    
    adc.spi->beginTransaction(adc.spiSetting);

    // send cmd to set 3v offset vref = 3.0V
    Serial.println("setting 3.0V reference voltage");
    digitalWrite(adc.csPin, LOW);

    while (digitalRead(adc.rvsPin))
        ;
    adc.spi->transfer((set3v_OFST_CAL_cmd >> 16) & 0xff);
    adc.spi->transfer((set3v_OFST_CAL_cmd >> 8) & 0xff);
    adc.spi->transfer(set3v_OFST_CAL_cmd & 0xff);
    digitalWrite(adc.csPin, HIGH);

    while (!digitalRead(adc.rvsPin))
        ;
    delay(1);

    // send cmd to read offset calibration register
    digitalWrite(adc.csPin, LOW);
    while (digitalRead(adc.rvsPin))
        ;
    adc.spi->transfer((get_OFST_CAL_data >> 16) & 0xff);
    adc.spi->transfer((get_OFST_CAL_data >> 8) & 0xff);
    adc.spi->transfer(0x00);
    digitalWrite(adc.csPin, HIGH);
    while (!digitalRead(adc.rvsPin))
        ;
    delay(1);

    // read offset calibration register
    digitalWrite(adc.csPin, LOW);
    while (digitalRead(adc.rvsPin))
        ;
    data = (data << 8) | adc.spi->transfer(0x00);
    data = (data << 8) | adc.spi->transfer(0x00);
    data = (data << 8) | adc.spi->transfer(0x00);
    digitalWrite(adc.csPin, HIGH);
    Serial.println(data, HEX);
    while (!digitalRead(adc.rvsPin))
        ;
    adc.spi->endTransaction();
}


uint32_t HighSpeedADC :: read_adc_data_1wire() // via spi pins
{
    data = 0;
    digitalWrite(adc.convstPin, HIGH);
    delayMicroseconds(2);
    digitalWrite(adc.convstPin, LOW);
    adc.spi->beginTransaction(adc.spiSetting);
    digitalWrite(adc.csPin, LOW);

    while (digitalRead(adc.rvsPin))
        ;

    for (int i = 0; i < 3; i++)
    {
        data = (data << 8) | adc.spi->transfer(0x00);
    }
    digitalWrite(adc.csPin, HIGH);
     while (!digitalRead(adc.rvsPin))
        ;
    adc.spi->endTransaction();  
      
    return (data>>6);
}


// void adc_init(pin_size_t cs_pin)
// {

//     // CS_pin = cs_pin;
//     // pin_initialisation();

//     pinMode(rvs_pin, INPUT);
//     pinMode(convst_pin, OUTPUT);
//     pinMode(rst_pin, OUTPUT);
//     pinMode(CS_pin, OUTPUT);

//     digitalWrite(CS_pin, HIGH);

//     digitalWrite(rst_pin, LOW);
//     delay(100);
//     digitalWrite(rst_pin, HIGH);

//     digitalWrite(convst_pin, LOW);

//     // SPI.setCS(CS_pin);
//     SPI.setSCK(sclk_pin);
//     SPI.setTX(tx_pin);
//     SPI.setRX(sdi0_pin);

//     SPI.begin();
//     delay(1);
//     adc_configure();
//     //adc_config4bitmode();
//     // SPI.end();
//     Serial.println("ADC initialised");

//     // pinMode(tx_pin, OUTPUT);
//     // digitalWrite(tx_pin, LOW);

//     // pinMode(sclk_pin, OUTPUT);
//     // digitalWrite(sclk_pin, LOW);

//     // pinMode(sdi0_pin, INPUT);
//     // pinMode(sdi1_pin, INPUT);
//     // pinMode(sdi2_pin, INPUT);
//     // pinMode(sdi3_pin, INPUT);

//     delay(1);
//     //Serial.println(read_4lines_adc_data(), HEX); // read SDO CNTL register as cmd is send while configuring
// }

// void adc_configure()
// {
    
//     uint32_t set3v_OFST_CAL_cmd = (SET_BITS << 17) | (OFST_CNTL_Reg << 8) | (0xff & 0x4);
//     uint32_t get_OFST_CAL_data = (RD_REG << 17) | (OFST_CNTL_Reg << 8);
//     uint32_t data = 0;
//     SPI.beginTransaction(ADCSPI);

//     // send cmd to set 3v offset vref = 3.0V
//     digitalWrite(CS_pin, LOW);

//     while (digitalRead(rvs_pin))
//         ;
//     SPI.transfer((set3v_OFST_CAL_cmd >> 16) & 0xff);
//     SPI.transfer((set3v_OFST_CAL_cmd >> 8) & 0xff);
//     SPI.transfer(set3v_OFST_CAL_cmd & 0xff);
//     digitalWrite(CS_pin, HIGH);

//     while (!digitalRead(rvs_pin))
//         ;
//     delay(1);

//     // send cmd to read offset calibration register
//     digitalWrite(CS_pin, LOW);
//     while (digitalRead(rvs_pin))
//         ;
//     SPI.transfer((get_OFST_CAL_data >> 16) & 0xff);
//     SPI.transfer((get_OFST_CAL_data >> 8) & 0xff);
//     SPI.transfer(0x00);
//     digitalWrite(CS_pin, HIGH);
//     while (!digitalRead(rvs_pin))
//         ;
//     delay(1);

//     // read offset calibration register
//     digitalWrite(CS_pin, LOW);
//     while (digitalRead(rvs_pin))
//         ;
//     data = (data << 8) | SPI.transfer(0x00);
//     data = (data << 8) | SPI.transfer(0x00);
//     data = (data << 8) | SPI.transfer(0x00);
//     digitalWrite(CS_pin, HIGH);
//     Serial.println(data, HEX);
//     while (!digitalRead(rvs_pin))
//         ;
//     SPI.endTransaction();
// }

// void adc_config4bitmode()
// {
//     uint32_t set_QSDO_cmd = (SET_BITS << 17) | (SDO_CNTL_Reg << 8) | (0xff & 0x0c);
//     uint32_t set3v_OFST_CAL_cmd = (SET_BITS << 17) | (OFST_CNTL_Reg << 8) | (0xff & 0x4);
//     uint32_t get_SDO_CNTL_data = (RD_REG << 17) | (SDO_CNTL_Reg << 8);
//     uint32_t get_OFST_CAL_data = (RD_REG << 17) | (OFST_CNTL_Reg << 8);
//     uint32_t data = 0;
//     SPI.beginTransaction(ADCSPI);

//     // send cmd to set 3v offset vref = 3.0V
//     digitalWrite(CS_pin, LOW);
//     while (digitalRead(rvs_pin))
//         ;
//     SPI.transfer((set3v_OFST_CAL_cmd >> 16) & 0xff);
//     SPI.transfer((set3v_OFST_CAL_cmd >> 8) & 0xff);
//     SPI.transfer(set3v_OFST_CAL_cmd & 0xff);
//     digitalWrite(CS_pin, HIGH);
//     while (!digitalRead(rvs_pin))
//         ;

//     delay(1);

//     // send cmd to read offset calibration register
//     digitalWrite(CS_pin, LOW);
//     while (digitalRead(rvs_pin))
//         ;
//     SPI.transfer((get_OFST_CAL_data >> 16) & 0xff);
//     SPI.transfer((get_OFST_CAL_data >> 8) & 0xff);
//     SPI.transfer(0x00);
//     digitalWrite(CS_pin, HIGH);
//     while (!digitalRead(rvs_pin))
//         ;

//     delay(1);

//     // read offset calibration register
//     digitalWrite(CS_pin, LOW);
//     while (digitalRead(rvs_pin))
//         ;
//     data = (data << 8) | SPI.transfer(0x00);
//     data = (data << 8) | SPI.transfer(0x00);
//     data = (data << 8) | SPI.transfer(0x00);
//     digitalWrite(CS_pin, HIGH);
//     Serial.println(data, HEX);
//     while (!digitalRead(rvs_pin))
//         ;

//     delay(1);

//     //send cmd to enable enhance spi and 4 sdo lines
//     digitalWrite(CS_pin, LOW);
//     while (digitalRead(rvs_pin))
//         ;

//     SPI.transfer((set_QSDO_cmd >> 16) & 0xff);
//     SPI.transfer((set_QSDO_cmd >> 8) & 0xff);
//     SPI.transfer((set_QSDO_cmd) & 0xff);

//     digitalWrite(CS_pin, HIGH);
//     while (!digitalRead(rvs_pin))
//         ;

//     delay(1);

//     // send cmd to read SDO CNTL register
//     digitalWrite(CS_pin, LOW);
//     while (digitalRead(rvs_pin))
//         ;
//     SPI.transfer((get_SDO_CNTL_data >> 16) & 0xff);
//     SPI.transfer((get_SDO_CNTL_data >> 8) & 0xff);
//     SPI.transfer(0x00);
//     digitalWrite(CS_pin, HIGH);
//     while (!digitalRead(rvs_pin))
//         ;

//     delay(1);

//     // as 4 sdo lines are enabled so we need to shift from 4 wire spi to enhance spi

//     SPI.endTransaction();

//     // delay(1);
// }

// uint32_t read_4lines_adc_data()
// {
//     uint32_t adc_data = 0;
//     digitalWrite(convst_pin, HIGH);
//     delayMicroseconds(1);
//     digitalWrite(convst_pin, LOW);
//     digitalWrite(CS_pin, LOW);
//     while (digitalRead(rvs_pin))
//         ; // wait for rvs pin to be pulled down
//     for (int i = 0; i < 6; i++)
//     {
//         // Generate a clock pulse and read data on the rising edge
//         digitalWrite(sclk_pin, LOW); // Set clock high
//         // delayMicroseconds(1);        // Half period delay for rising edge sampling

//         // Read data lines at the rising edge of the clock
//         int d0 = digitalRead(sdi0_pin);
//         int d1 = digitalRead(sdi1_pin);
//         int d2 = digitalRead(sdi2_pin);
//         int d3 = digitalRead(sdi3_pin);

//         // Combine the bits into a single 4-bit value
//         uint8_t data_chunk = (d3 << 3) | (d2 << 2) | (d1 << 1) | d0;
//         adc_data = (adc_data << 4) | data_chunk;
//         // Print the ADC data
//         // Serial.print("ADC Data: ");
//         // Serial.println(adcData, BIN); // Print as a binary string (4-bit)

//         digitalWrite(sclk_pin, HIGH); // Set clock low
//         // delayMicroseconds(1);         // Half period delay for falling edge
//     }
//     digitalWrite(CS_pin, HIGH);
//     while (!digitalRead(rvs_pin))
//         ; // wait for rvs pin to be pulled high
//     // Serial.println(adc_data);
//     return (adc_data >> 6);
// }

// uint32_t read_adc_data_1wire() // via spi pins
// {
//     uint32_t data = 0;
//     digitalWrite(convst_pin, HIGH);
//     delayMicroseconds(1);
//     digitalWrite(convst_pin, LOW);
//     digitalWrite(CS_pin, LOW);
//     while (digitalRead(rvs_pin))
//         ;
//     SPI.beginTransaction(ADCSPI);

//     for (int i = 0; i < 3; i++)
//     {
//         data = (data << 8) | SPI.transfer(0x00);
//     }
//     digitalWrite(CS_pin, HIGH);
//     while (!digitalRead(rvs_pin))
//         ; // wait for rvs pin to be pulled high
//     SPI.endTransaction();
//     Serial.println((data >> 6), HEX);
//     return data>>6;
// }
