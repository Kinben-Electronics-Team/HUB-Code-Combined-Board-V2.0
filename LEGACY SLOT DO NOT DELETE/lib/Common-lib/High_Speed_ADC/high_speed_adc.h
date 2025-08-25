#ifndef HIGH_SPEED_ADC_h
#define HIGH_SPEED_ADC_h

#include <SPI.h>
#include <Arduino.h>

/*commands for read write operations*/
#define SET_BITS 0x13
#define CLR_BITS 0x10
#define RD_REG 0x11
#define WR_REG 0x12
#define NO_OP 0x00

/* Register addresses*/
#define PD_CNTL_Reg 0x004
#define SDI_CNTL_Reg 0x008
#define SDO_CNTL_Reg 0x00C
#define DATA_CNTL_Reg 0x010
#define OFST_CNTL_Reg 0x020
#define REF_MRG0_Reg 0x030

// const pin_size_t CS_pin = 1;
// enum adc_pins
// {
//     cs_pin = CS_pin,
//     sclk_pin,
//     tx_pin,
//     sdi0_pin,
//     sdi1_pin,
//     sdi2_pin,
//     sdi3_pin,
//     rvs_pin,
//     convst_pin,
//     rst_pin
//     // pin_size_t sclk_pin = CS_pin + 1;
//     // pin_size_t tx_pin = CS_pin + 2;
//     // pin_size_t sdi0_pin = CS_pin + 3;
//     // pin_size_t sdi1_pin = CS_pin + 4;
//     // pin_size_t sdi2_pin = CS_pin + 5;
//     // pin_size_t sdi3_pin = CS_pin + 6;
//     // pin_size_t rvs_pin = CS_pin + 7;
//     // pin_size_t convst_pin = CS_pin + 8;
//     // pin_size_t rst_pin = CS_pin + 9;

// };

struct ADCvar
{
    const pin_size_t csPin;
    const pin_size_t rvsPin;
    const pin_size_t convstPin;
    const pin_size_t rstPin;
    const pin_size_t sckPin;
    const pin_size_t mosiPin;
    const pin_size_t misoPin;
    SPISettings spiSetting;
    SPIClassRP2040 *spi = &SPI;
};

class HighSpeedADC
{
public:
    

    /**
     * \brief constructor initialise all pins
     * \param adcSet for configuration
     */
    HighSpeedADC(ADCvar adcSet);

    /**
     * \brief configure adc for normal spi and reference volatge 3.0V
     */
    void adc_config();

    /**
     * \brief read adc data via normal spi 
     * \return 18 bit adc value
     */
    uint32_t read_adc_data_1wire();

private:
    ADCvar adc; // adc configuration
    uint32_t data = 0; //store adc value
};

// void adc_init(pin_size_t cs_pin);
// void adc_config4bitmode();
// void adc_configure();

// uint32_t read_4lines_adc_data();
// uint32_t read_adc_data_1wire();

#endif