#ifndef LVC595_h
#define LVC595_h

#include <Arduino.h>

/* parallel output pins after cascading two shift registers */
enum parallelpin
{
    Q0 = 0,
    Q1,
    Q2,
    Q3,
    Q4,
    Q5,
    Q6,
    Q7,
    Q8,
    Q9,
    Q10,
    Q11,
    Q12,
    Q13,
    Q14,
    Q15,
};

#define CS12 Q2
#define CS11 Q3
#define CS10 Q4
#define CS9 Q5
#define CS8 Q6
#define CS7 Q7

#define CS6 Q8
#define CS5 Q9
#define CS4 Q10
#define CS3 Q11
#define CS2 Q12
#define CS1 Q13

#define one_chip_output 8 // number of outputs in one shitf register

class ShiftRegister
{
private:
    bool srda_status = 0;
    unsigned long Total_output = one_chip_output * total_devices;
    const pin_size_t SRCLK_pin;
    const pin_size_t SRDA_pin;
    const pin_size_t SHCLK_pin;

    /* data */
public:
    uint8_t total_devices; // max number of device is 8
    /**
     * @brief Constructor for shift register. Initialise shift register parameters
     * @param num_devices number of shiftregisters used
     * @param srdaPin Data pin
     * @param srclkPin shift clock pin
     * @param shclkPin storage clock pin. if both srclk and shclk pins short together then shclkPin = -1
     */
    ShiftRegister(uint8_t num_devices, pin_size_t srdaPin, pin_size_t stclkPin, pin_size_t shclkPin = -1);

    /**
     * @brief for configuration pupose in case later you want to change the number of shift registers
     * @param num_devices number of shiftregisters used
     */
    void update_device_count(uint8_t num_devices);

    /**
     * @brief reset any desired output of shift register
     * @param CSpin output pin to be reset starting from 0
     */
    void SelectCS(int CSpin);

    /**
     * @brief set or reset any desired output of shift register
     * @param CSpin output pin to be set or reset startinng from 0
     * @param pinState desired output state HIGH or LOW
     */
    void SelectCS(int CSpin, bool pinState);

    /* shift one bit */
    void ShiftCS();

    /* set all output pins HIGH */
    void High_all_CS();
    /**store one low bit to store register.
     * when one shift is done this value appear in b0 of fisrt shift resgister
     */
    void StoreOneLOWbit();

    /**
     * @brief Store one low or high bit to store register.
     *
     * When one shift is done this value appear in b0 of fisrt shift resgister
     * @param BitVal value to be stored, HIGH or LOW
     */
    void StoreOnebit(bool BitVal);

    /**
     * @brief set any value in the output of shift register LSB will appear at bit0
     * @param val value to be set
     */
    void set_val_LSB_first(uint32_t val);

    /**
     * @brief set any value in the output of shift register MSB will appear at bit0
     * @param val value to be set
     */
    void set_val_MSB_first(uint32_t val);
};

#endif
