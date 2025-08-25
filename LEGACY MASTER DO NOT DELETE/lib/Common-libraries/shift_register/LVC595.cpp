#include "LVC595.h"

ShiftRegister::ShiftRegister(uint8_t num_devices, pin_size_t srdaPin, pin_size_t stclkPin, pin_size_t shclkPin)
    :total_devices(num_devices), SRDA_pin(srdaPin), SRCLK_pin(stclkPin), SHCLK_pin(shclkPin)
{
    /* configure shift register pins*/
    pinMode(SRDA_pin, OUTPUT);
    pinMode(SRCLK_pin, OUTPUT);
    if (SHCLK_pin != -1)
    {
        pinMode(SHCLK_pin, OUTPUT);
        digitalWrite(SHCLK_pin, LOW);
    }

    total_devices = num_devices;
    Total_output = one_chip_output * total_devices;
    digitalWrite(SRDA_pin, LOW);
    srda_status = 1;
    digitalWrite(SRCLK_pin, LOW);
    delayMicroseconds(1);
    ShiftCS();
}


void ShiftRegister::update_device_count(uint8_t num_devices)
{
    total_devices = num_devices;
    Total_output = one_chip_output * total_devices;
}


void ShiftRegister::SelectCS(int CSpin)
{
    High_all_CS();
    StoreOneLOWbit();
    for (int i = 0; i <= CSpin; i++) // move low bit to desired
    {
        ShiftCS();
    }
}

void ShiftRegister::SelectCS(int CSpin, bool pinState)
{
    High_all_CS();
    StoreOnebit(pinState);
    for (int i = 0; i <= CSpin; i++) // move low bit to desired
    {
        ShiftCS();
    }
}

void ShiftRegister::High_all_CS()
{
    digitalWrite(SRDA_pin, HIGH);
    srda_status = 1;
    for (int i = 0; i <= Total_output; i++) // Q15 + 1
    {
        ShiftCS();
    }
}

void ShiftRegister::ShiftCS()
{
    if (SHCLK_pin != -1)
    {
        digitalWrite(SHCLK_pin, HIGH);
        digitalWrite(SHCLK_pin, LOW);
    }
    digitalWrite(SRCLK_pin, HIGH);
    digitalWrite(SRCLK_pin, LOW);
}

void ShiftRegister::StoreOneLOWbit()
{
    // High_all_CS();
    digitalWrite(SRDA_pin, LOW);
    srda_status = 0;

    ShiftCS();

    digitalWrite(SRDA_pin, HIGH); // set other bits following low bit to high
    srda_status = 1;
}

void ShiftRegister::StoreOnebit(bool BitVal)
{
    High_all_CS();
    digitalWrite(SRDA_pin, BitVal);
    srda_status = BitVal;

    ShiftCS();

    digitalWrite(SRDA_pin, !BitVal); // set other bits following low bit to high
    srda_status = !BitVal;
}

void ShiftRegister ::set_val_LSB_first(uint32_t val)
{
    for (int i = 0; i < Total_output; i++)
    {
        (val & (1 << i)) ? digitalWrite(SRDA_pin, HIGH) : digitalWrite(SRDA_pin, LOW);
        ShiftCS();
    }
}

void ShiftRegister ::set_val_MSB_first(uint32_t val)
{
    for (int i = Total_output - 1; i >= 0; i--)
    {
        (val & (1 << i)) ? digitalWrite(SRDA_pin, HIGH) : digitalWrite(SRDA_pin, LOW);
        ShiftCS();
    }
}