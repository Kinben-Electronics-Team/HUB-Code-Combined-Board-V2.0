#include "MyI2CDeviceKT.h"

// Constructor
MyI2CDevice::MyI2CDevice(TwoWire *wirePort, uint8_t deviceAddress, Stream *userStream)
{
    _wirePort = wirePort;
    _deviceAddress = deviceAddress;
    SerialStream = userStream;
}

bool MyI2CDevice::check_comm()
{
    _wirePort->beginTransmission(_deviceAddress);
    return !(_wirePort->endTransmission()); // Returns true if the device responds
}

void MyI2CDevice::setpinMode(int pin, int pMode)
{
    if (pin < 0 || pin > 15 || pMode < 0 || pMode > 3)
    {
        if (SerialStream)
            SerialStream->println("Invalid pin or mode");
        return;
    }

    uint8_t configReg = (pin < 8) ? CONFIG_REG_PORT_0 : CONFIG_REG_PORT_1;
    uint8_t pullUpDownSelReg = (pin < 8) ? PULL_SEL_REG_PORT_0 : PULL_SEL_REG_PORT_1;
    uint8_t pullUpDownEnReg = (pin < 8) ? PULL_EN_REG_PORT_0 : PULL_EN_REG_PORT_1;
    uint8_t outputTypeReg = (pin < 8) ? OUTPUT_TYPE_REG_PORT_0 : OUTPUT_TYPE_REG_PORT_1;
    uint8_t pinOffset = pin % 8;

    uint8_t configValue = readRegister(configReg);
    uint8_t pullUpDownSelValue = readRegister(pullUpDownSelReg);
    uint8_t pullUpDownEnValue = readRegister(pullUpDownEnReg);
    uint8_t outputTypeValue = readRegister(outputTypeReg);

    switch (pMode)
    {
    case 0: // Input
        configValue |= (1 << pinOffset);
        break;
    case 1: // Output (Push-Pull)
        configValue &= ~(1 << pinOffset);
        outputTypeValue &= ~(1 << pinOffset); // Clear bit for push-pull
        break;
    case 2: // Pull-up
        pullUpDownSelValue |= (1 << pinOffset);
        pullUpDownEnValue |= (1 << pinOffset);
        configValue |= (1 << pinOffset);
        break;
    case 3: // Pull-down
        pullUpDownSelValue &= ~(1 << pinOffset);
        pullUpDownEnValue |= (1 << pinOffset);
        configValue |= (1 << pinOffset);
        break;
    }

    if (pMode == 2 || pMode == 3)
    {
        writeRegister(pullUpDownSelReg, pullUpDownSelValue);
        writeRegister(pullUpDownEnReg, pullUpDownEnValue);
    }
    writeRegister(configReg, configValue);

    if (pMode == 1)
    {
        writeRegister(outputTypeReg, outputTypeValue);
    }
}

bool MyI2CDevice::digitalRead(int pin)
{
    if (pin < 0 || pin > 15)
    {
        if (SerialStream)
            SerialStream->println("Invalid pin");
        return false;
    }

    uint8_t inputReg = (pin < 8) ? INPUT_REG_PORT_0 : INPUT_REG_PORT_1;
    uint8_t pinOffset = pin % 8;

    uint8_t inputValue = readRegister(inputReg);
    return (inputValue & (1 << pinOffset)) != 0;
}

void MyI2CDevice::digitalWrite(int pin, bool pState)
{
    if (pin < 0 || pin > 15)
    {
        if (SerialStream)
            SerialStream->println("Invalid pin");
        return;
    }

    uint8_t outputReg = (pin < 8) ? OUTPUT_REG_PORT_0 : OUTPUT_REG_PORT_1;
    uint8_t pinOffset = pin % 8;

    uint8_t outputValue = readRegister(outputReg);
    if (pState)
    {
        outputValue |= (1 << pinOffset);
    }
    else
    {
        outputValue &= ~(1 << pinOffset);
    }
    writeRegister(outputReg, outputValue);
}

// Write to a register
bool MyI2CDevice::writeRegister(uint8_t reg, uint8_t value)
{
    _wirePort->beginTransmission(_deviceAddress);
    _wirePort->write(reg);
    _wirePort->write(value);
    uint8_t status = _wirePort->endTransmission();

    if (status == 0)
    {
        // SerialStream->println("Write to register " + String(reg, HEX) + " successful.");
        return true;
    }
    else
    {
        // SerialStream->println("Write to register " + String(reg, HEX) + " failed.");
        return false;
    }
}

// Read from a register
uint8_t MyI2CDevice::readRegister(uint8_t reg)
{
    _wirePort->beginTransmission(_deviceAddress);
    _wirePort->write(reg);
    _wirePort->endTransmission(false);

    _wirePort->requestFrom(_deviceAddress, (uint8_t)1);
    if (_wirePort->available())
    {
        uint8_t value = _wirePort->read();
        _wirePort->endTransmission();
        // SerialStream->println("Read from register " + String(reg, HEX) + ": " + String(value, HEX));
        return value;
    }
    else
    {
        _wirePort->endTransmission();
        // SerialStream->println("Read from register " + String(reg, HEX) + " failed.");
        return 0;
    }
}