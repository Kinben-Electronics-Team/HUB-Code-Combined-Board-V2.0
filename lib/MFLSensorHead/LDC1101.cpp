#include "LDC1101.h"

Debugger::Debugger(bool enabled) : enabled(enabled) {}

void Debugger::setEnabled(bool en)
{
    enabled = en;
}

void Debugger::print(int value)
{
    if (enabled)
    {
        Serial.print(value);
    }
}

void Debugger::print(unsigned int value)
{
    if (enabled)
    {
        Serial.print(value);
    }
}

void Debugger::print(byte value)
{
    if (enabled)
    {
        Serial.print(value);
    }
}

void Debugger::print(int value, int format)
{
    if (enabled)
    {
        Serial.print(value, format);
    }
}

void Debugger::print(unsigned int value, int format)
{
    if (enabled)
    {
        Serial.print(value, format);
    }
}

void Debugger::print(float value, int Digits)
{
    if (enabled)
    {
        Serial.print(value, Digits);
    }
}

void Debugger::print(byte value, int format)
{
    if (enabled)
    {
        Serial.print(value, format);
    }
}

void Debugger::print(unsigned long value)
{
    if (enabled)
    {
        Serial.print(value);
    }
}

void Debugger::println(unsigned long value)
{
    if (enabled)
    {
        Serial.println(value);
    }
}

void Debugger::println(int value)
{
    if (enabled)
    {
        Serial.println(value);
    }
}

void Debugger::println(unsigned int value)
{
    if (enabled)
    {
        Serial.println(value);
    }
}

void Debugger::println(byte value)
{
    if (enabled)
    {
        Serial.println(value);
    }
}

void Debugger::println(int value, int format)
{
    if (enabled)
    {
        Serial.println(value, format);
    }
}

void Debugger::println(unsigned int value, int format)
{
    if (enabled)
    {
        Serial.println(value, format);
    }
}

void Debugger::println(byte value, int format)
{
    if (enabled)
    {
        Serial.println(value, format);
    }
}

void Debugger::print(const String &message)
{
    if (enabled)
    {
        Serial.print(message);
    }
}

void Debugger::println(const String &message)
{
    if (enabled)
    {
        Serial.println(message);
    }
}

Sensors::Sensors(ShiftRegister *sr, SPISettings spisetting, uint8_t num_sens, SPIClassRP2040 *spi)
    : SR(sr), _myspi(spi), SensorSPI(spisetting), num_sensors(num_sens), debug(false)
{
    if (!num_sens)
    {
        uint8_t BoardCsPinStart = 4;
        totalTMAG5170 = (SR->total_devices) * SENSOR_PER_EGP;
        for (int i = 0; i < totalTMAG5170; i++)
        {
            TMAG_CSpin[i] = BoardCsPinStart + i;
            if ((i + 1) % 4 == 0)
                BoardCsPinStart += 4;
        }
    }
}

void Sensors::enableDebugging(bool en)
{
    debug.setEnabled(en);
}

void Sensors::initSensor()
{
}

void Sensors::updateEGPCount(uint8_t num_EGP)
{
    uint8_t BoardCsPinStart = 4;
    const uint8_t TMAGperEGP = 4;
    if (num_EGP > MAX_NUM_ANG_TMAG)
    {
        Serial.println("Number of EGP sensors exceeds the limit");
        return;
    }

    SR->update_device_count(num_EGP);
    totalTMAG5170 = num_EGP * TMAGperEGP;
    /*get cs pin for tmag sensors*/
    for (int i = 0; i < totalTMAG5170; i++)
    {
        TMAG_CSpin[i] = BoardCsPinStart + i;
        if ((i + 1) % 4 == 0)
            BoardCsPinStart += 4;
    }
}

void Sensors::updateODOsensCount(uint8_t num_Sens)
{
    num_sensors = num_Sens;
}

void Sensors::LDC1101(int CS)
{
    if (numLDC1101 < totalLDC1101)
    {
        LDC_CSpin[numLDC1101++] = CS;
        pinMode(CS, OUTPUT);
        digitalWrite(CS, HIGH);
        // Additional initialization for LDC1101 as needed
    }
}

void Sensors::TMAG5170(int CS)
{
    if (numTMAG5170 < totalTMAG5170)
    {
        TMAG_CSpin[numTMAG5170++] = CS;
        pinMode(CS, OUTPUT);
        digitalWrite(CS, HIGH);
        // Additional initialization for Hall Effect sensor as needed
    }
}

bool Sensors::checkTMAG(uint8_t cs)
{
    uint8_t status = 0;
    bool SensorOK = 0;
    SR->SelectCS(cs);
    _myspi->transfer(writeReg | TEST_CONFIG);
    _myspi->transfer(0x00);
    _myspi->transfer(CRC_DIS_CRCdisabled);
    _myspi->transfer(0x07);
    SR->High_all_CS();
    delayMicroseconds(10);
    SR->SelectCS(cs);
    _myspi->transfer(readReg | TEST_CONFIG);
    _myspi->transfer(0x00);
    status = _myspi->transfer(0x00);
    debug.println(status, HEX);
    _myspi->transfer(0x00);
    SR->High_all_CS();
    status == 0x54 ? SensorOK = true : SensorOK = false;
    return SensorOK;
}

bool Sensors::checkLDC(uint8_t cs)
{
    uint8_t status = 0;
    bool SensorOK = 0;
    SR->SelectCS(cs);
    _myspi->transfer(readReg | LDC1101_REG_CHIP_ID);
    //_myspi->transfer(0x00) == 0xD4 ? SensorOK = true : SensorOK = false;

    status = _myspi->transfer(0x00);
    debug.println(status, HEX);
    status == 0xD4 ? SensorOK = true : SensorOK = false; //  Checks LDC Chip ID, should be D4 at 3F address
    return SensorOK;
}

void Sensors::configTMAG(uint8_t cs, angleAxis ang_axis, magAxisEN mag_axis)
{
    unsigned int data, recievedData;
    for (byte RegAddress = 0x0; RegAddress < 0x15; RegAddress++)
    {
        switch (RegAddress)
        {
        case /* 0x0 */ DEVICE_CONFIG:
            /* code */
            data = CONV_AVG_1x | MAG_TEMPCO_0R12pd | OPERATING_MODE_ConfigurationMode | T_CH_EN_TempChannelDisabled | T_RATE_sameAsOtherSensors | T_HLT_EN_tempLimitCheckOff | TEMP_COMP_EN_TempCompensationEnabled;
            debug.print("Data 0x");
            debug.println(data, HEX);
            SR->SelectCS(cs);
            _myspi->transfer(writeReg | RegAddress);
            _myspi->transfer((byte)(data >> 8));
            _myspi->transfer((byte)(data & 0x00ff));
            _myspi->transfer(0x00);
            SR->High_all_CS();
            delayMicroseconds(10);
            SR->SelectCS(cs);
            _myspi->transfer(readReg | RegAddress);
            debug.print((_myspi->transfer(0x00) == (byte)(data >> 8)) ? " OK" : " Error");
            debug.println((_myspi->transfer(0x00) == (byte)(data & 0x00ff)) ? " OK" : " Error");
            _myspi->transfer(0x00);
            SR->High_all_CS();
            break;
        case /* 0x1 */ SENSOR_CONFIG:
            /* code */
            data = ang_axis | SLEEPTIME_1ms | mag_axis | Z_RANGE_100mT | Y_RANGE_100mT | X_RANGE_100mT;
            debug.print("Data 0x");
            debug.println(data, HEX);
            SR->SelectCS(cs);
            _myspi->transfer(writeReg | RegAddress);
            _myspi->transfer((byte)(data >> 8));
            _myspi->transfer((byte)(data & 0x00ff));
            _myspi->transfer(0x00);
            SR->High_all_CS();
            delayMicroseconds(10);
            SR->SelectCS(cs);
            _myspi->transfer(readReg | RegAddress);
            debug.print((_myspi->transfer(0x00) == (byte)(data >> 8)) ? " OK" : " Error");
            debug.println((_myspi->transfer(0x00) == (byte)(data & 0x00ff)) ? " OK" : " Error");
            _myspi->transfer(0x00);
            SR->High_all_CS();
            break;
        case /* 0x2 */ SYSTEM_CONFIG:
            /* code */
            data = DIAG_SEL_enabledDataPath | TRIGGER_MODE_SPI | DATA_TYPE_32bit | DIAG_EN_AFEdiagnosticsDisabled | Z_HLT_EN_ZaxisLimitCheckoff | Y_HLT_EN_YaxisLimitCheckoff | X_HLT_EN_XaxisLimitCheckoff;
            debug.print("Data 0x");
            debug.println(data, HEX);
            SR->SelectCS(cs);
            _myspi->transfer(writeReg | RegAddress);
            _myspi->transfer((byte)(data >> 8));
            _myspi->transfer((byte)(data & 0x00ff));
            _myspi->transfer(0x00);
            SR->High_all_CS();
            delayMicroseconds(10);
            SR->SelectCS(cs);
            _myspi->transfer(readReg | RegAddress);
            debug.print((_myspi->transfer(0x00) == (byte)(data >> 8)) ? " OK" : " Error");
            debug.println((_myspi->transfer(0x00) == (byte)(data & 0x00ff)) ? " OK" : " Error");
            _myspi->transfer(0x00);
            SR->High_all_CS();
            break;

        default:
            break;
        }
    }
}

void Sensors::configLDC(uint8_t cs)
{
    for (byte RegAddress = 0x01; RegAddress <= 0x3F; RegAddress++)
    {

        switch (RegAddress)
        {
        case /*0x01*/ LDC1101_REG_CFG_RP_MEASUREMENT_DYNAMIC_RANGE:
            /* code */
            debug.print("0x");
            debug.print(RegAddress, HEX);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(writeReg | RegAddress);
            _myspi->transfer(0x34); // 27
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            delayMicroseconds(1);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(readReg | RegAddress);
            debug.println((_myspi->transfer(0x00) == 0x34) ? " - OK" : " - Error");
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();

            break;

        case /*0x02*/ LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_1:
            /* code */
            debug.print("0x");
            debug.print(RegAddress, HEX);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(writeReg | RegAddress);
            _myspi->transfer(0xDE); // D0
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            delayMicroseconds(1);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(readReg | RegAddress);
            debug.println((_myspi->transfer(0x00) == 0xDE) ? " - OK" : " - Error");
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            break;

        case /*0x03*/ LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_2:
            /* code */
            debug.print("0x");
            debug.print(RegAddress, HEX);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(writeReg | RegAddress);
            _myspi->transfer(0xF9); // E0
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            delayMicroseconds(1);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(readReg | RegAddress);
            debug.println((_myspi->transfer(0x00) == 0xF9) ? " - OK" : " - Error");
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            break;

        case /*0x04*/ LDC1101_REG_CFG_RPL_CONVERSION_INTERVAL:
            /* code */
            debug.print("0x");
            debug.print(RegAddress, HEX);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(writeReg | RegAddress);
            _myspi->transfer(0xE2); // 04
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            delayMicroseconds(1);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(readReg | RegAddress);
            debug.println((_myspi->transfer(0x00) == 0xE2) ? " - OK" : " - Error");
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            break;

        case /*0x05*/ LDC1101_REG_CFG_ADDITIONAL_DEVICE:
            /* code */
            debug.print("0x");
            debug.print(RegAddress, HEX);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(writeReg | RegAddress);
            _myspi->transfer(0x01);
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            delayMicroseconds(1);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(readReg | RegAddress);
            debug.println((_myspi->transfer(0x00) == 0x01) ? " - OK" : " - Error");
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            break;

        case /*0x0C*/ LDC1101_REG_AMPLITUDE_CONTROL_REQUIREMENT:
            /* code */
            debug.print("0x");
            debug.print(RegAddress, HEX);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(writeReg | RegAddress);
            _myspi->transfer(0x01);
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            delayMicroseconds(1);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(readReg | RegAddress);
            debug.println((_myspi->transfer(0x00) == 0x01) ? " - OK" : " - Error");
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            break;

        case /*0x30*/ LDC1101_REGLHR_RCOUNTLSB:
            /* code */
            debug.print("0x");
            debug.print(RegAddress, HEX);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(writeReg | RegAddress);
            _myspi->transfer(0x32); // 40
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            delayMicroseconds(1);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(readReg | RegAddress);
            debug.println((_myspi->transfer(0x00) == 0x32) ? " - OK" : " - Error");
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            break;

        case /*0x31*/ LDC1101_REGLHR_RCOUNT_MSB:
            /* code */
            debug.print("0x");
            debug.print(RegAddress, HEX);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(writeReg | RegAddress);
            _myspi->transfer(0x00);
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            delayMicroseconds(1);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(readReg | RegAddress);
            debug.println((_myspi->transfer(0x00) == 0x00) ? " - OK" : " - Error");
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            break;

        case /*0x34*/ LDC1101_REG_CFGLHR:
            /* code */
            debug.print("0x");
            debug.print(RegAddress, HEX);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(writeReg | RegAddress);
            _myspi->transfer(0x01);
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            delayMicroseconds(1);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(readReg | RegAddress);
            debug.println((_myspi->transfer(0x00) == 0x01) ? " - OK" : " - Error");
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            break;
        case /*0x32 - special case as LSB can only be programmed after msb*/ LDC1101_REGLHR_OFFSETLSB:
            /* code */
            debug.print("0x");
            debug.print(LDC1101_REGLHR_RCOUNTLSB, HEX);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(writeReg | LDC1101_REGLHR_RCOUNTLSB);
            _myspi->transfer(0x32); // 40
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            delayMicroseconds(1);
            // //digitalWrite(LDC_CSpin[i], LOW);
            SR->SelectCS(cs);
            _myspi->transfer(readReg | LDC1101_REGLHR_RCOUNTLSB);
            // debug.print(" last");
            debug.println((_myspi->transfer(0x00) == 0x32) ? " - OK" : " - Error");
            // //digitalWrite(LDC_CSpin[i], HIGH);
            SR->High_all_CS();
            break;

        default:
            break;
        }
    }
}

bool Sensors::CheckMFLSensors()
{
    bool SensorOK = 0;
    _myspi->beginTransaction(SensorSPI);
    debug.println("LDC COM check:- ");
    for (uint8_t i = CSstartPin; i < CSstartPin + numLDC; i++)
    {
        SensorOK = checkLDC(i);
    }
    debug.println("TMAG COM check:- ");
    for (uint8_t i = CSstartPin + numLDC; i < CSstartPin + numLDC + numTMAG; i++)
    {
        // Disable CRC
        SensorOK = checkTMAG(i);
    }

    _myspi->endTransaction();
    return SensorOK;
}

bool Sensors::CheckODOSensors()
{
    bool SensorOK = 0;
    _myspi->beginTransaction(SensorSPI);

    debug.println("TMAG COM check:- ");
    for (int i = 0; i < num_sensors; i++)
    {
        // Disable CRC
        SensorOK = checkTMAG(i);
    }

    _myspi->endTransaction();
    return SensorOK;
}
// Check Communication with all sensors
bool Sensors::CheckEGPSensors()
{
    _myspi->beginTransaction(SensorSPI);
    bool SensorOK = 0;
    debug.println("TMAG COM check:- ");
    for (int i = 0; i < totalTMAG5170; i++)
    {
        // Disable CRC
        SensorOK = checkTMAG(TMAG_CSpin[i]);
    }
    _myspi->endTransaction();
    return SensorOK;
}

void Sensors::configMFL(magAxisEN axis_en)
{
    bool SensorOK = 0;
    debug.println("Configuring Sensors - LDC1101s");
    _myspi->beginTransaction(SensorSPI);

    for (uint8_t i = CSstartPin; i < CSstartPin + numLDC; i++)
    {
        debug.print("LDC1101 ");
        debug.println(i - CSstartPin);
        configLDC(i);
    }

    debug.println("Configuring Sensors - TMAG5170s");
    for (uint8_t i = CSstartPin + numLDC; i < CSstartPin + numLDC + numTMAG; i++)
    {
        debug.print("TMAG ");
        debug.println(i - CSstartPin + numLDC);
        configTMAG(i, NO_ANGLE, axis_en);
    }

    _myspi->endTransaction();
}

void Sensors::configODO(angleAxis ang_axis, magAxisEN axis_en)
{
    _myspi->beginTransaction(SensorSPI);
    debug.println("Configuring Sensors - TMAG5170s");
    for (uint8_t i = 0; i < num_sensors; i++)
    {
        debug.print("TMAG ");
        debug.println(i);
        configTMAG(i, ang_axis, axis_en);
    }
    _myspi->endTransaction();
}

void Sensors::configEGP(angleAxis ang_axis, magAxisEN axis_en)
{
    _myspi->beginTransaction(SensorSPI);
    debug.println("Configuring Sensors - TMAG5170s");
    for (int i = 0; i < totalTMAG5170; i++)
    {
        configTMAG(TMAG_CSpin[i], ang_axis, axis_en);
    }
    _myspi->endTransaction();
}
// Configure and Diagnose all Sensors
void Sensors::ConfigureSensors()
{
    // debug.setEnabled(1);
    debug.println("Configuring Sensors - LDC1101s");
    _myspi->beginTransaction(SensorSPI);
    bool SensorOK = 0;
    // LDC Sensors
    for (int i = 0; i < totalLDC1101; i++)
    {
        debug.print("LDC1101 ");
        debug.println(i);
        // Configure LDC Sensors and Diagnose them
        for (byte RegAddress = 0x01; RegAddress <= 0x3F; RegAddress++)
        {

            switch (RegAddress)
            {
            case /*0x01*/ LDC1101_REG_CFG_RP_MEASUREMENT_DYNAMIC_RANGE:
                /* code */
                debug.print("0x");
                debug.print(RegAddress, HEX);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(writeReg | RegAddress);
                _myspi->transfer(0x34); // 27
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                delayMicroseconds(1);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(readReg | RegAddress);
                debug.println((_myspi->transfer(0x00) == 0x34) ? " - OK" : " - Error");
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();

                break;

            case /*0x02*/ LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_1:
                /* code */
                debug.print("0x");
                debug.print(RegAddress, HEX);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(writeReg | RegAddress);
                _myspi->transfer(0xDE); // D0
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                delayMicroseconds(1);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(readReg | RegAddress);
                debug.println((_myspi->transfer(0x00) == 0xDE) ? " - OK" : " - Error");
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                break;

            case /*0x03*/ LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_2:
                /* code */
                debug.print("0x");
                debug.print(RegAddress, HEX);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(writeReg | RegAddress);
                _myspi->transfer(0xF9); // E0
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                delayMicroseconds(1);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(readReg | RegAddress);
                debug.println((_myspi->transfer(0x00) == 0xF9) ? " - OK" : " - Error");
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                break;

            case /*0x04*/ LDC1101_REG_CFG_RPL_CONVERSION_INTERVAL:
                /* code */
                debug.print("0x");
                debug.print(RegAddress, HEX);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(writeReg | RegAddress);
                _myspi->transfer(0xE2); // 04
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                delayMicroseconds(1);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(readReg | RegAddress);
                debug.println((_myspi->transfer(0x00) == 0xE2) ? " - OK" : " - Error");
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                break;

            case /*0x05*/ LDC1101_REG_CFG_ADDITIONAL_DEVICE:
                /* code */
                debug.print("0x");
                debug.print(RegAddress, HEX);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(writeReg | RegAddress);
                _myspi->transfer(0x01);
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                delayMicroseconds(1);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(readReg | RegAddress);
                debug.println((_myspi->transfer(0x00) == 0x01) ? " - OK" : " - Error");
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                break;

            case /*0x0C*/ LDC1101_REG_AMPLITUDE_CONTROL_REQUIREMENT:
                /* code */
                debug.print("0x");
                debug.print(RegAddress, HEX);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(writeReg | RegAddress);
                _myspi->transfer(0x01);
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                delayMicroseconds(1);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(readReg | RegAddress);
                debug.println((_myspi->transfer(0x00) == 0x01) ? " - OK" : " - Error");
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                break;

            case /*0x30*/ LDC1101_REGLHR_RCOUNTLSB:
                /* code */
                debug.print("0x");
                debug.print(RegAddress, HEX);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(writeReg | RegAddress);
                _myspi->transfer(0x32); // 40
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                delayMicroseconds(1);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(readReg | RegAddress);
                debug.println((_myspi->transfer(0x00) == 0x32) ? " - OK" : " - Error");
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                break;

            case /*0x31*/ LDC1101_REGLHR_RCOUNT_MSB:
                /* code */
                debug.print("0x");
                debug.print(RegAddress, HEX);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(writeReg | RegAddress);
                _myspi->transfer(0x00);
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                delayMicroseconds(1);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(readReg | RegAddress);
                debug.println((_myspi->transfer(0x00) == 0x00) ? " - OK" : " - Error");
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                break;

            case /*0x34*/ LDC1101_REG_CFGLHR:
                /* code */
                debug.print("0x");
                debug.print(RegAddress, HEX);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(writeReg | RegAddress);
                _myspi->transfer(0x01);
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                delayMicroseconds(1);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(readReg | RegAddress);
                debug.println((_myspi->transfer(0x00) == 0x01) ? " - OK" : " - Error");
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                break;
            case /*0x32 - special case as LSB can only be programmed after msb*/ LDC1101_REGLHR_OFFSETLSB:
                /* code */
                debug.print("0x");
                debug.print(LDC1101_REGLHR_RCOUNTLSB, HEX);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(writeReg | LDC1101_REGLHR_RCOUNTLSB);
                _myspi->transfer(0x32); // 40
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                delayMicroseconds(1);
                // //digitalWrite(LDC_CSpin[i], LOW);
                SR->SelectCS(LDC_CSpin[i]);
                _myspi->transfer(readReg | LDC1101_REGLHR_RCOUNTLSB);
                // debug.print(" last");
                debug.println((_myspi->transfer(0x00) == 0x32) ? " - OK" : " - Error");
                // //digitalWrite(LDC_CSpin[i], HIGH);
                SR->High_all_CS();
                break;

            default:
                break;
            }
        }
    }
    debug.println("Configuring Sensors - TMAG5170s");
    // TMAG Sensors --- This ispending
    for (int i = 0; i < totalTMAG5170; i++)
    {
        unsigned int data, recievedData;
        for (byte RegAddress = 0x0; RegAddress < 0x15; RegAddress++)
        {
            switch (RegAddress)
            {
            case /* 0x0 */ DEVICE_CONFIG:
                /* code */
                data = CONV_AVG_1x | MAG_TEMPCO_0R12pd | OPERATING_MODE_ConfigurationMode | T_CH_EN_TempChannelDisabled | T_RATE_sameAsOtherSensors | T_HLT_EN_tempLimitCheckOff | TEMP_COMP_EN_TempCompensationEnabled;
                debug.print("Data 0x");
                debug.println(data, HEX);
                // digitalWrite(TMAG_CSpin[i], LOW);
                SR->SelectCS(TMAG_CSpin[i]);
                _myspi->transfer(writeReg | RegAddress);
                _myspi->transfer((byte)(data >> 8));
                _myspi->transfer((byte)(data & 0x00ff));
                _myspi->transfer(0x00);
                ////digitalWrite(TMAG_CSpin[i], HIGH);
                SR->High_all_CS();
                delayMicroseconds(10);
                // digitalWrite(TMAG_CSpin[i], LOW);
                SR->SelectCS(TMAG_CSpin[i]);
                _myspi->transfer(readReg | RegAddress);
                debug.print((_myspi->transfer(0x00) == (byte)(data >> 8)) ? " OK" : " Error");
                debug.println((_myspi->transfer(0x00) == (byte)(data & 0x00ff)) ? " OK" : " Error");
                _myspi->transfer(0x00);
                ////digitalWrite(TMAG_CSpin[i], HIGH);
                SR->High_all_CS();
                break;
            case /* 0x1 */ SENSOR_CONFIG:
                /* code */
                data = /*ANGLE_EN_NoAngleCalculation*/ ANGLE_EN_X_Y | SLEEPTIME_1ms | MAG_CH_EN_XYZenabled | Z_RANGE_100mT | Y_RANGE_100mT | X_RANGE_100mT;
                debug.print("Data 0x");
                debug.println(data, HEX);
                // digitalWrite(TMAG_CSpin[i], LOW);
                SR->SelectCS(TMAG_CSpin[i]);
                _myspi->transfer(writeReg | RegAddress);
                _myspi->transfer((byte)(data >> 8));
                _myspi->transfer((byte)(data & 0x00ff));
                _myspi->transfer(0x00);
                ////digitalWrite(TMAG_CSpin[i], HIGH);
                SR->High_all_CS();
                delayMicroseconds(10);
                // digitalWrite(TMAG_CSpin[i], LOW);
                SR->SelectCS(TMAG_CSpin[i]);
                _myspi->transfer(readReg | RegAddress);
                debug.print((_myspi->transfer(0x00) == (byte)(data >> 8)) ? " OK" : " Error");
                debug.println((_myspi->transfer(0x00) == (byte)(data & 0x00ff)) ? " OK" : " Error");
                _myspi->transfer(0x00);
                ////digitalWrite(TMAG_CSpin[i], HIGH);
                SR->High_all_CS();
                break;
            case /* 0x2 */ SYSTEM_CONFIG:
                /* code */
                data = DIAG_SEL_enabledDataPath | TRIGGER_MODE_SPI | DATA_TYPE_32bit | DIAG_EN_AFEdiagnosticsDisabled | Z_HLT_EN_ZaxisLimitCheckoff | Y_HLT_EN_YaxisLimitCheckoff | X_HLT_EN_XaxisLimitCheckoff;
                debug.print("Data 0x");
                debug.println(data, HEX);
                // digitalWrite(TMAG_CSpin[i], LOW);
                SR->SelectCS(TMAG_CSpin[i]);
                _myspi->transfer(writeReg | RegAddress);
                _myspi->transfer((byte)(data >> 8));
                _myspi->transfer((byte)(data & 0x00ff));
                _myspi->transfer(0x00);
                // //digitalWrite(TMAG_CSpin[i], HIGH);
                SR->High_all_CS();
                delayMicroseconds(10);
                // digitalWrite(TMAG_CSpin[i], LOW);
                SR->SelectCS(TMAG_CSpin[i]);
                _myspi->transfer(readReg | RegAddress);
                debug.print((_myspi->transfer(0x00) == (byte)(data >> 8)) ? " OK" : " Error");
                debug.println((_myspi->transfer(0x00) == (byte)(data & 0x00ff)) ? " OK" : " Error");
                _myspi->transfer(0x00);
                ////digitalWrite(TMAG_CSpin[i], HIGH);
                SR->High_all_CS();
                break;

            default:
                break;
            }
        }
    }

    _myspi->endTransaction();
}

void Sensors ::angleOffsetConfig(unsigned int Cspin)
{

    uint16_t set_val = 0;
    uint16_t reset_val = 0;
    _myspi->beginTransaction(SensorSPI);
    SR->SelectCS(TMAG_CSpin[Cspin]);
    // _myspi->transfer(readReg | SENSOR_CONFIG);
    // set_val = _myspi->transfer16(0x0000);
    // debug.println(set_val, HEX);
    // SR->High_all_CS();
    // reset_val = set_val | (ANGLE_EN_X_Y);
    // debug.println(reset_val, HEX);
    // SR->SelectCS(TMAG_CSpin[Cspin]);
    _myspi->transfer(writeReg | 0x12);
    _myspi->transfer16((0x2 << 14) | (0xa << 7));
    SR->High_all_CS();
    _myspi->endTransaction();
}

void Sensors ::runMFL(TMAGdataType dataType)
{

    _myspi->beginTransaction(SensorSPI);
    debug.println("LDC1101 Run Mode:- ");

    for (uint8_t i = CSstartPin; i < CSstartPin + numLDC; i++)
    {
        debug.print("LDC1101 ");
        debug.println(i - CSstartPin);
        PutLDCInRunMode(i);
    }

    debug.println("TMAG RUN MODE:- ");
    for (uint8_t i = CSstartPin + numLDC; i < CSstartPin + numLDC + numTMAG; i++)
    {
        debug.print("TMAG ");
        debug.println(i - CSstartPin + numLDC);
        PutTMAGInRunMode(i, dataType);
    }

    //_myspi->endTransaction();
}

void Sensors ::runODO(TMAGdataType dataType)
{
    _myspi->beginTransaction(SensorSPI);

    debug.println("ODO RUN MODE:- ");
    for (int i = 0; i < num_sensors; i++)
    {
        debug.print("TMAG ");
        debug.println(i);
        if (i % 2 == 0)
        {
            PutTMAGInRunMode(i, dataType); // magnetic sensor
        }
        else
        {
            PutTMAGInRunMode(i, bit32); // Angle sensor
        }
    }

    _myspi->endTransaction();
}

void Sensors ::runEGP(TMAGdataType dataType)
{
    for (int i = 0; i < totalTMAG5170; i++)
    {
        if (i % 4 == 0)
            PutTMAGInRunMode(TMAG_CSpin[i], bit32);
        else
            PutTMAGInRunMode(TMAG_CSpin[i], dataType);
    }
}

void Sensors ::PutLDCInRunMode(uint8_t cs)
{
    debug.print("0x");
    debug.print(LDC1101_REG_CFG_POWER_STATE, HEX);
    ////digitalWrite(LDC_CSpin[i], LOW);
    SR->SelectCS(cs);
    _myspi->transfer(writeReg | LDC1101_REG_CFG_POWER_STATE);
    _myspi->transfer(0x00);
    // digitalWrite(LDC_CSpin[i], HIGH);
    SR->High_all_CS();
    delayMicroseconds(1);
    // digitalWrite(LDC_CSpin[i], LOW);
    SR->SelectCS(cs);
    _myspi->transfer(readReg | LDC1101_REG_CFG_POWER_STATE);
    debug.println((_myspi->transfer(0x00) == 0x00) ? " - OK" : " - Error");
    // digitalWrite(LDC_CSpin[i], HIGH);
    SR->High_all_CS();
}

void Sensors ::PutTMAGInRunMode(uint8_t cs, TMAGdataType DataType)
{
    unsigned int data;
    debug.println("set data type:");
    data = DIAG_SEL_enabledDataPath | TRIGGER_MODE_SPI | DataType | DIAG_EN_AFEdiagnosticsDisabled | Z_HLT_EN_ZaxisLimitCheckoff | Y_HLT_EN_YaxisLimitCheckoff | X_HLT_EN_XaxisLimitCheckoff;
    SR->SelectCS(cs);
    _myspi->transfer(writeReg | SYSTEM_CONFIG);
    _myspi->transfer((byte)(data >> 8));
    _myspi->transfer((byte)(data & 0x00ff));
    _myspi->transfer(0x00);
    SR->High_all_CS();

    // Check if Run Mode Enable
    SR->SelectCS(cs);
    _myspi->transfer(readReg | SYSTEM_CONFIG);
    debug.print((_myspi->transfer(0x00) == (byte)(data >> 8)) ? " - OK" : " - Error");
    debug.println((_myspi->transfer(0x00) == (byte)(data & 0x00ff)) ? " - OK" : " - Error");
    _myspi->transfer(0x0F);
    SR->High_all_CS();

    data = CONV_AVG_1x | MAG_TEMPCO_0R12pd | OPERATING_MODE_activeMeasureMode | T_CH_EN_TempChannelDisabled | T_RATE_sameAsOtherSensors | T_HLT_EN_tempLimitCheckOff | TEMP_COMP_EN_TempCompensationDisenabled;
    SR->SelectCS(cs);
    _myspi->transfer(writeReg | DEVICE_CONFIG);
    _myspi->transfer((byte)(data >> 8));
    _myspi->transfer((byte)(data & 0x00ff));
    _myspi->transfer(0x0F);
    SR->High_all_CS();
}

void Sensors::putSensorsInRunMode(bool Special32BitEN)
{
    // Put LDC1101 in Run Mode
    // debug.setEnabled(1);
    _myspi->beginTransaction(SensorSPI);
    debug.println("LDC1101 Run Mode:- ");
    for (int i = 0; i < totalLDC1101; i++)
    {
        debug.print("0x");
        debug.print(LDC1101_REG_CFG_POWER_STATE, HEX);
        ////digitalWrite(LDC_CSpin[i], LOW);
        SR->SelectCS(LDC_CSpin[i]);
        _myspi->transfer(writeReg | LDC1101_REG_CFG_POWER_STATE);
        _myspi->transfer(0x00);
        // digitalWrite(LDC_CSpin[i], HIGH);
        SR->High_all_CS();
        delayMicroseconds(1);
        // digitalWrite(LDC_CSpin[i], LOW);
        SR->SelectCS(LDC_CSpin[i]);
        _myspi->transfer(readReg | LDC1101_REG_CFG_POWER_STATE);
        debug.println((_myspi->transfer(0x00) == 0x00) ? " - OK" : " - Error");
        // digitalWrite(LDC_CSpin[i], HIGH);
        SR->High_all_CS();
    }
    // Put TMAG5170 in Run Mode and in 32 Bit Special Write Mode

    for (int i = 0; i < totalTMAG5170; i++)
    {

        if (Special32BitEN) // Enable or Disable 32 Bit Special Read
        {
            debug.println("TMAG5170 Special Mode");
            unsigned int data;
            if (i % 4 == 0)
                data = DIAG_SEL_enabledDataPath | TRIGGER_MODE_SPI | DATA_TYPE_32bit | DIAG_EN_AFEdiagnosticsDisabled | Z_HLT_EN_ZaxisLimitCheckoff | Y_HLT_EN_YaxisLimitCheckoff | X_HLT_EN_XaxisLimitCheckoff;
            else
                data = DIAG_SEL_enabledDataPath | TRIGGER_MODE_SPI | DATA_TYPE_12bit_XZ | DIAG_EN_AFEdiagnosticsDisabled | Z_HLT_EN_ZaxisLimitCheckoff | Y_HLT_EN_YaxisLimitCheckoff | X_HLT_EN_XaxisLimitCheckoff;

            // digitalWrite(TMAG_CSpin[i], LOW);
            SR->SelectCS(TMAG_CSpin[i]);
            _myspi->transfer(writeReg | SYSTEM_CONFIG);
            _myspi->transfer((byte)(data >> 8));
            _myspi->transfer((byte)(data & 0x00ff));
            _myspi->transfer(0x00);
            // //digitalWrite(TMAG_CSpin[i], HIGH);
            SR->High_all_CS();
            delayMicroseconds(1);
            // Check if Run Mode Enable
        }
        else
        {
            debug.println("TMAG5170 Normal Mode");
            unsigned int data;
            data = DIAG_SEL_enabledDataPath | TRIGGER_MODE_SPI | DATA_TYPE_32bit | DIAG_EN_AFEdiagnosticsDisabled | Z_HLT_EN_ZaxisLimitCheckoff | Y_HLT_EN_YaxisLimitCheckoff | X_HLT_EN_XaxisLimitCheckoff;
            // digitalWrite(TMAG_CSpin[i], LOW);
            SR->SelectCS(TMAG_CSpin[i]);
            _myspi->transfer(writeReg | SYSTEM_CONFIG);
            _myspi->transfer((byte)(data >> 8));
            _myspi->transfer((byte)(data & 0x00ff));
            _myspi->transfer(0x00);
            // digitalWrite(TMAG_CSpin[i], HIGH);
            SR->High_all_CS();
            delayMicroseconds(1);
            // Check if Run Mode Enable
            // digitalWrite(TMAG_CSpin[i], LOW);
            SR->SelectCS(TMAG_CSpin[i]);
            _myspi->transfer(readReg | SYSTEM_CONFIG);
            debug.print((_myspi->transfer(0x00) == (byte)(data >> 8)) ? " - OK" : " - Error");
            debug.println((_myspi->transfer(0x00) == (byte)(data & 0x00ff)) ? " - OK" : " - Error");
            _myspi->transfer(0x0F);
            // digitalWrite(TMAG_CSpin[i], HIGH);
            SR->High_all_CS();
        }
        // Put in Run Mode
        debug.println("TMAG5170 Run Mode:- ");
        unsigned int data;
        data = CONV_AVG_1x | MAG_TEMPCO_0R12pd | OPERATING_MODE_activeMeasureMode | T_CH_EN_TempChannelDisabled | T_RATE_sameAsOtherSensors | T_HLT_EN_tempLimitCheckOff | TEMP_COMP_EN_TempCompensationDisenabled;
        // digitalWrite(TMAG_CSpin[i], LOW);
        SR->SelectCS(TMAG_CSpin[i]);
        _myspi->transfer(writeReg | DEVICE_CONFIG);
        _myspi->transfer((byte)(data >> 8));
        _myspi->transfer((byte)(data & 0x00ff));
        _myspi->transfer(0x0F);
        // digitalWrite(TMAG_CSpin[i], HIGH);
        SR->High_all_CS();
    }
    // _myspi->endTransaction();
}

void Sensors::putSensorsInSleep(bool Special32BitEN)
{
    // Put LDC1101 in Sleep Mode
    // debug.setEnabled(1);
    _myspi->beginTransaction(SensorSPI);
    debug.println("LDC1101 Sleep Mode:- ");
    for (int i = 0; i < totalLDC1101; i++)
    {
        debug.print("0x");
        debug.print(LDC1101_REG_CFG_POWER_STATE, HEX);
        // digitalWrite(LDC_CSpin[i], LOW);
        SR->SelectCS(TMAG_CSpin[i]);
        _myspi->transfer(writeReg | LDC1101_REG_CFG_POWER_STATE);
        _myspi->transfer(0x01);
        // digitalWrite(LDC_CSpin[i], HIGH);
        SR->High_all_CS();
        delayMicroseconds(1);
        // digitalWrite(LDC_CSpin[i], LOW);
        SR->SelectCS(TMAG_CSpin[i]);
        _myspi->transfer(readReg | LDC1101_REG_CFG_POWER_STATE);
        debug.println((_myspi->transfer(0x00) == 0x01) ? " - OK" : " - Error");
        // digitalWrite(LDC_CSpin[i], HIGH);
        SR->High_all_CS();
    }
    // Put TMAG5170 in Run Mode and in 32 Bit Special Write Mode

    for (int i = 0; i < totalTMAG5170; i++)
    {
        debug.println("TMAG5170 Normal Data Mode First");
        unsigned int data;
        data = DIAG_SEL_enabledDataPath | TRIGGER_MODE_SPI | DATA_TYPE_32bit | DIAG_EN_AFEdiagnosticsDisabled | Z_HLT_EN_ZaxisLimitCheckoff | Y_HLT_EN_YaxisLimitCheckoff | X_HLT_EN_XaxisLimitCheckoff;
        // digitalWrite(TMAG_CSpin[i], LOW);
        SR->SelectCS(TMAG_CSpin[i]);
        _myspi->transfer(writeReg | SYSTEM_CONFIG);
        _myspi->transfer((byte)(data >> 8));
        _myspi->transfer((byte)(data & 0x00ff));
        _myspi->transfer(0x00);
        // digitalWrite(TMAG_CSpin[i], HIGH);
        SR->High_all_CS();
        delayMicroseconds(1);
        // Check if Run Mode Enable
        // digitalWrite(TMAG_CSpin[i], LOW);
        SR->SelectCS(TMAG_CSpin[i]);
        _myspi->transfer(readReg | SYSTEM_CONFIG);
        debug.print((_myspi->transfer(0x00) == (byte)(data >> 8)) ? " - OK" : " - Error");
        debug.println((_myspi->transfer(0x00) == (byte)(data & 0x00ff)) ? " - OK" : " - Error");
        _myspi->transfer(0x0F);
        // digitalWrite(TMAG_CSpin[i], HIGH);
        SR->High_all_CS();
        // Put in Run Mode
        debug.println("TMAG5170 Sleep Mode:- ");
        data = CONV_AVG_1x | MAG_TEMPCO_0R12pd | OPERATING_MODE_SleepMode | T_CH_EN_TempChannelDisabled | T_RATE_sameAsOtherSensors | T_HLT_EN_tempLimitCheckOff | TEMP_COMP_EN_TempCompensationDisenabled;
        // digitalWrite(TMAG_CSpin[i], LOW);
        SR->SelectCS(TMAG_CSpin[i]);
        _myspi->transfer(writeReg | DEVICE_CONFIG);
        _myspi->transfer((byte)(data >> 8));
        _myspi->transfer((byte)(data & 0x00ff));
        _myspi->transfer(0x0F);
        // digitalWrite(TMAG_CSpin[i], HIGH);
        SR->High_all_CS();
        delayMicroseconds(1);
        // Check if Run Mode Enable
        // digitalWrite(TMAG_CSpin[i], LOW);
        SR->SelectCS(TMAG_CSpin[i]);
        _myspi->transfer(readReg | DEVICE_CONFIG);
        debug.print((_myspi->transfer(0x00) == (byte)(data >> 8)) ? " - OK" : " - Error");
        debug.println((_myspi->transfer(0x00) == (byte)(data & 0x00ff)) ? " - OK" : " - Error");
        _myspi->transfer(0x0F);
        // digitalWrite(TMAG_CSpin[i], HIGH);
        SR->High_all_CS();
    }
    _myspi->endTransaction();
}

void Sensors::livereadMFL(uint8_t Channel, bool special32bitEN)
{
    _myspi->beginTransaction(SensorSPI);
    SR->StoreOneLOWbit();
    SR->ShiftCS();
    SR->ShiftCS();
    
    // Clear screen and display header
    debug.print("\033[2J\033[H"); // Clear screen and move cursor to top
    debug.println("╔═══════════════════════════════════════════════════════════════════╗");
    debug.println("║                          MFL SENSOR DATA                         ║");
    debug.println("╠═══════════════════════════════════════════════════════════════════╣");
    debug.println("║                      LDC SENSORS (Inductance)                    ║");
    debug.println("╠═══════════════════════════════════════════════════════════════════╣");
    
    // Display LDC data in formatted table
    for (int i = 0; i < numLDC; i++)
    {
        SR->ShiftCS();
        uint32_t ldcValue = liveReadLDC();
        debug.printf("║  LDC %d │ %10lu │ [", i, ldcValue);
        
        // Visual bar representation
        int barLength = (ldcValue % 1000) / 50; // Scale for visual bar
        for(int j = 0; j < 20; j++) {
            if(j < barLength) debug.print("█");
            else debug.print("░");
        }
        debug.println("] ║");
    }
    
    debug.println("╠═══════════════════════════════════════════════════════════════════╣");
    debug.println("║                    TMAG SENSORS (Magnetic)                       ║");
    debug.println("╠═══════════════════════════════════════════════════════════════════╣");
    
    // Display TMAG data in formatted table
    for (int i = 0; i < numTMAG; i++)
    {
        SR->ShiftCS();
        debug.printf("║  TMAG %d │ ", i);
        LiveReadTMAGFormatted(Channel, special32bitEN);
        debug.println(" ║");
    }
    
    SR->High_all_CS();
    debug.println("╚═══════════════════════════════════════════════════════════════════╝");
    debug.printf("║ Timestamp: %10lu ms                                       ║\n", millis());
    debug.println("╚═══════════════════════════════════════════════════════════════════╝");
    _myspi->endTransaction();
}

void Sensors::liveReadODO(uint8_t Channel, bool special32bitEN)
{
    _myspi->beginTransaction(SensorSPI);
    SR->StoreOneLOWbit();
    thxStat = 0;
    for (int i = 0; i < num_sensors; i++)
    {
        SR->ShiftCS();

        debug.print("TMAG ");
        debug.print(i);
        debug.print("   ");
        // LiveReadTMAG(Channel, special32bitEN);
        if (i % 2 == 0)
        {
            LiveReadTMAG(Channel, special32bitEN); // live read magnetic sensor
        }
        else
        {
            debug.print(liveReadAngle(), 2); // live read angle sensor
            debug.println("");
        }
    }
    debug.print("TMAG status :");
    debug.println(thxStat, HEX);

    // debug.println("");
    //  SR->StoreOneLOWbit();
    //  for (int i = 0; i < num_sensors; i++)
    //  {
    //      SR->ShiftCS();
    //      debug.print("TMAG status");
    //      debug.print(i);
    //      debug.print("   ");

    //     _myspi->transfer(readReg | SYS_STATUS);
    //     debug.print(_myspi->transfer16(0x00), HEX);
    //     debug.print("   ");
    // }
    SR->ShiftCS();
    debug.println("\n");
    _myspi->endTransaction();
}

void Sensors::liveReadEGP(uint8_t Channel, bool special32bitEN)
{
    _myspi->beginTransaction(SensorSPI);
    SR->StoreOneLOWbit();
    
    // Clear screen and display header
    debug.print("\033[2J\033[H"); // Clear screen and move cursor to top
    debug.println("╔══════════════════════════════════════════════════════════════════════╗");
    debug.println("║                           EGP SENSOR DATA                           ║");
    debug.println("╠══════════════════════════════════════════════════════════════════════╣");
    debug.println("║ Sensor │    Type    │        Value        │ Status │     Visual     ║");
    debug.println("╠══════════════════════════════════════════════════════════════════════╣");
    
    for (int i = 0; i < totalTMAG5170; i++)
    {
        debug.printf("║ TMAG%-2d │", i);
        
        if (i % 4 == 0) // Angle sensor
        {
            SR->ShiftCS();
            SR->ShiftCS();
            SR->ShiftCS();
            SR->ShiftCS();
            SR->ShiftCS();
            
            float angleVal = liveReadAngle();
            debug.printf(" Angle      │ %8.2f°          │", angleVal);
            
            // Status indicator for angle
            if (angleVal > 300 || angleVal < 60) {
                debug.print(" 🟢GOOD │");
            } else {
                debug.print(" 🟡FAIR │");
            }
            
            // Visual angle representation
            int angleBars = (int)(angleVal / 18); // 360/20 = 18 degrees per bar
            debug.print(" [");
            for(int j = 0; j < 20; j++) {
                if(j == angleBars) debug.print("▲");
                else if(j < angleBars) debug.print("─");
                else debug.print("░");
            }
            debug.println("] ║");
        }
        else // Magnetic sensor
        {
            SR->ShiftCS();
            debug.print(" Magnetic   │ ");
            
            // Get magnetic value
            _myspi->transfer(readReg | Channel);
            sensorVal = _myspi->transfer16(0x00);
            actualVal = (float)sensorVal * 150 / 32768;
            
            debug.printf("%7.2f mT        │", actualVal);
            
            // Status indicator
            if (abs(actualVal) > 50) {
                debug.print(" 🔴HIGH │");
            } else if (abs(actualVal) > 25) {
                debug.print(" 🟡MED  │");
            } else {
                debug.print(" 🟢LOW  │");
            }
            
            // Visual magnetic field representation
            int magBars = (int)((abs(actualVal) + 5) / 5); // Scale magnetic field
            if(magBars > 20) magBars = 20;
            debug.print(" [");
            for(int j = 0; j < 20; j++) {
                if(j < magBars) {
                    if(actualVal < 0) debug.print("◄");
                    else debug.print("►");
                } else {
                    debug.print("░");
                }
            }
            debug.println("] ║");
        }
    }
    
    debug.println("╚══════════════════════════════════════════════════════════════════════╝");
    debug.printf("║ Timestamp: %10lu ms │ Total Sensors: %2d                      ║\n", millis(), totalTMAG5170);
    debug.println("╚══════════════════════════════════════════════════════════════════════╝");
    
    SR->ShiftCS();
    _myspi->endTransaction();
}

void Sensors::LiveReadTMAG(uint8_t Channel, bool special32bitEN)
{
    union c
    {
        uint16_t TMAGValues[3]; // 0 - xaxis, 1 - yaxis, 2 - zaxis
        int16_t sint[3];
    } c;
    if (special32bitEN)
    {
        byte First = _myspi->transfer(readReg | Channel);
        byte Second = _myspi->transfer(0x00); // MSB
        byte Third = _myspi->transfer(0x00);  // LSB
        byte Fourth = _myspi->transfer(0xFF);
        // Serial.printf("First: %x, Second: %x, Third: %x, Fourth: %x\n", First, Second, Third, Fourth);
        c.TMAGValues[0] = ((uint16_t)(Second & 0x0F) << 12) | ((Third & 0xF0) << 4) | (Fourth & 0xF0);
        c.TMAGValues[1] = ((uint16_t)(First & 0x0F) << 12) | ((Second & 0xF0) << 4) | ((Third & 0x0F) << 4);
        // Extracting the bits for Axis 2
        debug.print((float)c.sint[0] * 150 / 32768, 2);
        debug.print("   ");
        debug.print((float)c.sint[1] * 150 / 32768, 2);
        debug.print("   ");
    }
    else
    {
        _myspi->transfer(readReg | Channel);
        // MSB = _myspi->transfer(0x00); // MSB
        // LSB = _myspi->transfer(0x00); // LSB
        sensorVal = _myspi->transfer16(0x00);
        actualVal = (float)sensorVal * 150 / 32768;
        thxStat = (thxStat << 1);
        if (actualVal > thxHiLimit || actualVal < thxLoLimit)
            thxStat |= 0x1;
        //_myspi->transfer(0x0F);
        // sensorVal = (MSB << 8) | LSB;
        debug.print(actualVal, 2);
        debug.print("   ");
    }
}

void Sensors::LiveReadTMAGFormatted(uint8_t Channel, bool special32bitEN)
{
    union c
    {
        uint16_t TMAGValues[3]; // 0 - xaxis, 1 - yaxis, 2 - zaxis
        int16_t sint[3];
    } c;
    
    if (special32bitEN)
    {
        byte First = _myspi->transfer(readReg | Channel);
        byte Second = _myspi->transfer(0x00); // MSB
        byte Third = _myspi->transfer(0x00);  // LSB
        byte Fourth = _myspi->transfer(0xFF);
        
        c.TMAGValues[0] = ((uint16_t)(Second & 0x0F) << 12) | ((Third & 0xF0) << 4) | (Fourth & 0xF0);
        c.TMAGValues[1] = ((uint16_t)(First & 0x0F) << 12) | ((Second & 0xF0) << 4) | ((Third & 0x0F) << 4);
        
        float xVal = (float)c.sint[0] * 150 / 32768;
        float yVal = (float)c.sint[1] * 150 / 32768;
        
        debug.printf("X: %7.2f mT │ Y: %7.2f mT │", xVal, yVal);
        
        // Status indicator
        if (abs(xVal) > 50 || abs(yVal) > 50) {
            debug.print(" 🔴HIGH");
        } else if (abs(xVal) > 25 || abs(yVal) > 25) {
            debug.print(" 🟡MED ");
        } else {
            debug.print(" 🟢LOW ");
        }
    }
    else
    {
        _myspi->transfer(readReg | Channel);
        sensorVal = _myspi->transfer16(0x00);
        actualVal = (float)sensorVal * 150 / 32768;
        thxStat = (thxStat << 1);
        if (actualVal > thxHiLimit || actualVal < thxLoLimit)
            thxStat |= 0x1;
            
        debug.printf("Value: %7.2f mT │", actualVal);
        
        // Status indicator with color
        if (abs(actualVal) > thxHiLimit) {
            debug.print(" 🔴HIGH");
        } else if (abs(actualVal) > (thxHiLimit/2)) {
            debug.print(" 🟡MED ");
        } else {
            debug.print(" 🟢LOW ");
        }
    }
}

uint32_t Sensors::liveReadLDC()
{
    byte MSB, LSB, MMSB;
    uint32_t LDCValue;
    _myspi->transfer(readReg | LDC1101_REGLHR_DATALSB);
    LSB = _myspi->transfer(0x00);
    MMSB = _myspi->transfer(0x00);
    MSB = _myspi->transfer(0x00);
    // Serial.printf("MSB: %x, MMSB: %x, LSB: %x\n", MSB, MMSB, LSB);
    LDCValue = ((unsigned long)MSB << 16) | ((unsigned long)MMSB << 8) | LSB;
    return LDCValue;
}

void Sensors::liveReadSensors(bool Special32BitEN)
{
    // LDC Live Readout
    // debug.setEnabled(1);
    // SR->SelectCS(LDC_CSpin[Q1]);
    // unsigned long LDCValues[totalLDC1101];

    union c
    {
        uint16_t TMAGValues[MAX_NUM_TMAG][3]; // 0 - xaxis, 1 - yaxis, 2 - zaxis
        int16_t sint[MAX_NUM_TMAG][3];
    } c;

    byte MSB, LSB, MMSB;
    // SR->StoreOneLOWbit();
    // SR->ShiftCS();
    // SR->ShiftCS();
    // SR->SelectCS(Q0);
    SR->StoreOneLOWbit();
    // _myspi->beginTransaction(SensorSPI);
    //  for (int i = 0; i < totalLDC1101; i++)
    //  {
    //      // delayMicroseconds(10);
    //      // digitalWrite(LDC_CSpin[i], LOW);
    //      // SR->SelectCS(LDC_CSpin[i]);
    //      SR->ShiftCS();
    //      _myspi->transfer(readReg | LDC1101_REGLHR_DATALSB);
    //      LSB = _myspi->transfer(0x00);
    //      MMSB = _myspi->transfer(0x00);
    //      MSB = _myspi->transfer(0x00);
    //      // digitalWrite(LDC_CSpin[i], HIGH);
    //      // SR->High_all_CS();
    //      LDCValues[i] = ((unsigned long)MSB << 16) | ((unsigned long)MMSB << 8) | LSB;
    //      // debug.print(" : ");
    //      debug.print(LDCValues[i]);
    //      debug.print("   ");
    //  }
    //  debug.print("");
    //  TMAG Live Readout
    if (Special32BitEN)
    {
        for (int i = 0; i < totalTMAG5170; i++)
        {
            byte First, Second, Third, Fourth;
            // uint16_t axis1, axis2;
            //  digitalWrite(TMAG_CSpin[i], LOW);
            //  SR->SelectCS(TMAG_CSpin[i]);
            // Serial.print (i%4);
            if (i % 4 == 0) //== 0)
            {

                // SR->SelectCS(Q9);
                SR->ShiftCS();
                SR->ShiftCS();
                SR->ShiftCS();
                SR->ShiftCS();
                SR->ShiftCS();
                debug.println("");
                debug.print("tmag ");
                debug.print(i);
                debug.print(" : ");
                debug.print(liveReadAngle(), 2);
                // debug.print((liveReadAngle() - angleCalVal[i / 4]), 2);
                debug.print("  ");
                // liveReadAngle();
            }
            else
            {

                SR->ShiftCS();
                // SR->SelectCS(TMAG_CSpin[i]);
                First = _myspi->transfer(readReg | X_CH_RESULT);
                Second = _myspi->transfer(0x00); // MSB
                Third = _myspi->transfer(0x00);  // LSB
                Fourth = _myspi->transfer(0xFF);
                // digitalWrite(TMAG_CSpin[i], HIGH);
                // SR->High_all_CS();
                c.TMAGValues[i][0] = ((uint16_t)(Second & 0x0F) << 12) | ((Third & 0xF0) << 4) | (Fourth & 0xF0);
                c.TMAGValues[i][1] = ((uint16_t)(First & 0x0F) << 12) | ((Second & 0xF0) << 4) | ((Third & 0x0F) << 4);
                // Extracting the bits for Axis 2
                // TMAGValues[i][0] = (float)c.sint[i][0] * 300 / 32768;
                debug.print("tmag ");
                debug.print(i);
                debug.print(" : ");
                debug.print((float)c.sint[i][0] * 300 / 32768, 2);
                debug.print(" ");
                // TMAGValues[i][1] = (float)c.sint[i][1] * 300 / 32768;
                debug.print((float)c.sint[i][1] * 300 / 32768, 2);
                debug.print("      ");
            }
        }
        debug.println("");
        SR->ShiftCS();
        // SR->High_all_CS();
    }
    else
    {
        for (int i = 0; i < totalTMAG5170; i++)
        {
            byte MSB, LSB;
            // digitalWrite(TMAG_CSpin[i], LOW);
            SR->SelectCS(TMAG_CSpin[i]);
            _myspi->transfer(readReg | X_CH_RESULT);
            MSB = _myspi->transfer(0x00); // MSB
            LSB = _myspi->transfer(0x00); // LSB
            _myspi->transfer(0x0F);
            // digitalWrite(TMAG_CSpin[i], HIGH);
            SR->High_all_CS();
            c.TMAGValues[i][0] = (MSB << 8) | LSB; // x axis raw values
            debug.print((float)c.sint[i][0] * 300 / 32768, 2);
            debug.print(" ");
            delayMicroseconds(1);
            // digitalWrite(TMAG_CSpin[i], LOW);
            SR->SelectCS(TMAG_CSpin[i]);
            _myspi->transfer(readReg | Y_CH_RESULT);
            MSB = _myspi->transfer(0x00); // MSB
            LSB = _myspi->transfer(0x00); // LSB
            _myspi->transfer(0x0F);
            // digitalWrite(TMAG_CSpin[i], HIGH);
            SR->High_all_CS();
            c.TMAGValues[i][1] = (MSB << 8) | LSB; // y axis raw value
            debug.print((float)c.sint[i][1] * 300 / 32768, 2);
            debug.print(" ");
            delayMicroseconds(1);
            // digitalWrite(TMAG_CSpin[i], LOW);
            SR->SelectCS(TMAG_CSpin[i]);
            _myspi->transfer(readReg | Z_CH_RESULT);
            MSB = _myspi->transfer(0x00); // MSB
            LSB = _myspi->transfer(0x00); // LSB
            _myspi->transfer(0x0F);
            // digitalWrite(TMAG_CSpin[i], HIGH);
            SR->High_all_CS();
            c.TMAGValues[i][2] = (MSB << 8) | LSB; // z axis raw value
            debug.print((float)c.sint[i][2] * 300 / 32768, 2);
            debug.print(" ");
            delayMicroseconds(1);
        }
        debug.println("");
    }
    // _myspi->endTransaction();
    analog_val = analogRead(26);
}

void Sensors::calibrateAngle()
{
    // _myspi->beginTransaction(SensorSPI);
    // float angle_val[10];
    float av_val = 0;
    debug.print("calibrating angles .");
    for (int i = 0; i < totalTMAG5170; i = i + 4)
    {

        SR->SelectCS(TMAG_CSpin[i]);
        for (int j = 0; j < 10; j++)
        {
            av_val = av_val + liveReadAngle();
            delay(10);
        }
        av_val = av_val / 10;
        angleCalVal[i / 4] = av_val;

        av_val = 0;
        debug.print(".");
    }
    debug.println("");
    // _myspi->endTransaction();
}

float Sensors ::liveReadAngle()
{
    uint8_t MSB, LSB;
    int angle_val;
    int fraction_val;
    float actual_val;
    // _myspi->beginTransaction(SensorSPI);
    _myspi->transfer(readReg | ANGLE_RESULT);

    MSB = _myspi->transfer(0x00);
    LSB = _myspi->transfer(0x00);
    _myspi->transfer(0xff);
    fraction_val = (LSB & 0x0f);
    angle_val = ((MSB & 0x1f) << 4) | ((LSB & 0xf0) >> 4);
    actual_val = ((float)fraction_val / 16) + (float)angle_val;
    // debug.print(angle_val);//(MSB,HEX);

    // debug.print(((float)fraction_val / 16) + (float)angle_val, 2); //(LSB,HEX);
    // debug.print(" ");
    return actual_val;
    // actual_val = angle_val +((float)fraction_val/16);
    // angle_val = angle_val + fraction_val;
    // debug.print((angle_val));//fraction_val + angle_val);
}

void Sensors::getMFLdata(uint8_t *buffer, uint8_t Channel)
{
    _myspi->beginTransaction(SensorSPI);
    SR->StoreOneLOWbit();
    SR->ShiftCS();
    SR->ShiftCS();
    for (int i = 0; i < numLDC; i++)
    {
        SR->ShiftCS();
        _myspi->transfer(readReg | LDC1101_REGLHR_DATALSB);
        *buffer = _myspi->transfer(0x00);
        buffer++;
        *buffer = _myspi->transfer(0x00);
        buffer++;
        *buffer = _myspi->transfer(0x00);
        buffer++;
    }

    for (int i = 0; i < numTMAG; i++)
    {
        SR->ShiftCS();
        *buffer = _myspi->transfer(readReg | Channel);
        buffer++;
        *buffer = _myspi->transfer(0x00);
        buffer++;
        *buffer = _myspi->transfer(0x00);
        buffer++;
        *buffer = _myspi->transfer(0xFF);
        buffer++;
    }
    SR->ShiftCS();
    _myspi->endTransaction();
}

void Sensors::getODOdata(uint8_t *buffer, uint8_t Channel)
{
    _myspi->beginTransaction(SensorSPI);
    SR->StoreOneLOWbit();
    for (int i = 0; i < num_sensors; i++)
    {
        SR->ShiftCS();

        if (i % 2 == 0)
        {
            *buffer = _myspi->transfer(readReg | Channel);
            buffer++;
            *buffer = _myspi->transfer(0x00); // MSB
            buffer++;
            *buffer = _myspi->transfer(0x00); // LSB
            buffer++;
            *buffer = _myspi->transfer(0xFF);
            buffer++;
        }
        else
        {
            _myspi->transfer(readReg | ANGLE_RESULT);
            *buffer = _myspi->transfer(0x00);
            buffer++;
            *buffer = _myspi->transfer(0x00);
            buffer++;
        }

        // *buffer = _myspi->transfer(89);
        // buffer++;
        // *buffer = _myspi->transfer(0x00); // MSB
        // buffer++;
        // *buffer = _myspi->transfer(0x00); // LSB
        // buffer++;
        // *buffer = _myspi->transfer(0xFF);
        // buffer++;
    }
    // SR->High_all_CS();
    SR->ShiftCS();
    _myspi->endTransaction();
}

void Sensors::getODOdata1CH(uint8_t *buffer, uint8_t Channel)
{
    thxStat = 0;
    _myspi->beginTransaction(SensorSPI);
    SR->StoreOneLOWbit();
    for (int i = 0; i < num_sensors; i++)
    {
        SR->ShiftCS();

        if (i % 2 == 0)
        {
            _myspi->transfer(readReg | Channel);
            sensorVal = _myspi->transfer16(0x00);
            *buffer = (sensorVal >> 8) & 0xFF; // MSB
            buffer++;
            *buffer = sensorVal & 0xFF; // LSB
            buffer++;

            actualVal = (float)sensorVal * 150 / 32768;
            thxStat = (thxStat << 1);
            if (actualVal > thxHiLimit || actualVal < thxLoLimit)
                thxStat |= 0x1;
        }
        else
        {
            _myspi->transfer(readReg | ANGLE_RESULT);
            *buffer = _myspi->transfer(0x00);
            buffer++;
            *buffer = _myspi->transfer(0x00);
            buffer++;
        }
    }
    *buffer = thxStat;
    SR->ShiftCS();
    _myspi->endTransaction();
}

void Sensors::getEGPdata(uint8_t *buffer, uint8_t Channel)
{
    _myspi->beginTransaction(SensorSPI);
    SR->StoreOneLOWbit();
    for (int i = 0; i < totalTMAG5170; i++)
    {
        if (i % 4 == 0)
        {
            // SR->SelectCS(Q9);
            SR->ShiftCS();
            SR->ShiftCS();
            SR->ShiftCS();
            SR->ShiftCS();
            SR->ShiftCS();

            _myspi->transfer(readReg | ANGLE_RESULT);

            *buffer = _myspi->transfer(0x00); // MSB
            buffer++;
            *buffer = _myspi->transfer(0x00); // LSB
            buffer++;
            _myspi->transfer(0xFF);
        }
        else
        {
            SR->ShiftCS();

            *buffer = _myspi->transfer(0x89);
            buffer++;
            *buffer = _myspi->transfer(0x00); // MSB
            buffer++;
            *buffer = _myspi->transfer(0x00); // LSB
            buffer++;
            *buffer = _myspi->transfer(0xFF);
            buffer++;
        }
    }

    SR->ShiftCS();
    _myspi->endTransaction();
}

void Sensors::UpdateSensors(bool Special32BitEN, byte *buffer)
{

    SR->StoreOneLOWbit();
    // SR->SelectCS(Q3);
    //  TMAG Live Readout
    if (Special32BitEN)
    {
        for (int i = 0; i < totalTMAG5170; i++)
        {
            if (i % 4 == 0)
            {
                // SR->SelectCS(Q9);
                SR->ShiftCS();
                SR->ShiftCS();
                SR->ShiftCS();
                SR->ShiftCS();
                SR->ShiftCS();

                _myspi->transfer(readReg | ANGLE_RESULT);

                *buffer = _myspi->transfer(0x00); // MSB
                buffer++;
                *buffer = _myspi->transfer(0x00); // LSB
                buffer++;
                _myspi->transfer(0xFF);
            }
            else
            {
                SR->ShiftCS();

                *buffer = _myspi->transfer(0x89);
                buffer++;
                *buffer = _myspi->transfer(0x00); // MSB
                buffer++;
                *buffer = _myspi->transfer(0x00); // LSB
                buffer++;
                *buffer = _myspi->transfer(0xFF);
                buffer++;
            }
        }
    }
    SR->ShiftCS();
    // else
    // {
    //     for (int i = 0; i < totalTMAG5170; i++)
    //     {

    //         // digitalWrite(TMAG_CSpin[i], LOW);
    //         SR->SelectCS(TMAG_CSpin[i]);
    //         _myspi->transfer(readReg | X_CH_RESULT);
    //         //  MSB = _myspi->transfer(0x00); // MSB
    //         //  LSB = _myspi->transfer(0x00); // LSB
    //         _myspi->transfer(0x0F);
    //         // digitalWrite(TMAG_CSpin[i], HIGH);
    //         SR->High_all_CS();
    //         delayMicroseconds(1);
    //         // digitalWrite(TMAG_CSpin[i], LOW);
    //         SR->SelectCS(TMAG_CSpin[i]);
    //         _myspi->transfer(readReg | Y_CH_RESULT);
    //         //   MSB = _myspi->transfer(0x00); // MSB
    //         //   LSB = _myspi->transfer(0x00); // LSB
    //         _myspi->transfer(0x0F);
    //         // digitalWrite(TMAG_CSpin[i], HIGH);
    //         SR->High_all_CS();
    //         delayMicroseconds(1);
    //         // digitalWrite(TMAG_CSpin[i], LOW);
    //         SR->SelectCS(TMAG_CSpin[i]);
    //         _myspi->transfer(readReg | Z_CH_RESULT);
    //         //   MSB = _myspi->transfer(0x00); // MSB
    //         //   LSB = _myspi->transfer(0x00); // LSB
    //         _myspi->transfer(0x0F);
    //         // digitalWrite(TMAG_CSpin[i], HIGH);
    //         SR->High_all_CS();
    //         delayMicroseconds(1);
    //     }
    //     // debug.println("");
    // }

    // buffer[bufferindex] = uint8_t(0x45);
    // bufferindex++;
}
/*For Readout*/
/*
for (int h = 0; h < 10000; h++)
       {
           byte MSB, LSB, MMSB;
           delay(100);
           //digitalWrite(LDC_CSpin[i], LOW);
           _myspi->transfer(readReg | LDC1101_REGLHR_DATALSB);
           LSB = _myspi->transfer(0x00);
           MMSB = _myspi->transfer(0x00);
           MSB = _myspi->transfer(0x00);
           unsigned long combinedValue = ((unsigned long)MSB << 16) | ((unsigned long)MMSB << 8) | LSB;
           combinedValue = map(combinedValue, 0, 16777216, 0, 1000);
           Serial.println(combinedValue);
           //digitalWrite(LDC_CSpin[i], HIGH);
       }*/

void Sensors::dma_fetch_ldcdata(byte *buffer, uint32_t sample_count)
{
    // x = micros();
    uint16_t analog_val = analogRead(26);
    // Serial.println(analog_val);
    // sprintf((char*) numStr, ":%06u:%u\n", sample_count, analog_val); // Convert to string in ASCII decimal format
    /*if(sample_count != (prev_sample+1) )
    {
        Serial.println("sample missed");
    }
    prev_sample = sample_count;*/
    *buffer = uint8_t(0x4d);
    buffer++;
    *buffer = uint8_t(0x2c);
    buffer++;
    // SR->SelectCS(LDC_CSpin[Q1]);
    SR->StoreOneLOWbit();
    SR->ShiftCS();
    SR->ShiftCS();
    for (int i = 0; i < 4; i++)
    {
        // delayMicroseconds(10);
        // digitalWriteFast(LDC_CSpin[i], LOW);
        SR->ShiftCS();
        spi_dma_transfer(LDC_data_add, buffer, 3);

        /*for (int i=0;i<4;i++)
        {
         Serial.println(*buffer);
         buffer++;
        }*/
        buffer += 3;
        *buffer = uint8_t(0x2c);
        buffer++;
    }

    for (int i = 0; i < 8; i++)
    {
        if (i == 2)
        {
            SR->ShiftCS();
            SR->ShiftCS();
        }
        SR->ShiftCS();
        spi_dma_transfer(TMAG_data_add, buffer, 4);
        buffer += 4;
        *buffer = uint8_t(0x2c);
        buffer++;
    }

    *buffer = uint8_t(0x45);
    buffer++;
    *buffer = uint8_t(0x2c);
    buffer++;
    *buffer = uint8_t(sample_count >> 24);
    buffer++;
    *buffer = uint8_t(sample_count >> 16);
    buffer++;
    *buffer = uint8_t(sample_count >> 8);
    buffer++;
    *buffer = uint8_t(sample_count);
    buffer++;
    *buffer = uint8_t(0x2c);
    buffer++;
    *buffer = uint8_t(analog_val >> 8);
    buffer++;
    *buffer = uint8_t(analog_val);
    buffer++;
    *buffer = uint8_t(0x0A);
    buffer++;

    /*for (int i = 0; numStr[i] != '\0'; i++)
    {
        *buffer = uint8_t(numStr[i]);
        buffer++;
    }*/
    // y = micros()-x;
}

void Sensors::dma_setup()
{
    dma_tx_channel = dma_claim_unused_channel(true);
    dma_rx_channel = dma_claim_unused_channel(true);
}

void Sensors::spi_dma_transfer(uint8_t *tx_data, uint8_t *rx_data, size_t len)
{
    dma_channel_config tx_config = dma_channel_get_default_config(dma_tx_channel);
    channel_config_set_transfer_data_size(&tx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_config, true);
    channel_config_set_write_increment(&tx_config, false);
    channel_config_set_dreq(&tx_config, DREQ_SPI1_TX);

    dma_channel_configure(
        dma_tx_channel,
        &tx_config,
        &spi_get_hw(SPI_PORT)->dr, // SPI data register for transfer
        tx_data,                   // Source buffer
        len,                       // Transfer count
        false                      // Don't start yet
    );
    dma_channel_config rx_config = dma_channel_get_default_config(dma_rx_channel);
    channel_config_set_transfer_data_size(&rx_config, DMA_SIZE_8);
    channel_config_set_read_increment(&rx_config, false);
    channel_config_set_write_increment(&rx_config, true);
    channel_config_set_dreq(&rx_config, DREQ_SPI1_RX);

    dma_channel_configure(
        dma_rx_channel,
        &rx_config,
        rx_data,                   // Destination buffer
        &spi_get_hw(SPI_PORT)->dr, // SPI data register for receive
        len,                       // Transfer count
        false                      // Don't start yet
    );
    // Start SPI transaction
    // gpio_put(-1, 0);  // Select the slave
    dma_start_channel_mask((1u << dma_tx_channel) | (1u << dma_rx_channel));
    // dma_channel_set_read_addr(dma_tx_channel, rx_data, true);
    //  Wait for DMA transfer to complete
    dma_channel_wait_for_finish_blocking(dma_tx_channel);
    dma_channel_wait_for_finish_blocking(dma_rx_channel);
    // gpio_put(-1, 1);  // Deselect the slave
}
