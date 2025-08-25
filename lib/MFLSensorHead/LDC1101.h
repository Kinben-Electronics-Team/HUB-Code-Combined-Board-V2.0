#ifndef LDC1101_h
#define LDC1101_h
#include <Arduino.h>
#include <SPI.h>
#include "shift_register/LVC595.h"
#include "hardware/dma.h"

/* ------------------------------------------------------------------- MACROS */

#define SPI_PORT spi1
#define SPI_BAUD_RATE 500000

/* Register addresses*/
#define PD_CNTL_Reg 0x004
#define SDI_CNTL_Reg 0x008
#define SDO_CNTL_Reg 0x00C
#define DATA_CNTL_Reg 0x010
#define OFST_CNTL_Reg 0x020
#define REF_MRG0_Reg 0x030

/*   SPI Registers   */
#define readReg 0x80
#define writeReg 0x0

/* sensor Register */
#define LDC1101_REG_CFG_RP_MEASUREMENT_DYNAMIC_RANGE 0x01
#define LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_1 0x02
#define LDC1101_REG_CFG_INTERNAL_TIME_CONSTANT_2 0x03
#define LDC1101_REG_CFG_RPL_CONVERSION_INTERVAL 0x04
#define LDC1101_REG_CFG_ADDITIONAL_DEVICE 0x05
#define LDC1101_REG_RP_THRESH_HLSB 0x06
#define LDC1101_REG_RP_THRESH_H_MSB 0x07
#define LDC1101_REG_RP_THRESHLLSB 0x08
#define LDC1101_REG_RP_THRESHL_MSB 0x09
#define LDC1101_REG_CFG_INTB_MODE 0x0A
#define LDC1101_REG_CFG_POWER_STATE 0x0B
#define LDC1101_REG_AMPLITUDE_CONTROL_REQUIREMENT 0x0C
#define LDC1101_REGL_THRESH_HILSB 0x16
#define LDC1101_REGL_THRESH_HI_MSB 0x17
#define LDC1101_REGL_THRESHLOLSB 0x18
#define LDC1101_REGL_THRESHLO_MSB 0x19
#define LDC1101_REG_RPL_MEASUREMENT_STATUS 0x20
#define LDC1101_REG_RP_DATALSB 0x21
#define LDC1101_REG_RP_DATA_MSB 0x22
#define LDC1101_REGL_DATALSB 0x23
#define LDC1101_REGL_DATA_MSB 0x24
#define LDC1101_REGLHR_RCOUNTLSB 0x30
#define LDC1101_REGLHR_RCOUNT_MSB 0x31
#define LDC1101_REGLHR_OFFSETLSB 0x32
#define LDC1101_REGLHR_OFFSET_MSB 0x33
#define LDC1101_REG_CFGLHR 0x34
#define LDC1101_REGLHR_DATALSB 0x38
#define LDC1101_REGLHR_DATA_MID 0x39
#define LDC1101_REGLHR_DATA_MSB 0x3A
#define LDC1101_REGLHR_STATUS 0x3B
#define LDC1101_REG_DEVICE_RID_VALUE 0x3E
#define LDC1101_REG_CHIP_ID 0x3F

/* Register RP_SET Field Descriptions (RW) */
#define LDC1101_RP_SET_RP_MAX_IS_DRIVEN 0x00
#define LDC1101_RP_SET_RP_MAX_CURRENT_IS_IGNORED 0x80
#define LDC1101_RP_SET_RP_MAX_96KOhm 0x00
#define LDC1101_RP_SET_RP_MAX_48KOhm 0x10
#define LDC1101_RP_SET_RP_MAX_24KOhm 0x20
#define LDC1101_RP_SET_RP_MAX_12KOhm 0x30
#define LDC1101_RP_SET_RP_MAX_6KOhm 0x40
#define LDC1101_RP_SET_RP_MAX_3KOhm 0x50
#define LDC1101_RP_SET_RP_MAX_1_5KOhm 0x60
#define LDC1101_RP_SET_RP_MAX_0_75KOh 0x70
#define LDC1101_RP_SET_RP_MIN_96KOhm 0x00
#define LDC1101_RP_SET_RP_MIN_48KOhm 0x01
#define LDC1101_RP_SET_RP_MIN_24KOhm 0x02
#define LDC1101_RP_SET_RP_MIN_12KOhm 0x03
#define LDC1101_RP_SET_RP_MIN_6KOhm 0x04
#define LDC1101_RP_SET_RP_MIN_3KOhm 0x05
#define LDC1101_RP_SET_RP_MIN_1_5KOhm 0x06
#define LDC1101_RP_SET_RP_MIN_0_75KOh 0x07

/* Configure Internal Time Constant 1 (RW) */
#define LDC1101_TC1_C1_0_75pF 0x00
#define LDC1101_TC1_C1_1_5pF 0x40
#define LDC1101_TC1_C1_3pF 0x80
#define LDC1101_TC1_C1_6pF 0xC0
#define LDC1101_TC1_R1_417kOhm 0x00
#define LDC1101_TC1_R1_212_7kOhm 0x10
#define LDC1101_TC1_R1_21_1kOhm 0x1F

/* Configure Internal Time Constant 2 (RW) */
#define LDC1101_TC2_C2_3pF 0x00
#define LDC1101_TC2_C2_6pF 0x40
#define LDC1101_TC2_C2_12pF 0x80
#define LDC1101_TC2_C2_24pF 0xC0
#define LDC1101_TC2_R2_835kOhm 0x00
#define LDC1101_TC2_R2_426_4kOhm 0x20
#define LDC1101_TC2_R2_30_5kOhm 0x3F

/* Configure RP+L conversion interval (RW) */
#define LDC1101_DIG_CFG_MIN_FREQ_500kHz 0x00
#define LDC1101_DIG_CFG_MIN_FREQ_8MHz 0xF0
#define LDC1101_DIG_CFG_RESP_TIME_192s 0x02
#define LDC1101_DIG_CFG_RESP_TIME_384s 0x03
#define LDC1101_DIG_CFG_RESP_TIME_768s 0x04
#define LDC1101_DIG_CFG_RESP_TIME_1536s 0x05
#define LDC1101_DIG_CFG_RESP_TIME_3072s 0x06
#define LDC1101_DIG_CFG_RESP_TIME_6144s 0x07

/* Configure additional device settings (RW) */
#define LDC1101_ALT_CFG_SHUTDOWN_ENABLE 0x02
#define LDC1101_ALT_CFG_SHUTDOWN_DISABLE 0x00
#define LDC1101_ALT_CFGL_OPTIMAL_DISABLED 0x00
#define LDC1101_ALT_CFGL_OPTIMAL_ENABLE 0x01

/* Configure INTB reporting on SDO pin (RW) */
#define LDC1101_INTB_MODE_DONT_REPORT_INTB_ON_SDO_PIN 0x00
#define LDC1101_INTB_MODE_REPORT_INTB_ON_SDO_PIN 0x80
#define LDC1101_INTB_MODE_REPORTLHR_DATA_READY 0x20
#define LDC1101_INTB_MODEL_CONVERSION_TOL_THRESHOLDS 0x10
#define LDC1101_INTB_MODEL_CONVERSION_TOL_HIGH_THRESHOLDS 0x08
#define LDC1101_INTB_MODE_REPORT_RPL_DATA_READY 0x04
#define LDC1101_INTB_MODE_RP_CONVERSION_TOL_THRESHOLDS 0x02
#define LDC1101_INTB_MODE_RP_CONVERSION_TOL_HIGH_THRESHOLDS 0x01
#define LDC1101_INTB_MODE_NO_OUTPUT 0x00

/* Configure Power State (RW) */
#define LDC1101_FUNC_MODE_ACTIVE_CONVERSION_MODE 0x00
#define LDC1101_FUNC_MODE_SLEEP_MODE 0x01
#define LDC1101_FUNC_MODE_SHUTDOWN_MODE 0x02

/* High Resolution L Configuration (RW) */
#define LDC1101LHR_CFG_FREQUENCY_NOT_DIVIDED 0x00
#define LDC1101LHR_CFG_FREQUENCY_DIVIDED_BY_2 0x01
#define LDC1101LHR_CFG_FREQUENCY_DIVIDED_BY_4 0x02
#define LDC1101LHR_CFG_FREQUENCY_DIVIDED_BY_8 0x03

/* TMAG Register Defines*/

#define STAY 0xffff
#define axis_X 0x0
#define axis_Y 0x1
#define axis_Z 0x2
//------------------Register Address------------------------
#define DEVICE_CONFIG 0x0
#define SENSOR_CONFIG 0x1
#define SYSTEM_CONFIG 0x2
#define ALERT_CONFIG 0x3
#define X_THRX_CONFIG 0x4
#define Y_THRX_CONFIG 0x5
#define Z_THRX_CONFIG 0x6
#define T_THRX_CONFIG 0x7
#define CONV_STATUS 0x8
#define X_CH_RESULT 0x9
#define Y_CH_RESULT 0xA
#define Z_CH_RESULT 0xB
#define TEMP_RESULT 0xC
#define AFE_STATUS 0xD
#define SYS_STATUS 0xE
#define TEST_CONFIG 0xF
#define OSC_MONITOR 0x10
#define MAG_GAIN_CONFIG 0x11
#define ANGLE_RESULT 0x13
#define MAGNITUDE_RESULT 0x14

#define start_DE_CRC 0x0f000400
#define DeviceConfigData 0b0101000000001000
#define DeviceStart 0b0101000000101000 // 0x5028
#define RANGE_50mT 0x0

//------------------DEVICE_CONFIG------------------------
#define CONV_AVG_MASK 0x7000
#define CONV_AVG_1x 0x0
#define CONV_AVG_2x 0x1000
#define CONV_AVG_4x 0x2000
#define CONV_AVG_8x 0x3000
#define CONV_AVG_16x 0x4000
#define CONV_AVG_32x 0x5000

#define MAG_TEMPCO_MASK 0x300
#define MAG_TEMPCO_0pd 0x0
#define MAG_TEMPCO_0R12pd 0x100
#define MAG_TEMPCO_0R2pd 0x300

#define OPERATING_MODE_MASK 0x70
#define OPERATING_MODE_ConfigurationMode 0x0
#define OPERATING_MODE_StandbyMode 0x10
#define OPERATING_MODE_activeMeasureMode 0x20
#define OPERATING_MODE_ActiveTriggerMode 0x30
#define OPERATING_MODE_WakeupAndSleepMode 0x40
#define OPERATING_MODE_SleepMode 0x50
#define OPERATING_MODE_DeepsleepMode 0x60

#define T_CH_EN_TempChannelDisabled 0x0
#define T_CH_EN_TempChannelEnabled 0x8

#define T_RATE_sameAsOtherSensors 0x0
#define T_RATE_oncePerConversionSet 0x4

#define T_HLT_EN_tempLimitCheckOff 0x0
#define T_HLT_EN_tempLimitCheckOn 0x2

#define TEMP_COMP_EN_TempCompensationDisenabled 0x0
#define TEMP_COMP_EN_TempCompensationEnabled 0x1

//------------------SENSOR_CONFIG------------------------
#define ANGLE_EN_NoAngleCalculation 0x0
#define ANGLE_EN_X_Y 0x4000
#define ANGLE_EN_Y_Z 0x8000
#define ANGLE_EN_Z_X 0xC000

#define SLEEPTIME_1ms 0x0
#define SLEEPTIME_5ms 0x400
#define SLEEPTIME_10ms 0x800
#define SLEEPTIME_15ms 0xC00
#define SLEEPTIME_20ms 0x1000
#define SLEEPTIME_30ms 0x1400
#define SLEEPTIME_50ms 0x1800
#define SLEEPTIME_100ms 0x1C00
#define SLEEPTIME_500ms 0x2000
#define SLEEPTIME_1000ms 0x2400

#define MAG_CH_EN_MASK 0x3C0
#define MAG_CH_EN_OFF 0x0
#define MAG_CH_EN_Xenabled 0x40
#define MAG_CH_EN_Yenabled 0x80
#define MAG_CH_EN_XYenabled 0xC0
#define MAG_CH_EN_Zenabled 0x100
#define MAG_CH_EN_ZXenabled 0x140
#define MAG_CH_EN_YZenabled 0x180
#define MAG_CH_EN_XYZenabled 0x1C0
#define MAG_CH_EN_XYXenabled 0x200
#define MAG_CH_EN_YXYenabled 0x240
#define MAG_CH_EN_YZYenabled 0x280
#define MAG_CH_EN_ZYZenabled 0x2C0
#define MAG_CH_EN_ZXZenabled 0x300
#define MAG_CH_EN_XZXenabled 0x340
#define MAG_CH_EN_XYZYXenabled 0x380
#define MAG_CH_EN_XYZZYXenabled 0x3C0

#define Z_RANGE_MASK 0x30
#define Z_RANGE_50mT 0x0
#define Z_RANGE_25mT 0x10
#define Z_RANGE_100mT 0x20

#define Y_RANGE_MASK 0xC
#define Y_RANGE_50mT 0x0
#define Y_RANGE_25mT 0x4
#define Y_RANGE_100mT 0x8

#define X_RANGE_MASK 0x3
#define X_RANGE_50mT 0x0
#define X_RANGE_25mT 0x1
#define X_RANGE_100mT 0x2
//------------------SYSTEM_CONFIG------------------------
#define DIAG_SEL_AllDataPath 0x0
#define DIAG_SEL_enabledDataPath 0x1000
#define DIAG_SEL_ALLDataPathInsequence 0x2000
#define DIAG_SEL_enabledDataPathInsequence 0x3000

#define TRIGGER_MODE_MASK 0x600
#define TRIGGER_MODE_SPI 0x0
#define TRIGGER_MODE_nCSsyncPulse 0x200
#define TRIGGER_MODE_ALERTsyncPulse 0x400

#define DATA_TYPE_32bit 0x0
#define DATA_TYPE_12bit_XY 0x40
#define DATA_TYPE_12bit_XZ 0x80
#define DATA_TYPE_12bit_ZY 0xC0
#define DATA_TYPE_12bit_XT 0x100
#define DATA_TYPE_12bit_YT 0x140
#define DATA_TYPE_12bit_ZT 0x180
#define DATA_TYPE_12bit_AM 0x1C0

#define DIAG_EN_AFEdiagnosticsDisabled 0x0
#define DIAG_EN_ExecutionOftheDiagnosticsSelectedInDEVICE_CFG 0x20

#define Z_HLT_EN_ZaxisLimitCheckoff 0x0
#define Z_HLT_EN_ZaxisLimitCheckon 0x4

#define Y_HLT_EN_YaxisLimitCheckoff 0x0
#define Y_HLT_EN_YaxisLimitCheckon 0x2

#define X_HLT_EN_XaxisLimitCheckoff 0x0
#define X_HLT_EN_XaxisLimitCheckon 0x1
//------------------ALERT_CONFIG------------------------
#define ALERT_LATCH_sourcesNotLatched 0x0
#define ALERT_LATCH_sourcesLatched 0x2000

#define ALERT_MODE_interruptMode 0x0
#define ALERT_MODE_comparatorMode 0x1000

#define STATUS_ALRT_ALERTisNotAsserted 0x0
#define STATUS_ALRT_ALERTisAsserted 0x800

#define RSLT_ALRT_ALERTisNotUsedToSignal 0x0
#define RSLT_ALRT_ALERTisUsedToSignal 0x100

#define THRX_COUNT_1_ConversionResult 0x0
#define THRX_COUNT_2_ConversionResult 0x10
#define THRX_COUNT_3_ConversionResult 0x20
#define THRX_COUNT_4_ConversionResult 0x30

#define T_THRX_ALRT_ALERTisNotUsedToSignal 0x0
#define T_THRX_ALRT_ALERTisUsedToSignal 0x8

#define Z_THRX_ALRT_ALERTisNotUsedToSignal 0x0
#define Z_THRX_ALRT_ALERTisUsedToSignal 0x4

#define Y_THRX_ALRT_ALERTisNotUsedToSignal 0x0
#define Y_THRX_ALRT_ALERTisUsedToSignal 0x2

#define X_THRX_ALRT_ALERTisNotUsedToSignal 0x0
#define X_THRX_ALRT_ALERTisUsedToSignal 0x1
//------------------TEST_CONFIG------------------------
#define CRC_DIS_CRCenabled 0x0
#define CRC_DIS_CRCdisabled 0x4

#define OSC_CNT_CTL_ResetCounters 0x0
#define OSC_CNT_CTL_StartOscCounterdrivenbyHFOSC 0x1
#define OSC_CNT_CTL_StartOscCounterdrivenbyLFOSC 0x2
#define OSC_CNT_CTL_stopCounter 0x3
//------------------MAG_GAIN_CONFIG------------------------
#define GAIN_SELECTION_noAxisSelected 0x0
#define GAIN_SELECTION_XisSelected 0x4000
#define GAIN_SELECTION_YisSelected 0x8000
#define GAIN_SELECTION_ZisSelected 0xC000

#define SENSOR_PER_EGP 4                               // number of sensors in one EGP board
#define NO_pin -1                                      // value for no pin assignment
#define MAX_NUM_TMAG 32                                // max number of tmag sensors 32
#define MAX_NUM_ANG_TMAG MAX_NUM_TMAG / SENSOR_PER_EGP // max possible number of angle sensors 8
#define MAX_SAMPLE_SIZE ((MAX_NUM_ANG_TMAG * 14))      // maximum size of sensor data

enum magAxisEN
{
    X_AXIS = MAG_CH_EN_Xenabled,
    Y_AXIS = MAG_CH_EN_Yenabled,
    XY_AXIS = MAG_CH_EN_XYenabled,
    Z_AXIS = MAG_CH_EN_Zenabled,
    ZX_AXIS = MAG_CH_EN_ZXenabled,
    YZ_AXIS = MAG_CH_EN_YZenabled,
    XYZ_AXIS = MAG_CH_EN_XYZenabled,
    XYX_AXIS = MAG_CH_EN_XYXenabled,
    YXY_AXIS = MAG_CH_EN_YXYenabled,
    YZY_AXIS = MAG_CH_EN_YZYenabled,
    ZYZ_AXIS = MAG_CH_EN_ZYZenabled,
    ZXZ_AXIS = MAG_CH_EN_ZXZenabled,
    XZX_AXIS = MAG_CH_EN_XZXenabled,
    XYZYX_AXIS = MAG_CH_EN_XYZYXenabled,
    XYZZYX_AXIS = MAG_CH_EN_XYZZYXenabled
};

enum angleAxis
{
    NO_ANGLE = ANGLE_EN_NoAngleCalculation,
    XY_ANGLE = ANGLE_EN_X_Y,
    YZ_ANGLE = ANGLE_EN_Y_Z,
    ZX_ANGLE = ANGLE_EN_Z_X
};

enum TMAGdataType
{
    bit32 = DATA_TYPE_32bit,
    XY = DATA_TYPE_12bit_XY,
    XZ = DATA_TYPE_12bit_XZ,
    ZY = DATA_TYPE_12bit_ZY,
    XT = DATA_TYPE_12bit_XT,
    YT = DATA_TYPE_12bit_YT,
    ZT = DATA_TYPE_12bit_ZT,
    AM = DATA_TYPE_12bit_AM,
};

class Debugger
{
public:
    Debugger(bool enabled);

    void print(const String &message);

    void println(const String &message);

    // Add more methods or overload methods as needed
    void print(int value);
    void print(unsigned int value);
    void print(byte value);
    void print(int value, int format);
    void print(unsigned int value, int format);
    void print(byte value, int format);
    void print(unsigned long value);
    void print(float value, int Digits);
    void println(unsigned long value);
    void println(int value);
    void println(unsigned int value);
    void println(byte value);
    void println(int value, int format);
    void println(unsigned int value, int format);
    void println(byte value, int format);
    void setEnabled(bool en);

private:
    bool enabled;
};

class Sensors
{

public:
    unsigned long LDCValues[4];
    float TMAGValues[8][3];
    uint16_t analog_val;
    unsigned int x, y;
    uint8_t numStr[20]; // Large enough to hold any 32-bit number as a string
    uint8_t thxStat;
    // unsigned long prev_sample = 0;

    /*----------------DONE---------------*/
    //-----Contructor -----------
    /**
     * @brief Constructor for MFL sensor. initialises class objects
     * @param sr address of shift register object. if shift register is not used pass NULL
     * @param spisetting setting of spi
     * @param num_sens number of sensor. By default it is set to 0, if egp is used then pass 0, 1 for mfl and number of odo sensors for odo
     * @param spi address of spi instance (SPI or SPI1 for RP2040). By default it is SPI1.
     */
    Sensors(ShiftRegister *sr, SPISettings spisetting, uint8_t num_sens = 0, SPIClassRP2040 *spi = &SPI); // Done

    /*Intitalize data and mention total number of both sensors
   MAX num_EGP is set to 8
    */
    void initSensor();

    /**
     * @brief update the number of EGP in a string
     * @param num_EGP number of EGP in a string. MAX 8 */
    void updateEGPCount(uint8_t num_EGP);

    /**
     * @brief update number of sensors in Odometer
     * @param num_Sens number of sensors in odometer. default 3
     */
    void updateODOsensCount(uint8_t num_Sens = 3);

    // Initialize CS Pins
    void LDC1101(int CS);
    void TMAG5170(int CS);

    /**
     * @brief check MFL sensor head with 4 LDC and 8 TMAG cs pin start at 2
     * @return true if sensors are ok else false*/
    bool CheckMFLSensors();

    /**
     * @brief check ODO sensors
     * @return true if sensors are ok else false */
    bool CheckODOSensors();

    /**
     * @brief check EGP sensors
     * @return true if sensors are ok else false*/
    bool CheckEGPSensors();

    // Configure all Sensors
    void ConfigureSensors();

    /**
     * @brief configure MFL sensor head with 4 LDC and 8 TMAG cs pin start at 2
     * @param axis_en magnatic axis to be enable. Default XYZ_AXIS
     */
    void configMFL(magAxisEN axis_en = XYZ_AXIS);

    /**
     * @brief configure ODO sensor
     * @param ang_axis axis for angle calculation. Default XY_ANGLE
     * @param axis_en magnatic axis to be enable. Default XYZ_AXIS
     */
    void configODO(angleAxis ang_axis = XY_ANGLE, magAxisEN axis_en = XYZ_AXIS);

    /**
     * @brief configure EGP sensors
     * @param ang_axis axis for angle calculation. Default XY_ANGLE
     * @param axis_en magnatic axis to be enable. Default XYZ_AXIS
     */
    void configEGP(angleAxis ang_axis = XY_ANGLE, magAxisEN axis_en = XYZ_AXIS);

    void runMFL(TMAGdataType dataType = XZ);

    /**
     * @brief put odo sensors in run mode
     * @param dataType data type for magnetic sensor. default value XZ
     */
    void runODO(TMAGdataType dataType = XZ);

    void runEGP(TMAGdataType dataType = XZ);

    // angle mode enable for selected cs pin
    void angleOffsetConfig(unsigned int Cspin);
    // Put sensors in Run Mode
    void putSensorsInRunMode(bool Special32BitEN); // Done

    void calibrateAngle();

    float liveReadAngle();

    void livereadMFL(uint8_t Channel = X_CH_RESULT, bool special32bitEN = true, bool sdCardConnected = false);

    /**
     * @brief Print sensor values of ODO meter
     * @param Channel select magnatic axis to read data values to pass X_CH_RESULT, Y_CH_RESULT, Z_CH_RESULT. Default X_CH_RESULT
     * @param special32bitEN true to read two axis data at once. default true
     */
    void liveReadODO(uint8_t Channel = X_CH_RESULT, bool special32bitEN = true);

    void liveReadEGP(uint8_t Channel = X_CH_RESULT, bool special32bitEN = true, bool sdCardConnected = false);
    
    // Low Frequency Readout on Serial
    void liveReadSensors(bool Special32BitEN); // Done
    // enable debug functionality
    void enableDebugging(bool en); // Done

    /*----------------PENDING---------------*/
    // Diagnose the sensors - this will put them in sleep mode and then diagnose them
    void DiagnoseSensors();
    // For low Power and Diagnosis
    void putSensorsInSleep(bool Special32BitEN);

    void getMFLdata(uint8_t *buffer, uint8_t Channel = X_CH_RESULT);

    void getODOdata(uint8_t *buffer, uint8_t Channel = X_CH_RESULT);

    void getODOdata1CH(uint8_t *buffer, uint8_t Channel = X_CH_RESULT);

    void getEGPdata(uint8_t *buffer, uint8_t Channel = X_CH_RESULT);

    // Run Sensors - Put this in loop to get updates from sensors
    // as frequently as possible
    void UpdateSensors(bool Special32BitEN, byte *buffer);

    void dma_fetch_ldcdata(byte *buffer, uint32_t sample_count);
    void dma_setup();
    void spi_dma_transfer(uint8_t *tx_data, uint8_t *rx_data, size_t len);

     bool checkTMAG(uint8_t cs);
    bool checkLDC(uint8_t cs);
    void configTMAG(uint8_t cs, angleAxis ang_axis, magAxisEN mag_axis);
    void configLDC(uint8_t cs);
    void PutLDCInRunMode(uint8_t cs);
    void PutTMAGInRunMode(uint8_t cs, TMAGdataType DataType);

    /**
     * @brief print result of TMAG sensor value for selected channel
     * @param Channel channel to be read. pass X_CH_RESULT  Y_CH_RESULT  Z_CH_RESULT
     * @param special32bitEN true if two channel to be read at once is enabled
     */
    void LiveReadTMAG(uint8_t Channel, bool special32bitEN);
    void LiveReadTMAGFormatted(uint8_t Channel, bool special32bitEN);

    uint32_t liveReadLDC();

private:
    Debugger debug;
    ShiftRegister *SR;
    SPIClassRP2040 *_myspi;
    SPISettings SensorSPI;
    const float thxHiLimit = 15, thxLoLimit = -15; //for odo sensor head
    const uint8_t CSstartPin = 2, numLDC = 4, numTMAG = 8; // for mfl sensor head
    uint8_t num_sensors = 0;
 
    int16_t sensorVal = 0;
    float actualVal = 0;
    float angleCalVal[MAX_NUM_ANG_TMAG];
    int LDC_CSpin[4];             // Array to store CS pins of LDC1101 sensors
    int TMAG_CSpin[MAX_NUM_TMAG]; // Array to store CS pins of Hall Effect sensors
    int numLDC1101, totalLDC1101; // Number of LDC1101 sensors initialized
    int numTMAG5170, totalTMAG5170;
    int dma_channel2;
    uint8_t LDC_data_add[3] = {0xB8, 0x00, 0x00};
    uint8_t TMAG_data_add[4] = {0x89, 0x00, 0x00, 0xff};
    int dma_tx_channel, dma_rx_channel;

   

}; // class TMAG5170

#endif