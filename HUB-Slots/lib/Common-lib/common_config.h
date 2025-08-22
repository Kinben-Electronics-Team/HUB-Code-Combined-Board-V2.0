#ifndef COMMON_CONFIG_H
#define COMMON_CONFIG_H

#include <Arduino.h>
#define COM_BAUDRATE 921600 // common baudrate for communication with master

#define ACQ_MODE 1
#define NOT_ACQ_MODE !ACQ_MODE

#define SENSOR_TYPE_MFL 0                   // sensor type MFL
#define SENSOR_TYPE_EGP 1                   // sensor type EGP
#define DEFAULT_SENSOR_TYPE SENSOR_TYPE_MFL // default sensor type

const uint8_t startBytes[] = {
    0xF1, 0xE2, 0xD3, 0xC4, 0xB5, 0xA6, 0x97, 0x88, 0x79, 0x6A, 0x5B, 0x4C, 0x3D, 0x2E, 0x1F, 0x10,
    0x01, 0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89, 0x9A, 0xAB, 0xBC, 0xCD, 0xDE, 0xEF, 0xF0};

enum hubParam
{
    set_sid = 1,     // cmd set SID for the hub board
    set_hid,         // cmd set HID for the hub board
    set_num_sensor,  // cmd set number of sensors for the hub board
    set_sensor_type, // cmd set sensor type for the hub board EGP/MFL
    set_mag_axis,    // cmd set magnetic axis for the hub board
    set_ang_axis,    // cmd set angular axis for the hub board

    cmd_sd_con,    // command to connect sd card to card reader
    cmd_sd_discon, // command to disconect sd card from sd card reader ic

    cmd_hub_boot, // command to bootup hub board mcu
    cmd_hub_sel,  // command to select hub board
    cmd_pwr_en,   // command to enable disable power to hub board

    cmd_get_data,  // command to get sensor data from hub board
    cmd_get_config // command to get configuration data from hub board

};

struct cmd_t
{
    uint8_t BoardID;
    uint8_t param; // command to be sent to the hub board
    uint8_t value; // data to be sent to the hub board
};

#endif // COMMON_CONFIG_H