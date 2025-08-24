#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>
#include <stddef.h>

// CRC16 CCITT-FALSE calculation
inline uint16_t calculateCRC16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Calculate CRC for TestCommand (excluding CRC field itself)
template<typename T>
inline uint16_t calculateStructCRC(const T& obj) {
    return calculateCRC16((const uint8_t*)&obj, sizeof(T) - sizeof(uint16_t));
}

// Verify CRC for incoming data
template<typename T>
inline bool verifyCRC(const T& obj) {
    uint16_t calculated = calculateStructCRC(obj);
    return calculated == obj.crc16;
}

#endif // CRC16_H