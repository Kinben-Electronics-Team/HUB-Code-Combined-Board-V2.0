#ifndef TEST_PROTOCOL_H
#define TEST_PROTOCOL_H

#include <stdint.h>

// Command types for testing
enum TestCommandType {
    CMD_PING = 0x01,        // Simple connectivity test
    CMD_SET_ID = 0x02,      // Configure slot ID
    CMD_GET_STATUS = 0x03,  // Request slot status
    CMD_LED_CONTROL = 0x04, // Control LED for visual feedback
    CMD_POWER_TEST = 0x05,  // Test power control via IO Expander
    CMD_TRIGGER = 0x10      // Time-critical trigger (no immediate ACK required)
};

// Error codes
enum TestErrorCode {
    ERR_NONE = 0x00,
    ERR_CRC = 0x01,
    ERR_TIMEOUT = 0x02,
    ERR_INVALID_CMD = 0x03,
    ERR_BUSY = 0x04,
    ERR_COMM = 0x05
};

// Simple 8-byte command structure
struct TestCommand {
    uint8_t cmd_id;         // Unique incrementing ID for tracking
    uint8_t cmd_type;       // TestCommandType
    uint8_t slot_mask;      // Bit field: which slots should respond
    uint8_t payload[3];     // Command-specific data
    uint16_t crc16;         // CRC for integrity check
} __attribute__((packed));

// Simple 8-byte response structure  
struct TestResponse {
    uint8_t cmd_id;         // Echo of command ID
    uint8_t slot_id;        // Which slot is responding (1-5)
    uint8_t status;         // 0=success, >0=error code
    uint8_t data[3];        // Response data
    uint16_t crc16;         // CRC for integrity check
} __attribute__((packed));

// Timeout and retry configuration
#define DEFAULT_TIMEOUT_MS 100
#define MAX_RETRIES 3
#define RETRY_DELAY_MS 50

// Communication parameters
#define SERIAL_BAUD 921600
#define DEBUG_BAUD 115200
#define I2C_FREQ 100000

// Slot configuration
#define MAX_SLOTS 5
#define SLOT_BASE_ADDR 0x40

#endif // TEST_PROTOCOL_H