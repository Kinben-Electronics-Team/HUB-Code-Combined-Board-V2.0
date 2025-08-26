/**
 * @file slot_debug.h
 * @brief Dedicated debugging and live readout module for slot devices
 * 
 * This module isolates all debugging/display functionality from sensor libraries,
 * providing centralized control over serial output formatting and I2C status monitoring.
 * 
 * Features:
 * - Live sensor data display with configurable intervals
 * - I2C communication status and command history
 * - Slot configuration and health monitoring
 * - Unified display formatting for all debug outputs
 */

#ifndef SLOT_DEBUG_H
#define SLOT_DEBUG_H

// Only provide slot debugging for BUILD_SLOT builds
#ifdef BUILD_SLOT

#include <Arduino.h>

// Forward declarations
class Sensors;

// Debug display configuration
#define DEBUG_DISPLAY_WIDTH 70
#define DEBUG_HISTORY_SIZE 8
#define DEBUG_MIN_INTERVAL_MS 50   // Minimum display interval (MAXIMUM SPEED!)

// Debug display modes
enum DebugDisplayMode {
    DEBUG_MODE_SENSOR_ONLY,      // Show only sensor data
    DEBUG_MODE_I2C_ONLY,         // Show only I2C status
    DEBUG_MODE_COMBINED,         // Show both sensor + I2C (default)
    DEBUG_MODE_MINIMAL          // Minimal output mode
};

// I2C command history for debugging
struct I2CCommandHistory {
    uint8_t cmd;
    uint8_t data_len;
    uint8_t data[4];
    uint32_t timestamp;
    bool was_query;
    bool was_broadcast;
    bool was_processed;
};

// I2C status tracking
struct I2CStatusInfo {
    uint8_t slave_address;
    uint32_t clock_speed;
    bool is_initialized;
    uint32_t total_commands_received;
    uint32_t commands_processed;
    uint32_t broadcasts_received;
    uint32_t queries_processed;
    uint32_t last_activity;
    uint8_t last_query_cmd;
};

/**
 * @brief Centralized debugging and live readout controller
 * 
 * This class handles all debugging output for slot devices, isolating
 * display functionality from sensor and communication libraries.
 */
class SlotDebugger {
private:
    // Display configuration
    DebugDisplayMode display_mode;
    unsigned long display_interval_ms;
    unsigned long last_display_time;
    bool display_enabled;
    
    // I2C monitoring
    I2CStatusInfo i2c_status;
    I2CCommandHistory cmd_history[DEBUG_HISTORY_SIZE];
    uint8_t cmd_history_index;
    
    // Sensor interface
    Sensors* sensor_interface;
    
    // Display helpers
    void clearScreen();
    void printBorder(char corner_tl, char corner_tr, char corner_bl, char corner_br, char horizontal, char vertical);
    void printCenteredText(const char* text);
    void printFormattedLine(const char* format, ...);
    
    // I2C display functions
    void displayI2CHeader();
    void displayI2CStats();
    void displayI2CCommandHistory();
    
    // Sensor display functions  
    void displaySensorHeader();
    void displayMFLSensors(uint8_t channel, bool special32bit, bool sd_connected);
    void displayEGPSensors(uint8_t channel, bool special32bit, bool sd_connected);
    
    // Comprehensive debug displays
    void displayComprehensiveMFLDebug(uint8_t channel, bool special32bit, bool sd_connected);
    void displayComprehensiveEGPDebug(uint8_t channel, bool special32bit, bool sd_connected);

public:
    /**
     * @brief Initialize the debug system
     * @param mode Display mode (combined, sensor only, I2C only, minimal)
     * @param interval_ms Update interval in milliseconds (min 50ms)
     */
    SlotDebugger(DebugDisplayMode mode = DEBUG_MODE_COMBINED, unsigned long interval_ms = 100);
    
    /**
     * @brief Set the sensor interface for data retrieval
     * @param sensors Pointer to initialized Sensors object
     */
    void setSensorInterface(Sensors* sensors);
    
    /**
     * @brief Update I2C status information
     * @param status Updated I2C status structure
     */
    void updateI2CStatus(const I2CStatusInfo& status);
    
    /**
     * @brief Add I2C command to history
     * @param cmd Command byte received
     * @param data Data bytes (can be NULL)
     * @param data_len Length of data
     * @param is_query True if this was a query command
     * @param is_broadcast True if this was broadcast to all slots
     * @param was_processed True if command was successfully processed
     */
    void logI2CCommand(uint8_t cmd, const uint8_t* data, uint8_t data_len, 
                       bool is_query, bool is_broadcast, bool was_processed);
    
    /**
     * @brief Main update function - call this in loop()
     * Should be called frequently to maintain timing
     */
    void update();
    
    /**
     * @brief Live readout for MFL sensors (replaces sensor library version)
     * @param channel Sensor channel to read
     * @param special32bit Enable 32-bit special mode
     * @param sd_connected SD card connection status
     */
    void liveReadMFL(uint8_t channel = 0, bool special32bit = true, bool sd_connected = false);
    
    /**
     * @brief Live readout for EGP sensors (replaces sensor library version)  
     * @param channel Sensor channel to read
     * @param special32bit Enable 32-bit special mode
     * @param sd_connected SD card connection status
     */
    void liveReadEGP(uint8_t channel = 0, bool special32bit = true, bool sd_connected = false);
    
    /**
     * @brief Force immediate display update (ignores timing interval)
     */
    void forceUpdate();
    
    /**
     * @brief Enable/disable debug display
     * @param enabled True to enable display, false to disable
     */
    void setEnabled(bool enabled);
    
    /**
     * @brief Change display mode
     * @param mode New display mode
     */
    void setDisplayMode(DebugDisplayMode mode);
    
    /**
     * @brief Change update interval
     * @param interval_ms New interval in milliseconds (min 50ms)
     */
    void setDisplayInterval(unsigned long interval_ms);
    
    /**
     * @brief Get current I2C status
     * @return Copy of current I2C status structure
     */
    I2CStatusInfo getI2CStatus() const;
    
    /**
     * @brief Print a one-time debug message (doesn't interfere with live display)
     * @param message Debug message to print
     */
    void debugPrint(const char* message);
    
    /**
     * @brief Print formatted debug message
     * @param format Printf-style format string
     * @param ... Format arguments
     */
    void debugPrintf(const char* format, ...);
};

// Global debug instance (initialized in slot firmware)
extern SlotDebugger* g_slot_debugger;

// Convenience macros for debug output
#define DEBUG_PRINT(msg) if(g_slot_debugger) { g_slot_debugger->debugPrint(msg); }
#define DEBUG_PRINTF(...) if(g_slot_debugger) { g_slot_debugger->debugPrintf(__VA_ARGS__); }

#else // BUILD_SLOT

// Provide empty macros for non-slot builds (master)
#define DEBUG_PRINT(msg) 
#define DEBUG_PRINTF(...)

#endif // BUILD_SLOT

#endif // SLOT_DEBUG_H