/**
 * @file slot_debug.cpp
 * @brief Implementation of dedicated debugging and live readout module
 * 
 * This module provides centralized debugging functionality for slot devices,
 * isolating display logic from sensor libraries and providing integrated
 * I2C communication monitoring.
 * 
 * NOTE: This file should only be compiled for BUILD_SLOT builds
 */

#ifdef BUILD_SLOT  // Only compile for slot builds

#include "slot_debug.h"
#include "function.h"  // For sensor access
#include <stdarg.h>

// Global debug instance
SlotDebugger* g_slot_debugger = nullptr;

// External variables from slot firmware
extern uint8_t SID, HID;
extern uint8_t last_query_cmd;

SlotDebugger::SlotDebugger(DebugDisplayMode mode, unsigned long interval_ms) 
    : display_mode(mode), display_enabled(true), sensor_interface(nullptr),
      cmd_history_index(0), last_display_time(0)
{
    setDisplayInterval(interval_ms);
    
    // Initialize I2C status structure
    memset(&i2c_status, 0, sizeof(i2c_status));
    memset(cmd_history, 0, sizeof(cmd_history));
    
    // Set global instance
    g_slot_debugger = this;
}

void SlotDebugger::setSensorInterface(Sensors* sensors)
{
    sensor_interface = sensors;
}

void SlotDebugger::updateI2CStatus(const I2CStatusInfo& status)
{
    memcpy(&i2c_status, &status, sizeof(status));
}

void SlotDebugger::logI2CCommand(uint8_t cmd, const uint8_t* data, uint8_t data_len,
                                bool is_query, bool is_broadcast, bool was_processed)
{
    // Store in circular buffer
    I2CCommandHistory& entry = cmd_history[cmd_history_index];
    entry.cmd = cmd;
    entry.data_len = min(data_len, (uint8_t)4);
    entry.timestamp = millis();
    entry.was_query = is_query;
    entry.was_broadcast = is_broadcast;
    entry.was_processed = was_processed;
    
    // Copy data if provided
    if (data && entry.data_len > 0) {
        memcpy(entry.data, data, entry.data_len);
    } else {
        memset(entry.data, 0, 4);
    }
    
    // Advance circular buffer
    cmd_history_index = (cmd_history_index + 1) % DEBUG_HISTORY_SIZE;
    
    // Update I2C statistics
    i2c_status.total_commands_received++;
    i2c_status.last_activity = millis();
    
    if (was_processed) {
        i2c_status.commands_processed++;
        if (is_query) i2c_status.queries_processed++;
        if (is_broadcast) i2c_status.broadcasts_received++;
    }
}

void SlotDebugger::update()
{
    if (!display_enabled) return;
    
    unsigned long current_time = millis();
    if (current_time - last_display_time >= display_interval_ms) {
        forceUpdate();
    }
}

void SlotDebugger::forceUpdate()
{
    if (!display_enabled) return;
    
    switch (display_mode) {
        case DEBUG_MODE_I2C_ONLY:
            clearScreen();
            displayI2CHeader();
            displayI2CStats();
            displayI2CCommandHistory();
            break;
            
        case DEBUG_MODE_SENSOR_ONLY:
            // Let sensor libraries handle their own display
            // This mode just provides timing control
            break;
            
        case DEBUG_MODE_COMBINED:
            clearScreen();
            displayI2CHeader();
            displayI2CStats();
            displayI2CCommandHistory();
            break;
            
        case DEBUG_MODE_MINIMAL:
            // Minimal output - just basic status line
            Serial.printf("[SLOT%d] I2C:0x%02X RX:%lu Active:%lus ago\\n",
                         SID, i2c_status.slave_address, 
                         i2c_status.total_commands_received,
                         (millis() - i2c_status.last_activity) / 1000);
            break;
    }
    
    last_display_time = millis();
}

void SlotDebugger::liveReadMFL(uint8_t channel, bool special32bit, bool sd_connected)
{
    if (!display_enabled) return;
    
    if (display_mode == DEBUG_MODE_SENSOR_ONLY) {
        // Just call the original sensor library function for sensor-only mode
        if (sensor_interface) {
            sensor_interface->livereadMFL(channel, special32bit, sd_connected);
        }
        return;
    }
    
    if (display_mode == DEBUG_MODE_I2C_ONLY) {
        // I2C status only
        clearScreen();
        displayI2CHeader();
        displayI2CStats();
        displayI2CCommandHistory();
        Serial.println("╚════════════════════════════════════════════════════════╝");
        last_display_time = millis();
        return;
    }
    
    if (display_mode == DEBUG_MODE_COMBINED) {
        // Beautiful comprehensive debug display
        displayComprehensiveMFLDebug(channel, special32bit, sd_connected);
        last_display_time = millis();
        return;
    }
    
    // Fallback - just call original
    if (sensor_interface) {
        sensor_interface->livereadMFL(channel, special32bit, sd_connected);
    }
}

void SlotDebugger::liveReadEGP(uint8_t channel, bool special32bit, bool sd_connected)
{
    if (!display_enabled) return;
    
    if (display_mode == DEBUG_MODE_SENSOR_ONLY) {
        // Just call the original sensor library function for sensor-only mode
        if (sensor_interface) {
            sensor_interface->liveReadEGP(channel, special32bit, sd_connected);
        }
        return;
    }
    
    if (display_mode == DEBUG_MODE_I2C_ONLY) {
        // I2C status only
        clearScreen();
        displayI2CHeader();
        displayI2CStats();
        displayI2CCommandHistory();
        Serial.println("╚════════════════════════════════════════════════════════╝");
        last_display_time = millis();
        return;
    }
    
    if (display_mode == DEBUG_MODE_COMBINED) {
        // Beautiful comprehensive debug display
        displayComprehensiveEGPDebug(channel, special32bit, sd_connected);
        last_display_time = millis();
        return;
    }
    
    // Fallback - just call original
    if (sensor_interface) {
        sensor_interface->liveReadEGP(channel, special32bit, sd_connected);
    }
}

void SlotDebugger::clearScreen()
{
    Serial.print("\\033[2J\\033[H"); // ANSI clear screen and move cursor to top
}

void SlotDebugger::displayI2CHeader()
{
    Serial.println("╔════════════════════════════════════════════════════════╗");
    Serial.println("║                    SLOT I2C STATUS                    ║");
    Serial.println("╠════════════════════════════════════════════════════════╣");
    
    // I2C Configuration
    Serial.printf("║ I2C Address: 0x%02X │ Speed: %lu Hz │ Status: %s ║\\n",
                 i2c_status.slave_address, i2c_status.clock_speed,
                 i2c_status.is_initialized ? "READY" : "ERROR");
                 
    // Slot identification
    Serial.printf("║ Slot ID: %-2d         │ Hub ID: %-2d        │ Build: %-3d   ║\\n",
                 SID, HID, DEFAULT_SLOT_ID);
}

void SlotDebugger::displayI2CStats() 
{
    uint32_t seconds_since_activity = (millis() - i2c_status.last_activity) / 1000;
    Serial.printf("║ Last Activity: %lu sec ago │ Query Pending: 0x%02X     ║\\n", 
                 seconds_since_activity, last_query_cmd);
    Serial.println("║                                                        ║");
    
    // Statistics
    Serial.printf("║ Total RX: %-4lu │ Processed: %-4lu │ Broadcasts: %-4lu ║\\n", 
                 i2c_status.total_commands_received,
                 i2c_status.commands_processed, 
                 i2c_status.broadcasts_received);
    Serial.printf("║ Queries: %-5lu │ Build Default: %-2d │              ║\\n", 
                 i2c_status.queries_processed, DEFAULT_SLOT_ID);
    Serial.println("║                                                        ║");
}

void SlotDebugger::displayI2CCommandHistory()
{
    Serial.println("║ RECENT COMMANDS PROCESSED BY THIS SLOT:               ║");
    uint32_t current_time = millis();
    int displayed_count = 0;
    
    // Display commands in reverse chronological order (newest first)
    for (int i = 0; i < DEBUG_HISTORY_SIZE; i++) {
        int idx = (cmd_history_index - 1 - i + DEBUG_HISTORY_SIZE) % DEBUG_HISTORY_SIZE;
        const I2CCommandHistory& entry = cmd_history[idx];
        
        if (entry.timestamp > 0 && entry.was_processed) {
            uint32_t age_sec = (current_time - entry.timestamp) / 1000;
            const char* cmd_name = "OTHER";
            
            // Decode command names
            switch(entry.cmd) {
                case 0x04: cmd_name = "SD_CON"; break;
                case 0x05: cmd_name = "SD_DISCON"; break;
                case 0x06: cmd_name = "SET_SID"; break;
                case 0x07: cmd_name = "SET_HID"; break;
                case 0x10: cmd_name = "GET_STATUS"; break;
                case 0x11: cmd_name = "GET_SENSORS"; break;
                case 0x12: cmd_name = "GET_CONFIG"; break;
                case 0x13: cmd_name = "PING"; break;
                default: cmd_name = "OTHER"; break;
            }
            
            char data_str[20] = "";
            for (int j = 0; j < entry.data_len && j < 4; j++) {
                char hex_byte[6];
                snprintf(hex_byte, sizeof(hex_byte), "%s%02X", j > 0 ? " " : "", entry.data[j]);
                strcat(data_str, hex_byte);
            }
            
            Serial.printf("║ %d. %s %s [%s] %s %ds ago                    ║\\n", 
                         displayed_count + 1,
                         entry.was_query ? "🔍" : "⚙️",
                         cmd_name,
                         data_str,
                         entry.was_broadcast ? "BROADCAST" : "DIRECT   ",
                         age_sec);
            displayed_count++;
        }
    }
    
    if (displayed_count == 0) {
        Serial.println("║                  (No commands processed yet)          ║");
    }
    
    // Fill remaining lines if needed
    for (int i = displayed_count; i < 5; i++) {
        Serial.println("║                                                        ║");
    }
}

void SlotDebugger::displaySensorHeader()
{
#ifdef EGP
    Serial.println("║                    EGP SENSOR DATA                     ║");
#else
    Serial.println("║                    MFL SENSOR DATA                     ║");
#endif
    Serial.println("╠════════════════════════════════════════════════════════╣");
}

void SlotDebugger::displayMFLSensors(uint8_t channel, bool special32bit, bool sd_connected)
{
    Serial.println("║ [MFL SENSOR DATA - Placeholder]                       ║");
    Serial.println("║ Sensor integration requires access to sensor library  ║");
    Serial.println("║ Real implementation will call sensor->liveReadLDC()   ║");
    Serial.println("║ and sensor->liveReadTMAG() functions                  ║");
}

void SlotDebugger::displayEGPSensors(uint8_t channel, bool special32bit, bool sd_connected) 
{
    Serial.println("║ [EGP SENSOR DATA - Placeholder]                       ║");
    Serial.println("║ Sensor integration requires access to sensor library  ║");
    Serial.println("║ Real implementation will call sensor->liveReadAngle() ║");
    Serial.println("║ and sensor->liveReadTMAG() functions                  ║");
}

void SlotDebugger::displayComprehensiveMFLDebug(uint8_t channel, bool special32bit, bool sd_connected)
{
    clearScreen();
    
    // Clean, aligned debug display with consistent column widths
    extern uint8_t SID, HID;
    uint32_t uptime_sec = millis() / 1000;
    uint32_t uptime_min = uptime_sec / 60;
    uint32_t uptime_hr = uptime_min / 60;
    uint32_t seconds_since_activity = (millis() - i2c_status.last_activity) / 1000;
    
    Serial.println("╔══════════════════════════════════════════════════════════════════════╗");
    Serial.println("║                         🧲 MFL SENSOR DEBUG                          ║");
    Serial.println("╠══════════════════════════════════════════════════════════════════════╣");
    Serial.printf("║ SLOT: %d │ HUB: %d │ UP: %02luh:%02lum │ I2C: 0x%02X │ SD: %s │ CH: %d     ║\n",
                 SID, HID, uptime_hr, uptime_min % 60, i2c_status.slave_address, 
                 sd_connected ? "✅" : "❌", channel);
    Serial.printf("║ I2C RX: %3lu │ PROC: %3lu │ QUERY: %3lu │ LAST: %3lus │ MODE: %s  ║\n", 
                 i2c_status.total_commands_received, i2c_status.commands_processed,
                 i2c_status.queries_processed, seconds_since_activity,
                 special32bit ? "32BIT" : "16BIT");
    Serial.println("╠══════════════════════════════════════════════════════════════════════╣");
    
    // MFL sensor data with consistent alignment (70 chars wide)
    Serial.println("║ ID   │  TYPE  │   VALUE    │ BAR               │ STATUS │ HEALTH ║");
    Serial.println("╠══════════════════════════════════════════════════════════════════════╣");
    
    // FAKE DATA GENERATION REMOVED - This was generating fake LDC values
    // Real sensor data is now provided by DEBUG_MODE_SENSOR_ONLY
    Serial.println("║ FAKE DATA GENERATION DISABLED - USE DEBUG_MODE_SENSOR_ONLY           ║");
    /*
    // LDC Inductance sensors (first 4 sensors)
    for (int i = 0; i < 4; i++) {
        uint32_t ldc_value = 120000 + (millis() % 1000) * (i + 1);  // FAKE DATA!
        int bar_len = ((ldc_value % 1000) / 50) * 18 / 20; // Scale to 18 chars
        
        char id_str[6], type_str[8], value_str[12], bar_str[20], status_str[10], health_str[10];
        
        snprintf(id_str, sizeof(id_str), "%02d", i);
        strcpy(type_str, "  LDC  ");
        snprintf(value_str, sizeof(value_str), "%7luuH", ldc_value);
        strcpy(status_str, ldc_value > 125000 ? "  OK  " : " LOW ");
        strcpy(health_str, ldc_value > 122000 ? " GOOD " : " WEAK ");
        
        // Create 18-char progress bar using ASCII characters
        for (int j = 0; j < 18; j++) {
            bar_str[j] = (j < bar_len) ? '#' : '.';
        }
        bar_str[18] = '\\0';
        
        Serial.printf("║ %s │ %s │ %s │ %s │ %s │ %s ║\n",
                     id_str, type_str, value_str, bar_str, status_str, health_str);
    }
    */ // End of commented out LDC fake data
    
    // FAKE TMAG DATA GENERATION REMOVED 
    // Real sensor data is now provided by DEBUG_MODE_SENSOR_ONLY
    /*
    // TMAG Magnetic sensors (next 8 sensors)  
    for (int i = 0; i < 8; i++) {
        float mag_value = -50.0f + (millis() % 2000) / 20.0f + i * 15.0f;  // FAKE DATA!
        int bar_len = (int)((mag_value + 100.0f) / 10.0f);
        bar_len = max(0, min(18, bar_len));
        
        char id_str[6], type_str[8], value_str[12], bar_str[20], status_str[10], health_str[10];
        
        snprintf(id_str, sizeof(id_str), "%02d", i + 4); // Continue numbering from LDC sensors
        strcpy(type_str, " TMAG  ");
        snprintf(value_str, sizeof(value_str), "%+6.1fmT", mag_value);
        strcpy(status_str, abs(mag_value) < 30.0f ? "  OK  " : " HIGH ");
        strcpy(health_str, abs(mag_value) < 40.0f ? " GOOD " : "ALERT ");
        
        // Create 18-char progress bar using ASCII characters
        for (int j = 0; j < 18; j++) {
            bar_str[j] = (j < bar_len) ? '#' : '.';
        }
        bar_str[18] = '\\0';
        
        Serial.printf("║ %s │ %s │ %s │ %s │ %s │ %s ║\n",
                     id_str, type_str, value_str, bar_str, status_str, health_str);
    }
    */ // End of commented out TMAG fake data
    
    Serial.println("╠══════════════════════════════════════════════════════════════════════╣");
    // FAKE SYSTEM STATS DISABLED - These were hardcoded fake values
    Serial.printf("║ SYSTEM: FAKE STATS DISABLED - Use real monitoring tools          ║\n");
    // Serial.printf("║ SYSTEM: CPU %2d%% │ RAM %4.1fKB │ TEMP %4.1f°C │ LOOP %4.1fms │ STATUS ║\n",
    //              85, 47.6f, 42.3f, 1.2f);  // THESE WERE FAKE VALUES!
    Serial.println("╚══════════════════════════════════════════════════════════════════════╝");
}

void SlotDebugger::displayComprehensiveEGPDebug(uint8_t channel, bool special32bit, bool sd_connected)
{
    clearScreen();
    
    // Clean, aligned debug display with consistent column widths
    extern uint8_t SID, HID;
    uint32_t uptime_sec = millis() / 1000;
    uint32_t uptime_min = uptime_sec / 60;
    uint32_t uptime_hr = uptime_min / 60;
    uint32_t seconds_since_activity = (millis() - i2c_status.last_activity) / 1000;
    
    Serial.println("╔══════════════════════════════════════════════════════════════════════╗");
    Serial.println("║                         🎯 EGP SENSOR DEBUG                          ║");
    Serial.println("╠══════════════════════════════════════════════════════════════════════╣");
    Serial.printf("║ SLOT: %d │ HUB: %d │ UP: %02luh:%02lum │ I2C: 0x%02X │ SD: %s │ CH: %d     ║\n",
                 SID, HID, uptime_hr, uptime_min % 60, i2c_status.slave_address, 
                 sd_connected ? "✅" : "❌", channel);
    Serial.printf("║ I2C RX: %3lu │ PROC: %3lu │ QUERY: %3lu │ LAST: %3lus │ MODE: %s  ║\n", 
                 i2c_status.total_commands_received, i2c_status.commands_processed,
                 i2c_status.queries_processed, seconds_since_activity,
                 special32bit ? "32BIT" : "16BIT");
    Serial.println("╠══════════════════════════════════════════════════════════════════════╣");
    
    // EGP sensor data with consistent alignment (70 chars wide)
    Serial.println("║ ID   │  TYPE  │   VALUE    │ BAR               │ STATUS │ HEALTH ║");
    Serial.println("╠══════════════════════════════════════════════════════════════════════╣");
    
    for (int i = 0; i < 20; i++) {
        char id_str[6], type_str[8], value_str[12], bar_str[20], status_str[10], health_str[10];
        
        snprintf(id_str, sizeof(id_str), "%02d", i);
        
        if (i % 4 == 0) {
            // FAKE ANGLE DATA GENERATION DISABLED
            // Real sensor data is now provided by DEBUG_MODE_SENSOR_ONLY
            /*
            // Angle sensor
            float angle = fmod(45.0f + (millis() / 100.0f) + i * 18.0f, 360.0f);  // FAKE DATA!
            int bar_len = (int)(angle / 18.0f); // 0-18 scale for 18 char bar
            
            strcpy(type_str, " ANGLE ");
            snprintf(value_str, sizeof(value_str), "%7.1f°", angle);
            strcpy(status_str, (angle > 300 || angle < 60) ? "  OK  " : " WARN ");
            strcpy(health_str, " GOOD ");
            
            // Create 18-char progress bar using ASCII characters
            for (int j = 0; j < 18; j++) {
                bar_str[j] = (j < bar_len) ? '#' : '.';
            }
            bar_str[18] = '\0';
            */ // End of commented out angle fake data
        } else {
            // FAKE MAGNETIC FIELD DATA GENERATION DISABLED  
            /*
            // Magnetic field sensor
            float mag_field = -80.0f + (millis() % 3000) / 18.75f + i * 8.0f;  // FAKE DATA!
            int bar_len = (int)((mag_field + 100.0f) / 10.0f);
            bar_len = max(0, min(18, bar_len));
            
            strcpy(type_str, "  MAG  ");
            snprintf(value_str, sizeof(value_str), "%+6.1fmT", mag_field);
            strcpy(status_str, abs(mag_field) < 50.0f ? "  OK  " : " HIGH ");
            strcpy(health_str, abs(mag_field) < 30.0f ? " GOOD " : "ALERT ");
            
            // Create 18-char progress bar using ASCII characters
            for (int j = 0; j < 18; j++) {
                bar_str[j] = (j < bar_len) ? '#' : '.';
            }
            bar_str[18] = '\0';
            */ // End of commented out magnetic field fake data
        }
        
        Serial.printf("║ %s │ %s │ %s │ %s │ %s │ %s ║\n",
                     id_str, type_str, value_str, bar_str, status_str, health_str);
    }
    
    Serial.println("╠══════════════════════════════════════════════════════════════════════╣");
    // FAKE EGP SYSTEM STATS DISABLED - These were hardcoded fake values  
    Serial.printf("║ SYSTEM: FAKE STATS DISABLED - Use real monitoring tools          ║\n");
    // Serial.printf("║ SYSTEM: CPU %2d%% │ RAM %4.1fKB │ TEMP %4.1f°C │ LOOP %4.1fms │ STATUS ║\n",
    //              78, 47.6f, 39.8f, 0.9f);  // THESE WERE FAKE VALUES!
    Serial.println("╚══════════════════════════════════════════════════════════════════════╝");
}

void SlotDebugger::setEnabled(bool enabled)
{
    display_enabled = enabled;
}

void SlotDebugger::setDisplayMode(DebugDisplayMode mode)
{
    display_mode = mode;
}

void SlotDebugger::setDisplayInterval(unsigned long interval_ms)
{
    display_interval_ms = max(interval_ms, (unsigned long)DEBUG_MIN_INTERVAL_MS);
}

I2CStatusInfo SlotDebugger::getI2CStatus() const
{
    return i2c_status;
}

void SlotDebugger::debugPrint(const char* message)
{
    Serial.print("🐞 DEBUG: ");
    Serial.println(message);
}

void SlotDebugger::debugPrintf(const char* format, ...)
{
    Serial.print("🐞 DEBUG: ");
    
    va_list args;
    va_start(args, format);
    
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);
    Serial.println(buffer);
    
    va_end(args);
}

#endif // BUILD_SLOT