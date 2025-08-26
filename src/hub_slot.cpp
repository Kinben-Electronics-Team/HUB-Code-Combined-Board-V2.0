#ifdef BUILD_SLOT
#include "function.h"

// Beta feature: Sample count debugging
bool beta_sample_debug = true; // Enable sample count debugging
uint64_t last_received_sample = 0;
unsigned long sample_receive_count = 0;

// Live readout configuration
unsigned long LIVE_READOUT_INTERVAL_MS = 1000; // Configurable interval for live sensor readout (1000ms = 1 second)

/*varible for psram*/
psram_spi_inst_t psram_spi = psram_spi_init(pio0, -1, false);

/*ISR variables */
volatile bool trig_status = 0;
uint8_t wireData[4];
volatile bool wire_received = 0;
uint8_t last_query_cmd = 0; // Track the last query command for requestEvent response

// Enhanced command history for persistent I2C status display
#define CMD_HISTORY_SIZE 5
struct CommandHistory {
    uint8_t cmd;
    uint8_t data_len;
    uint8_t data[4];  // Store command data
    uint32_t timestamp;
    bool was_query;
    bool was_broadcast;
    bool was_processed;  // Did this slot process it?
} cmd_history[CMD_HISTORY_SIZE];
uint8_t cmd_history_index = 0;

// I2C status tracking
struct I2CStatus {
    uint8_t slave_address;
    uint32_t clock_speed;
    uint32_t total_commands_received;
    uint32_t commands_processed;
    uint32_t broadcasts_received;
    uint32_t queries_processed;
    unsigned long last_activity;
    bool is_initialized;
} i2c_status = {0};

EasyTransfer EscRX;

SPISettings SPIset(SPI_FREQ, MSBFIRST, SPI_MODE0);
ShiftRegister sr(2, SRDA_pin, STCLK_pin, -1);
Sensors Sens(&sr, SPIset, 2, &SENSOR_SPI);
Acquire core(&psram_spi, &Sens);

void setup_slot()
{
    set_sys_clock_khz(133000, true);
    if (rp2040.getResetReason() == 4)
    {
        rp2040.fifo.push_nb(RESET_WDT_EVENT); // reset watchdog timer
    }

    Serial.begin(BAUDRATE);

    MSTR_Serial.setRX(Rx_Data_pin);
    MSTR_Serial.setTX(Tx_Data_pin);
    MSTR_Serial.begin(COM_BAUDRATE); // begin serial communication with master

    core.begin();
#if defined(EGP)
    core.Sensor_begin(true, 5); // initialise EGP sensors - 5 EGP boards (20 TMAG sensors)
    Serial.println("EGP Sensors initialised - 5 EGPs (20 TMAG sensors)");
#elif defined(MFL)
    core.Sensor_begin(true, 0); // initialise MFL sensors  
    Serial.println("MFL Sensors initialised");
#else
    core.Sensor_begin(true, 0); // default initialization
    Serial.println("Sensors initialised");
#endif

    pinMode(SD_SW_pin, OUTPUT);
    digitalWrite(SD_SW_pin, LOW);                // CONNECT SD CARD TO uC
    
    Serial.println("Requesting SD card initialization...");
    rp2040.fifo.push_nb(MOUNT_SD);               // mount sd card

    // Wait for Core 1 to acknowledge SD initialization
    uint32_t timeout = millis() + 10000; // 10 second timeout
    bool sdInitialized = false;
    
    Serial.println("Waiting for SD initialization...");
    while (millis() < timeout && !sdInitialized) {
        if (rp2040.fifo.available()) {
            uint32_t response = rp2040.fifo.pop();
            if (response == MOUNT_SD_ACK) {
                sdInitialized = true;
                Serial.println("SD card initialized successfully!");
            } else {
                // Put other FIFO messages back for later processing
                rp2040.fifo.push_nb(response);
                Serial.printf("Got FIFO message: %lu (not SD ACK)\n", response);
            }
        }
        delay(10); // Small delay to prevent busy waiting
    }
    
    if (!sdInitialized) {
        Serial.println("ERROR: SD card initialization timeout!");
        Serial.println("Continuing with setup but SD may not work...");
    }

    EscRX.begin(details(core.sc), &MSTR_Serial); // begin EasyTransfer for receiving data from master
    rp2040.wdt_begin(WDT_RESET_TIME);            // start watchdog timer
    Serial.println("Setup complete");
}

void loop_slot()
{

    if (core.check_mode() == ACQ_MODE)
    {
        if (trig_status)
        {
            trig_status = 0;     // reset trigger flag
            EscRX.receiveData(); // receive data from master
            
            // CLEAN: Only track sample count for post-acquisition statistics
            if (beta_sample_debug && core.sc != last_received_sample) {
                sample_receive_count++;
                last_received_sample = core.sc;
            }
            
            core.get_data();     // get data from sensor and log into psram
        }
    }

    else
    {
        // Beta feature: Display sample count statistics (non-intrusive)
        static unsigned long last_stats_time = 0;
        if (beta_sample_debug && millis() - last_stats_time > 5000) { // Every 5 seconds
            if (sample_receive_count > 0) {
                Serial.printf("BETA Stats: Last sample ID: %llu, Total received: %lu\n", 
                             last_received_sample, sample_receive_count);
            }
            last_stats_time = millis();
        }

        if (wire_received)
        {
            Serial.println("Wire data received");
            wire_received = false;
            core.ExecuteWireCmd(wireData[0], wireData[1]);
            memset(wireData, 0, sizeof(wireData));
        }
        
        // Persistent I2C Status Display - every 8 seconds
        static unsigned long last_status_display = 0;
        if (millis() - last_status_display > 8000) {
            extern uint8_t SID; // Actual SID from EEPROM
            
            Serial.println("╔════════════════════════════════════════════════════════╗");
            Serial.printf("║                    I2C STATUS - SLOT %d                  ║\n", SID);
            Serial.println("╠════════════════════════════════════════════════════════╣");
            
            // I2C Configuration and Statistics
            Serial.printf("║ Address: 0x%02X  │ Speed: %d Hz │ Status: %s      ║\n", 
                         i2c_status.slave_address, 
                         i2c_status.clock_speed,
                         i2c_status.is_initialized ? "READY" : "ERROR");
                         
            uint32_t seconds_since_activity = (millis() - i2c_status.last_activity) / 1000;
            Serial.printf("║ Last Activity: %lu sec ago │ Query Pending: 0x%02X     ║\n", 
                         seconds_since_activity, last_query_cmd);
            Serial.println("║                                                        ║");
            
            // Statistics
            Serial.printf("║ Total RX: %-4lu │ Processed: %-4lu │ Broadcasts: %-4lu ║\n", 
                         i2c_status.total_commands_received,
                         i2c_status.commands_processed, 
                         i2c_status.broadcasts_received);
            Serial.printf("║ Queries: %-5lu │ Build Default: %-2d │              ║\n", 
                         i2c_status.queries_processed, DEFAULT_SLOT_ID);
            Serial.println("║                                                        ║");
            
            // Recent Command History (Last 5 commands processed by this slot)
            Serial.println("║ RECENT COMMANDS PROCESSED BY THIS SLOT:               ║");
            uint32_t current_time = millis();
            int displayed_count = 0;
            
            // Display commands in reverse chronological order (newest first)
            for (int i = 0; i < CMD_HISTORY_SIZE; i++) {
                int idx = (cmd_history_index - 1 - i + CMD_HISTORY_SIZE) % CMD_HISTORY_SIZE;
                if (cmd_history[idx].timestamp > 0 && cmd_history[idx].was_processed) {
                    uint32_t age_sec = (current_time - cmd_history[idx].timestamp) / 1000;
                    const char* cmd_name = "OTHER";
                    
                    // Decode command names
                    switch(cmd_history[idx].cmd) {
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
                    for (int j = 0; j < cmd_history[idx].data_len && j < 4; j++) {
                        char hex_byte[6];
                        snprintf(hex_byte, sizeof(hex_byte), "%s%02X", j > 0 ? " " : "", cmd_history[idx].data[j]);
                        strcat(data_str, hex_byte);
                    }
                    
                    Serial.printf("║ %d. %s %s [%s] %s %ds ago                    ║\n", 
                                 displayed_count + 1,
                                 cmd_history[idx].was_query ? "🔍" : "⚙️",
                                 cmd_name,
                                 data_str,
                                 cmd_history[idx].was_broadcast ? "BROADCAST" : "DIRECT   ",
                                 age_sec);
                    displayed_count++;
                }
            }
            
            if (displayed_count == 0) {
                Serial.println("║                  (No commands processed yet)          ║");
            }
            
            // Fill remaining lines if needed
            for (int i = displayed_count; i < CMD_HISTORY_SIZE; i++) {
                Serial.println("║                                                        ║");
            }
            
            Serial.println("╚════════════════════════════════════════════════════════╝");
            last_status_display = millis();
        }
        static unsigned long lastLiveReadTime = 0;
        if (millis() - lastLiveReadTime > LIVE_READOUT_INTERVAL_MS) { // Throttle based on configurable interval
#if defined(MFL)
            Sens.livereadMFL(X_CH_RESULT, true, core.getSDCardStatus());
#else
            Sens.liveReadEGP(X_CH_RESULT, true, core.getSDCardStatus());
#endif
            lastLiveReadTime = millis();
        }
        delay(100); // Main loop delay - prevents busy waiting, allows other processes to run
        rp2040.fifo.push_nb(HEALTH_FLAG); // push health flag to reset watchdog timer
    }
}

/***core1 varibles***/

FsFile file;

Logging l(&file, &psram_spi);

void setup1_slot()
{
    pinMode(SD_VCC_EN_pin, OUTPUT);
    digitalWrite(SD_VCC_EN_pin, HIGH); // power on sd card
    
    // Process any pending FIFO messages during setup
    delay(100); // Let Core 0 send MOUNT_SD
    
    // Check for MOUNT_SD during setup phase
    Serial.println("Core 1 setup checking for MOUNT_SD...");
    if (rp2040.fifo.available()) {
        uint32_t cmd = rp2040.fifo.pop();
        if (cmd == MOUNT_SD) {
            Serial.println("Processing early SD mount request");
            l.sd_card_init();
            rp2040.fifo.push_nb(MOUNT_SD_ACK);
            Serial.println("SD initialization complete, ACK sent");
        } else {
            // Put it back for loop processing
            rp2040.fifo.push_nb(cmd);
            Serial.printf("Got non-MOUNT_SD command in setup1: %lu\n", cmd);
        }
    } else {
        Serial.println("No FIFO messages in Core 1 setup");
    }
}

void loop1_slot()
{

    if (rp2040.fifo.available())
    {
        l.check_fifo(rp2040.fifo.pop()); // check multicore fifo value and execute required action
    }
}

/**
 * \brief interrupt callback function for Wire receive event
 * \param numBytes number of bytes received
 */
void receiveEventCallback(int numBytes)
{
    int i = 0;
    memset(wireData, 0, sizeof(wireData)); // reset wire buffer
    while (Wire.available())
    {
        if (i < 4)
        {
            wireData[i] = Wire.read();
            i++;
        }
        else
        {
            char trash = Wire.read();
        }
    }
    Serial.printf("Wire data received:%d \n", i);
    
    // Process and categorize received commands
    if (i > 0) {
        extern uint8_t SID; // Get current slot's SID
        uint8_t cmd = wireData[0];
        
        // Determine if this command is relevant to this slot
        bool is_broadcast = true;  // All I2C commands via broadcast address (0x00) are broadcasts
        bool is_query = (cmd >= 0x10 && cmd <= 0x13);
        bool should_process = true;  // All broadcasts should be tracked
        
        // Update I2C statistics
        i2c_status.total_commands_received++;
        i2c_status.last_activity = millis();
        
        if (is_broadcast) {
            i2c_status.broadcasts_received++;
        }
        
        if (should_process) {
            i2c_status.commands_processed++;
            if (is_query) {
                i2c_status.queries_processed++;
            }
            
            // Store command in history (only commands processed by this slot)
            cmd_history[cmd_history_index].cmd = cmd;
            cmd_history[cmd_history_index].data_len = i;
            for (int j = 0; j < i && j < 4; j++) {
                cmd_history[cmd_history_index].data[j] = wireData[j];
            }
            cmd_history[cmd_history_index].timestamp = millis();
            cmd_history[cmd_history_index].was_query = is_query;
            cmd_history[cmd_history_index].was_broadcast = is_broadcast;
            cmd_history[cmd_history_index].was_processed = true;
            cmd_history_index = (cmd_history_index + 1) % CMD_HISTORY_SIZE;
            
            Serial.printf("📡 I2C RX: Cmd=0x%02X Data=[", cmd);
            for (int j = 0; j < i; j++) {
                Serial.printf("0x%02X", wireData[j]);
                if (j < i-1) Serial.print(" ");
            }
            Serial.printf("] %s%s\n", 
                         is_broadcast ? "[BROADCAST] " : "[DIRECT] ",
                         is_query ? "[QUERY]" : "[CONFIG]");
        }
        
        if (is_query) {
            last_query_cmd = cmd;
            Serial.printf("🔍 Query command stored: 0x%02X (will respond in requestEvent)\n", cmd);
        }
    }
    
    wire_received = 1;
}

/**
 * \brief Interrupt callback function for Wire request event
 */
void requestEventCallback()
{
    core.requestEvent();
}

/**
 * \brief Interrupt callback function for Interrupt on Trigger pin
 */
void trigger()
{
    trig_status = 1;
}

/**
 * \brief Set live readout interval
 * \param interval_ms New interval in milliseconds (minimum 500ms to prevent serial overload)
 */
void setLiveReadoutInterval(unsigned long interval_ms)
{
    if (interval_ms < 500) {
        interval_ms = 500; // Minimum 500ms to prevent serial port overload
    }
    LIVE_READOUT_INTERVAL_MS = interval_ms;
    Serial.printf("Live readout interval set to %lu ms\n", LIVE_READOUT_INTERVAL_MS);
}

#endif // BUILD_SLOT