#ifdef BUILD_MASTER
#include "hub_master.h"

void receiveEvent(int howMany);
void requestEvent();
void Channel_sel(slot_t Channel);
void trigger();
void HUB_BOOT(slot_t HUB);
void check_mode();
void send_I2C_cmd(slot_t slot, const uint8_t *cmd, uint8_t n);

uint8_t wireBroadcast(const uint8_t *data, uint8_t n);
bool receiveDataWithTimeout(EasyTransfer* et, uint32_t timeout_ms, uint8_t retries);

// Bidirectional communication query functions
bool querySlot(slot_t slot, uint8_t query_cmd, uint8_t* response, uint8_t response_len, uint32_t timeout_ms = 500);
bool getSlotStatus(slot_t slot, uint8_t* status_data);
bool getSlotSensors(slot_t slot, uint16_t* sensor_value);
bool getSlotConfig(slot_t slot, uint8_t* config_data);
bool pingSlot(slot_t slot);

void check_Serial_cmd(uint8_t cmd);
void checkCSB(cmd_t *c);
void displayMainMenu();
void displaySubMenu();
void processMenuInput();
void processSubMenuInput(int cmd);
void processSlotCommand(int cmd);
void processSDCommand(int cmd);
void processConfigCommand(int cmd);
void processTestCommand(int cmd);
bool isValidInput(int input, int minVal, int maxVal);
void powerOnSlot(slot_t slot);
void powerOffSlot(slot_t slot);
bool getSlotPowerStatus(slot_t slot);
void displaySlotStatus();

bool mode_stat = OTHER_MODE, current_mode = 0;
volatile bool Trig_stat = 0;

cmd_t configRX; // struct to receive data from CSB
configData cd;  // struct to store configuration data
uint32_t dummy_log_freq = 1000; // Default dummy logging frequency in Hz
bool dummy_logging_active = false; // Track if dummy logging is active
uint64_t dummy_sample_count = 0; // Sample count for dummy logging
unsigned long last_dummy_trigger_time = 0; // Last time we sent a dummy trigger
uint32_t dummy_trigger_interval = 1000; // Interval in microseconds (will be calculated from frequency)

EasyTransfer EscRX, // EasyTransfer object to get sample count from CSB
    EconfigRX,      // EasyTransfer object to get Cofiguration from CSB
    EstatTX,        // EasyTransfer object to send config data data to CSB
    EscTX;          // EasyTransfer object to send sample count to CSB

uint64_t sc = 0; // sample count to be received from CSB

// Communication statistics
struct CommStats {
  uint32_t i2c_success_count = 0;
  uint32_t i2c_error_count = 0;
  uint32_t easytransfer_success_count = 0;
  uint32_t easytransfer_timeout_count = 0;
  unsigned long last_comm_activity = 0;
};

CommStats comm_stats;

// Menu system variables
bool inMainMenu = true;
bool inSubMenu = false;
int currentSubMenu = 0;

MyI2CDevice IOEX(&IOEX_Wire, 0x74, &Serial); // use actual I²C address macro or value
Adafruit_NeoPixel pixel(1, LED_pin, NEO_GRB + NEO_KHZ800);

void checkCSB(cmd_t *c);

void setup_master()
{
  set_sys_clock_khz(133000, false); // set system clock to 133MHz
  Serial.begin(BAUDRATE);           // set baudrate for serial communication

  /*CSB serial communication init*/
  CSB_Serial.setRX(DATA_RX_pin);
  CSB_Serial.setTX(DATA_TX_pin);
  CSB_Serial.begin(CSB_BR); // Initialize CSB Serial communication

  // --- WS2812B LED Init ---
  pixel.begin();                                  // Initialize NeoPixel library
  pixel.setPixelColor(0, pixel.Color(0, 0, 255)); // Set initial color to Blue
  pixel.show();                                   // Ensure all LEDs are off

  /** IOEX INIT */
  Serial.println("IO Expander init");
  pinMode(IOEX_INT_pin, INPUT);     // Set IO Expander interrupt pin as input
  pinMode(IOEX_RST_pin, OUTPUT);    // Set IO Expander reset pin as output
  digitalWrite(IOEX_RST_pin, HIGH); // Reset IO Expander
  IOEX_Wire.setClock(I2C_CLOCK_SPEED);
  IOEX_Wire.setSDA(IOEX_SDA_pin);
  IOEX_Wire.setSCL(IOEX_SCL_pin);
  IOEX_Wire.begin(); // Initialize I2C for IO Expander

  if (IOEX.check_comm()) // Check if the IO Expander is present
  {
    Serial.println("IO Expander found");
    for (int i = 0; i < 16; i++)
    { // 0–7 LDO, 8–15 BOOT, 16–23 I2C_IO
      IOEX.setpinMode(i, OUTPUT);
    }
    for (int i = SLOT1; i <= SLOT5; i++)
    {
      Serial.printf("Setting IO Expander pins for slot %d\n", i + 1);
      IOEX.digitalWrite(IOEX_ioPins[i], HIGH);   // Set IO Expander pins low
      IOEX.digitalWrite(IOEX_bootPins[i], HIGH); // Set IO Expander boot pins high
      IOEX.digitalWrite(IOEX_pwrPins[i], LOW);   // Set IO Expander power pins low to power on slots
    }
  }
  else
  {
    Serial.println("IO Expander not found");
  }

  /*slot wire init */
  Wire.setClock(I2C_CLOCK_SPEED);
  Wire.setSDA(I2C_SDA_pin);
  Wire.setSCL(I2C_SCL_pin);
  Wire.begin();

  EEPROM.begin(EEPROM_SIZE); // EEPROM init

  EEPROM.get(set_hid, cd.hid); // get hub id from EEPROM
  Serial.printf("HUB ID = %d\n", cd.hid);

  EEPROM.get(set_num_sensor, cd.ns); // get number of sensors from EEPROM
  Serial.printf("Number of sensors = %d\n", cd.ns);

  EEPROM.get(set_sensor_type, cd.sensor_Type); // get sensor type from EEPROM
  Serial.println(cd.sensor_Type);
  if (cd.sensor_Type == 0xff)
  {
    Serial.printf("sensor type not initialised, setting default sensor type %s\n",
                  (DEFAULT_SENSOR_TYPE == SENSOR_TYPE_MFL) ? "MFL" : "EGP");
    cd.sensor_Type = DEFAULT_SENSOR_TYPE;        // set default sensor type if not set
    EEPROM.put(set_sensor_type, cd.sensor_Type); // save default sensor type to EEPROM
  }
  Serial.println(cd.sensor_Type);
  EEPROM.get(set_mag_axis, cd.mag_axis); // get magnetic axis from EEPROM
  Serial.printf("Magnetic axis = %d\n", cd.mag_axis);

  EEPROM.get(set_ang_axis, cd.ang_axis); // get angular axis from EEPROM
  Serial.printf("Angular axis = %d\n", cd.ang_axis);
  Serial.printf("Sensor type = %d (%s)\n", cd.sensor_Type,
                (cd.sensor_Type == SENSOR_TYPE_MFL) ? "MFL" : "EGP");

  /**Sensor power init default mfl */
  pinMode(SENSOR_SUPPLY_SEL0, OUTPUT);
  pinMode(SENSOR_SUPPLY_SEL1, OUTPUT);                                  // Set EGP power pin as output
  setSensorPwr((cd.sensor_Type == SENSOR_TYPE_MFL) ? pwr_3v3 : pwr_5v); // disable EGP power by default

  /*CSB pins initialisation*/
  pinMode(MODE_pin, INPUT_PULLDOWN); // Set mode pin as input with pull-down resistor
  pinMode(TRIG_pin, INPUT);
  attachInterrupt(TRIG_pin, trigger, RISING); // Enable trigger interrupt at startup

  pinMode(TX_EN_pin, OUTPUT);
  digitalWrite(TX_EN_pin, TX_EN); // enable data tx

  EconfigRX.begin(details(configRX), &CSB_Serial);
  EscRX.begin(details(sc), &CSB_Serial);
  EstatTX.begin(details(cd), &CSB_Serial);

  /* slot serial init */
  SLOT_Serial.setRX(SLOT_RX_pin);
  SLOT_Serial.setTX(SLOT_TX_pin);
  SLOT_Serial.begin(SLOT_BR); // Initialize slot Serial communication
  EscTX.begin(details(sc), &SLOT_Serial);

  /*Slot pins init*/
  pinMode(SLOT_TRIG_pin, OUTPUT);
  pinMode(SLOT_MODE_pin, OUTPUT);
  digitalWrite(SLOT_MODE_pin, OTHER_MODE);

  /*usb hub pins init*/
  pinMode(USB_RPBUS_PWR_pin, INPUT);
  pinMode(SDCARD_HUB_DET_pin, OUTPUT);
  digitalWrite(SDCARD_HUB_DET_pin, HIGH);

  /***mux init ***/
  pinMode(MUX_EN_pin, OUTPUT);
  digitalWrite(MUX_EN_pin, LOW);
  pinMode(MUX_C0_pin, OUTPUT);
  pinMode(MUX_C1_pin, OUTPUT);
  pinMode(MUX_C2_pin, OUTPUT);

  // Initialize menu system
  inMainMenu = true;
  inSubMenu = false;
  currentSubMenu = 0;
  
  // Initialize communication statistics
  comm_stats.last_comm_activity = millis();
  
  // Display welcome message and main menu
  Serial.println("\n🚀 HUB Master Controller Initialized!");
  Serial.println("📋 Menu system ready - Press Enter to display menu");
}

uint32_t p = 0, x = 0;
void loop_master()
{
  check_mode();

  if (mode_stat == ACQ_MODE && Trig_stat) // Acquisition mode and trigger pin is high
  {
    Trig_stat = 0;
    digitalWrite(SLOT_TRIG_pin, HIGH);
    if (!receiveDataWithTimeout(&EscRX, 200, 3)) { // receive sample count from CSB with 200ms timeout + 3 retries
      Serial.println("⚠️ Sample count timeout - keeping last value");
      // Don't reset sc to 0, keep previous value to avoid getting stuck at 0
    }
    // sc++;             // increment sample count
    EscTX.sendData(); // send sample count to CSB

    delayMicroseconds(200);
    digitalWrite(SLOT_TRIG_pin, LOW);
  }
  else // other mode
  {
    // Handle dummy logging sample count transmission (CLEAN - no prints)
    if (dummy_logging_active)
    {
      unsigned long current_time = micros();
      if (current_time - last_dummy_trigger_time >= dummy_trigger_interval)
      {
        // Update sample count and send to slots
        dummy_sample_count++;
        sc = dummy_sample_count; // Update the global sample count variable
        EscTX.sendData(); // Send sample count to slots
        last_dummy_trigger_time = current_time;
      }
    }
    
    if (Serial.available())
    {
      processMenuInput();
    }
  }
  if (EconfigRX.receiveData()) // receive configuration from CSB (non-critical, keep original for now)
  checkCSB(&configRX);
}

/**
 * @brief Check the mode of the system
 */
void check_mode()
{
  current_mode = digitalRead(MODE_pin);
  if (mode_stat != current_mode)
  {
    mode_stat = current_mode;
    digitalWrite(SLOT_MODE_pin, current_mode);
    if (mode_stat == ACQ_MODE) // logging mode
    {
      pixel.setPixelColor(0, pixel.Color(0, 255, 0)); // Set NeoPixel color to Green
      Serial.println("🟢 Entering ACQUISITION mode");
      Serial.println("Trigger interrupt is active, waiting for CSB triggers...");
      // Interrupt already attached at startup, no need to attach again
      // Keep Serial active for debugging
    }
    else
    {
      pixel.setPixelColor(0, pixel.Color(255, 0, 0)); // Set NeoPixel color to Red
      digitalWrite(TX_EN_pin, TX_EN);
      Serial.println("🔴 Entering OTHER mode (menu mode)");
      // Keep interrupt attached but it won't be processed in OTHER_MODE
      // Serial remains active for menu system
    }

    pixel.show(); // Update the NeoPixel color
  }
}

/**
 * @brief Selects the channel of the MUX
 * @param Channel channel to be selected
 */
void Channel_sel(slot_t Channel)
{
  // disable Mux
  digitalWrite(MUX_EN_pin, LOW);
  // select channel
  delay(100);
  digitalWrite(MUX_C0_pin, hubChannels[Channel] & 0x01);
  digitalWrite(MUX_C1_pin, hubChannels[Channel] & 0x02);
  digitalWrite(MUX_C2_pin, hubChannels[Channel] & 0x04);
  // MUX enable
  digitalWrite(MUX_EN_pin, HIGH);
}

/**
 * @brief Bootup selected HUB
 * @param HUB HUB to be booted
 */
void HUB_BOOT(slot_t HUB)
{
  IOEX.digitalWrite(IOEX_pwrPins[HUB], HIGH);  // Set power pin HIGH to power the HUB
  IOEX.digitalWrite(IOEX_bootPins[HUB], LOW);  // Set boot pin low to boot the HUB
  delay(10);                                   // Wait for 100ms to allow the HUB to boot
  IOEX.digitalWrite(IOEX_pwrPins[HUB], LOW);   // Set power pin LOW to power the HUB
  delay(10);                                   // Wait for 100ms to allow the HUB to boot
  IOEX.digitalWrite(IOEX_bootPins[HUB], HIGH); // Set boot pin high to complete the boot process
  Channel_sel(HUB);
  Serial.printf("HUB %d booted\n", HUB + 1); // Print the booted HUB number
}

/**
 * @brief Trigger pin ISR
 */
void trigger()
{
  Trig_stat = 1;
}

/**
 * @brief Check the serial command
 * @param cmd command to be executed
 */
void check_Serial_cmd(uint8_t cmd)
{
  uint8_t configSlave[2];
  switch (cmd)
  {
  case Hub1_boot:
    HUB_BOOT(SLOT1);
    break;
  case Hub2_boot:
    HUB_BOOT(SLOT2);
    break;
  case Hub3_boot:
    HUB_BOOT(SLOT3);
    break;
  case Hub4_boot:
    HUB_BOOT(SLOT4);
    break;
  case Hub5_boot:
    HUB_BOOT(SLOT5);
    break;
  case Hub1_SD_conn:
    // Serial.printf("cmd %d executing\n",cmd);
    configSlave[0] = I2C_CMD_SD_CON;
    send_I2C_cmd(SLOT1, configSlave, 1);
    break;
  case Hub2_SD_conn:
    configSlave[0] = I2C_CMD_SD_CON;
    send_I2C_cmd(SLOT2, configSlave, 1);
    break;
  case Hub3_SD_conn:
    configSlave[0] = I2C_CMD_SD_CON;
    send_I2C_cmd(SLOT3, configSlave, 1);
    break;
  case Hub4_SD_conn:
    configSlave[0] = I2C_CMD_SD_CON;
    send_I2C_cmd(SLOT4, configSlave, 1);
    break;
  case Hub5_SD_conn:
    configSlave[0] = I2C_CMD_SD_CON;
    send_I2C_cmd(SLOT5, configSlave, 1);
    break;
  case disconnect_sd:
    // Code to disconnect SD card (needs to be defined)
    digitalWrite(SDCARD_HUB_DET_pin, HIGH);
    configSlave[0] = I2C_CMD_SD_DISCON;

    // === Set all I2C IO pins HIGH before broadcast ===
    for (uint8_t i = 0; i < 5; i++)
    {
      IOEX.digitalWrite(IOEX_ioPins[i], HIGH);
    }
    wireBroadcast(configSlave, 1);

    break;
  case Hub1_sel:
    Channel_sel(SLOT1);
    break;
  case Hub2_sel:
    Channel_sel(SLOT2);
    break;
  case Hub3_sel:
    Channel_sel(SLOT3);
    break;
  case Hub4_sel:
    Channel_sel(SLOT4);
    break;
  case Hub5_sel:
    Channel_sel(SLOT5);
    break;
  case MUX_disconnect:
    digitalWrite(MUX_EN_pin, LOW);
    break;
  case config_hid:
    configSlave[0] = HID_REG;
    configSlave[1] = cd.hid;
    wireBroadcast(configSlave, 2);
    break;

  case config_sid:
    for (uint8_t i = 0; i < 5; i++)
    {
      configSlave[0] = SID_REG;
      configSlave[1] = i + 1;
      send_I2C_cmd((slot_t)i, configSlave, sizeof(configSlave));
      delay(2000);
    }
    break;

  case dummy_log_start:
    // Code to start dummy logging (needs to be defined)
    Serial.printf("Dummy logging started at %u Hz\n", dummy_log_freq);
    digitalWrite(SLOT_MODE_pin, HIGH); // Set slot mode to logging
    delay(1000);                       // Wait for 1 second
    analogWriteFreq(dummy_log_freq);   // Set PWM frequency to configured frequency
    analogWrite(SLOT_TRIG_pin, 128);   // Set 50% duty cycle
    
    // Initialize dummy logging variables (CLEAN)
    dummy_logging_active = true;
    dummy_sample_count = 0;
    dummy_trigger_interval = 1000000 / dummy_log_freq; // Convert Hz to microseconds interval
    last_dummy_trigger_time = micros();

    break;

  case dummy_log_end:
    // Code to end dummy logging (needs to be defined)
    Serial.println("Dummy logging ended");
    digitalWrite(SLOT_MODE_pin, LOW); // Set slot mode to other
    analogWrite(SLOT_TRIG_pin, 0);    // Set duty cycle to 0%
    
    // Stop dummy logging (CLEAN)
    dummy_logging_active = false;
    break;

  default:
    // Handle invalid command
    break;
  }
}

/**
 * @brief Check the received data from CSB
 * @param c pointer to the received data
 */
void checkCSB(cmd_t *c)
{
  uint8_t configSlave[2];
  // Check if the received data is valid
  if (c->BoardID == cd.hid || c->BoardID == Broadcast_ADDR)
  {
    // Process the received data
    Serial.printf("Received Board ID: %d\n", c->BoardID);
    Serial.printf("Received Command: %d\n", c->param);
    Serial.printf("Received Value: %d\n", c->value);
    switch (c->param)
    {
    case set_hid:
      if (c->value > 32 || c->value <= 0)
        c->value = 0xff; // set invalid command
      else if (c->value != cd.hid)
      {
        cd.hid = c->value;           // set hub id
        EEPROM.put(set_hid, cd.hid); // save hub id to EEPROM
        EEPROM.commit();
        check_Serial_cmd(config_hid); // send command to set hub id to the hub board
      }
      break;

    case set_num_sensor:
      cd.ns = c->value;                  // set number of sensors
      EEPROM.put(set_num_sensor, cd.ns); // save number of sensors to EEPROM
      EEPROM.commit();
      // ldo_mask = 0xFF;
      // usb_mask = 0xFF;
      // tcal9539_update();
      configSlave[0] = set_num_sensor;
      configSlave[1] = cd.ns;
      wireBroadcast(configSlave, 2); // send number of sensors to the hub board
      break;

    case set_sensor_type:
      cd.sensor_Type = c->value;                   // set sensor type
      EEPROM.put(set_sensor_type, cd.sensor_Type); // save sensor type to EEPROM
      EEPROM.commit();
      // ldo_mask = 0xFF;
      // usb_mask = 0xFF;
      // tcal9539_update();
      configSlave[0] = set_sensor_type;
      configSlave[1] = cd.sensor_Type;
      wireBroadcast(configSlave, 2); // send sensor type to the hub board
      break;

    case set_mag_axis:
      cd.mag_axis = c->value;                // set magnetic axis
      EEPROM.put(set_mag_axis, cd.mag_axis); // save magnetic axis to EEPROM
      EEPROM.commit();
      // ldo_mask = 0xFF;
      // usb_mask = 0xFF;
      // tcal9539_update();
      configSlave[0] = set_mag_axis;
      configSlave[1] = cd.mag_axis;
      wireBroadcast(configSlave, 2); // send magnetic axis to the hub board
      break;

    case set_ang_axis:
      cd.ang_axis = c->value;                // set angular axis
      EEPROM.put(set_ang_axis, cd.ang_axis); // save angular axis to EEPROM
      EEPROM.commit();
      // ldo_mask = 0xFF;
      // usb_mask = 0xFF;
      // tcal9539_update();
      configSlave[0] = set_ang_axis;
      configSlave[1] = cd.ang_axis;
      wireBroadcast(configSlave, 2); // send angular axis to the hub board
      break;

    case cmd_hub_boot:
      if (c->value > 0 && c->value < 8)
      {
        HUB_BOOT((slot_t)(c->value - 1)); // bootup hub board
      }
      else
      {
        c->value = 0xff; // set invalid command
        Serial.printf("Received invalid command: %d\n", c->param);
      }
      break;

    case cmd_hub_sel:
      if (c->value > 0 && c->value < 8)
      {
        Channel_sel((slot_t)(c->value - 1)); // select hub board
      }
      else
      {
        c->value = 0xff; // set invalid command
        Serial.printf("Received invalid command: %d\n", c->param);
      }
      break;
    case cmd_pwr_en:
      if (c->value > 0 && c->value <= 8)
      {
        // io_expander_set_ldo(c->value - 1, true); // enable power to the hub board
      }
      else
      {
        c->value = 0xff; // set invalid command
        Serial.printf("Received invalid command for power enable: %d\n", c->value);
      }
      break;

    case cmd_get_data:
    {
      EasyTransfer dtx;
      uint8_t *data = nullptr;
      if(cd.sensor_Type == 0)
        data = new uint8_t [44];
      else
        data = new uint8_t [14*6];
      dtx.begin(details(data), &CSB_Serial); // begin EasyTransfer object to send data to CSB
      dtx.sendData();                        // send data to CSB
      // Optionally free memory if needed:
      delete[] data;
      break;
    }

    case cmd_get_config:
      EstatTX.sendData(); // send configuration data to CSB
      break;

    default:
      c->param = 0xff; // set invalid command
      Serial.printf("Received unknown command: %d\n", c->param);
      break;
    }
  }
  else
  {
    // Process the received data
    c->BoardID = 0xff; // set invalid board id
    Serial.printf("Received Board ID is invalid: %d\n", c->BoardID);
  }
  EconfigRX.sendData(); // send data back to CSB
}

/**
 * @brief Send I2C command to the slaves
 * @param i2c_io I2C IO to be selected
 * @param cmd command to be sent
 * @param n number of bytes to be sent
 */
void send_I2C_cmd(slot_t slot, const uint8_t *cmd, uint8_t n)
{
  if (*cmd == I2C_CMD_SD_CON)
  {
    // === Set all I2C IO pins HIGH before broadcast ===
    for (uint8_t i = SLOT1; i <= SLOT5; i++)
    {
      IOEX.digitalWrite(IOEX_ioPins[i], i == slot ? LOW : HIGH); // Set all I2C IO pins LOW except the selected one
    }

    digitalWrite(SDCARD_HUB_DET_pin, HIGH);
    uint8_t c = I2C_CMD_SD_DISCON;
    wireBroadcast(&c, 1);
    delay(2000);
  }
  for (uint8_t i = SLOT1; i <= SLOT5; i++)
  {
    IOEX.digitalWrite(IOEX_ioPins[i], i == slot ? HIGH : LOW); // Set all I2C IO pins LOW except the selected one
  }
  delay(1);              // Allow IO to settle
  wireBroadcast(cmd, n); // Send actual I2C command
  if (*cmd == I2C_CMD_SD_CON)
  {
    delay(1000); // Wait for 1 second to allow SD card to connect
    digitalWrite(SDCARD_HUB_DET_pin, LOW);
  }
}

/**
 * @brief Broadcast bytes to the slaves
 * @param data data to be sent
 * @param n number of bytes to be sent
 * @return status of endTransmission
 */
uint8_t wireBroadcast(const uint8_t *data, uint8_t n)
{
  Serial.printf("Broadcasting %d bytes\n", n);
  Wire.beginTransmission(0x00);
  for (size_t i = 0; i < n; i++)
  {
    Wire.write(*data);
    data++;
  }

  uint8_t result = Wire.endTransmission();
  comm_stats.last_comm_activity = millis();
  
  if (result == 0) {
    comm_stats.i2c_success_count++;
    Serial.println("✅ I2C Broadcast successful");
  } else {
    comm_stats.i2c_error_count++;
    Serial.printf("❌ I2C Broadcast failed with error code: %d\n", result);
    Serial.println("   Error codes: 1=Buffer full, 2=NACK on address, 3=NACK on data, 4=Other");
  }
  
  return result;
}

/**
 * @brief Enhanced EasyTransfer receive with timeout and statistics
 * @param et EasyTransfer object pointer
 * @param timeout_ms Timeout in milliseconds (default 100ms)
 * @param retries Number of retry attempts (default 1)
 * @return true if data received successfully, false on timeout/failure
 */
bool receiveDataWithTimeout(EasyTransfer* et, uint32_t timeout_ms = 100, uint8_t retries = 1)
{
  for (uint8_t attempt = 0; attempt <= retries; attempt++) {
    uint32_t start_time = millis();
    
    while (millis() - start_time < timeout_ms) {
      if (et->receiveData()) {
        comm_stats.easytransfer_success_count++;
        comm_stats.last_comm_activity = millis();
        if (attempt > 0) {
          Serial.printf("📡 EasyTransfer success on retry %d\n", attempt);
        }
        return true;  // Success
      }
      delayMicroseconds(100);  // Brief yield to prevent busy waiting
    }
    
    if (attempt < retries) {
      Serial.printf("⏰ EasyTransfer timeout on attempt %d, retrying...\n", attempt + 1);
      delay(5);  // Brief delay before retry
    }
  }
  
  comm_stats.easytransfer_timeout_count++;
  Serial.printf("❌ EasyTransfer failed after %d attempts\n", retries + 1);
  return false;  // All attempts failed
}

/**
 * @brief ISR for wire receive event
 * @param howMany number of bytes received
 */
void receiveEvent(int howMany)
{
  // Serial.print("Received data: ");
  static uint16_t prev_val;
  uint8_t c[howMany];
  digitalWrite(LED_pin, HIGH);
  if (Wire.available() > 0)
  {
    Wire.readBytes(c, howMany); // Read each byte from the I2C buffer
                                // Print the received byte for debugging
  }
  digitalWrite(LED_pin, LOW);
  uint16_t val = ((uint16_t)c[0] << 8) | c[1];
  Serial.println(val);
  if (val - prev_val != 1)
  {
    Serial.print("data loss: ");
    Serial.println(val - prev_val);
  }
  prev_val = val;
}

/**
 * @brief ISR for wire request event
 */
void requestEvent()
{
  Serial.println("Request received, sending data...");
  // Wire.write(0x31);  // Send some data back to the master
}

/**
 * @brief Generic function to query a slot and get response
 * @param slot Target slot (SLOT1-SLOT5)
 * @param query_cmd Query command (I2C_CMD_GET_STATUS, etc.)
 * @param response Buffer to store response data
 * @param response_len Expected response length
 * @param timeout_ms Timeout in milliseconds
 * @return true if successful, false if failed
 */
bool querySlot(slot_t slot, uint8_t query_cmd, uint8_t* response, uint8_t response_len, uint32_t timeout_ms)
{
  if (slot < SLOT1 || slot > SLOT5 || !response || response_len == 0) {
    Serial.println("❌ Invalid query parameters");
    return false;
  }
  
  // Send query command to specific slot
  Serial.printf("🔍 Querying slot %d with command 0x%02X\n", slot + 1, query_cmd);
  send_I2C_cmd(slot, &query_cmd, 1); // This will handle addressing and call wireBroadcast
  
  // Request response from slot
  delay(10); // Brief delay to let slot process the query
  
  // Select the specific slot for I2C communication
  for (uint8_t i = SLOT1; i <= SLOT5; i++) {
    IOEX.digitalWrite(IOEX_ioPins[i], i == slot ? HIGH : LOW);
  }
  delay(1); // Allow IO to settle
  
  // Request data from the selected slot using dynamic addressing
  uint8_t slot_i2c_addr = slot + 1; // Use slot position as I2C address (1-5)
  uint8_t bytes_requested = Wire.requestFrom(slot_i2c_addr, response_len);
  Serial.printf("Requesting from I2C address 0x%02X (slot %d)\n", slot_i2c_addr, slot + 1);
  
  // Check if requestFrom succeeded (returned the expected number of bytes)
  if (bytes_requested != response_len) {
    Serial.printf("❌ I2C requestFrom failed: requested %d, got %d from slot %d\n", 
                 response_len, bytes_requested, slot + 1);
    comm_stats.i2c_error_count++;
    return false;
  }
  
  uint32_t start_time = millis();
  uint8_t bytes_received = 0;
  
  // Clear response buffer first
  memset(response, 0, response_len);
  
  // Read available data with timeout
  while (millis() - start_time < timeout_ms && bytes_received < response_len) {
    if (Wire.available()) {
      response[bytes_received] = Wire.read();
      bytes_received++;
    } else {
      delayMicroseconds(100);
    }
  }
  
  // Validate we received the expected number of bytes
  if (bytes_received != response_len) {
    Serial.printf("❌ Timeout: Got %d/%d bytes from slot %d\n", bytes_received, response_len, slot + 1);
    comm_stats.i2c_error_count++;
    return false;
  }
  
  // Validate the slot ID in response (allow reasonable slot IDs)
  if (response[0] == 0 || response[0] > 5) { // Basic sanity check - should be 1-5
    Serial.printf("❌ Invalid slot ID in response: got %d from slot %d (expected 1-5)\n", 
                 response[0], slot + 1);
    comm_stats.i2c_error_count++;
    return false;
  }
  
  Serial.printf("✅ Received valid %d bytes from slot %d [ID:%d]\n", 
               bytes_received, slot + 1, response[0]);
  comm_stats.i2c_success_count++;
  comm_stats.last_comm_activity = millis();
  return true;
}

/**
 * @brief Get slot status and health information
 * @param slot Target slot
 * @param status_data Buffer to store 4 bytes: [slot_id, status, error_flags, activity]
 * @return true if successful
 */
bool getSlotStatus(slot_t slot, uint8_t* status_data)
{
  return querySlot(slot, I2C_CMD_GET_STATUS, status_data, 4, 500);
}

/**
 * @brief Get sensor readings from slot
 * @param slot Target slot  
 * @param sensor_value Pointer to store 16-bit sensor value
 * @return true if successful
 */
bool getSlotSensors(slot_t slot, uint16_t* sensor_value)
{
  uint8_t response[4];
  if (querySlot(slot, I2C_CMD_GET_SENSORS, response, 4, 500)) {
    if (response[3] == 0xAA) { // Verify data valid marker
      *sensor_value = (response[1] << 8) | response[2];
      return true;
    }
  }
  *sensor_value = 0;
  return false;
}

/**
 * @brief Get slot configuration
 * @param slot Target slot
 * @param config_data Buffer to store 4 bytes: [slot_id, sensor_type, num_sensors, mag_axis]
 * @return true if successful  
 */
bool getSlotConfig(slot_t slot, uint8_t* config_data)
{
  return querySlot(slot, I2C_CMD_GET_CONFIG, config_data, 4, 500);
}

/**
 * @brief Ping slot for connectivity test
 * @param slot Target slot
 * @return true if slot responds to ping
 */
bool pingSlot(slot_t slot)
{
  uint8_t response[4];
  if (querySlot(slot, I2C_CMD_PING, response, 4, 300)) {
    // Verify ping response markers are exactly what we expect
    if (response[1] == 0xAA && response[2] == 0x55 && response[3] == I2C_CMD_PING) {
      Serial.printf("🏓 Slot %d PING: Valid markers [0xAA, 0x55, 0x%02X]\n", 
                   slot + 1, response[3]);
      return true;
    } else {
      Serial.printf("❌ Slot %d PING: Invalid markers [0x%02X, 0x%02X, 0x%02X]\n", 
                   slot + 1, response[1], response[2], response[3]);
    }
  }
  return false;
}

/**
 * @brief Set sensor power based on selection
 * @param sel Selection for sensor power
 */
void setSensorPwr(sensorpwrsel sel)
{
  switch (sel)
  {
  case pwr_3v3:
    digitalWrite(SENSOR_SUPPLY_SEL0, LOW);
    digitalWrite(SENSOR_SUPPLY_SEL1, LOW);
    break;

  case pwr_5v:
    digitalWrite(SENSOR_SUPPLY_SEL0, LOW);
    digitalWrite(SENSOR_SUPPLY_SEL1, HIGH); // Enable EGP power
    break;

  case pwr_off:
    digitalWrite(SENSOR_SUPPLY_SEL0, HIGH); // Disable sensor power
    digitalWrite(SENSOR_SUPPLY_SEL1, HIGH); // Disable EGP power
    break;
  default:
    // power off
    Serial.println("Wrong selection, powering off sensors");
    digitalWrite(SENSOR_SUPPLY_SEL0, HIGH); // Disable sensor power
    digitalWrite(SENSOR_SUPPLY_SEL1, HIGH); // Disable EGP power
    break;
  }

  /**
   * SEL0 SEL1 OUTPUT
   * 0    0    3.3V
   * 0    1    5V
   * 1    1    Hi-Z (power off)
   */
}

/**
 * @brief Display main menu with beautiful formatting
 */
void displayMainMenu()
{
  Serial.println();
  Serial.println("╔══════════════════════════════════════════════════════════╗");
  Serial.println("║                    HUB MASTER CONTROL                   ║");
  Serial.println("╠══════════════════════════════════════════════════════════╣");
  Serial.println("║                                                          ║");
  Serial.println("║  1. Slot Management                                      ║");
  Serial.println("║  2. SD Card Management                                   ║");
  Serial.println("║  3. Configuration                                        ║");
  Serial.println("║  4. System Info                                          ║");
  Serial.println("║  5. Test Mode                                            ║");
  Serial.println("║                                                          ║");
  Serial.println("║  0. Exit                                                 ║");
  Serial.println("║                                                          ║");
  Serial.println("╚══════════════════════════════════════════════════════════╝");
  Serial.print("Select option (0-5): ");
}

/**
 * @brief Display sub menu based on main selection
 */
void displaySubMenu()
{
  Serial.println();
  switch(currentSubMenu)
  {
    case 1: // Slot Management
      Serial.println("╔══════════════════════════════════════════════════════════╗");
      Serial.println("║                   SLOT MANAGEMENT                       ║");
      Serial.println("╠══════════════════════════════════════════════════════════╣");
      displaySlotStatus();
      Serial.println("║                                                          ║");
      Serial.println("║  Boot Slots:        │  Select Slots:    │ Power Control: ║");
      Serial.println("║  1. Boot Slot 1     │  6. Select Slot 1 │ 12. Power ON 1 ║");
      Serial.println("║  2. Boot Slot 2     │  7. Select Slot 2 │ 13. Power ON 2 ║");
      Serial.println("║  3. Boot Slot 3     │  8. Select Slot 3 │ 14. Power ON 3 ║");
      Serial.println("║  4. Boot Slot 4     │  9. Select Slot 4 │ 15. Power ON 4 ║");
      Serial.println("║  5. Boot Slot 5     │ 10. Select Slot 5 │ 16. Power ON 5 ║");
      Serial.println("║                     │ 11. Disconnect    │ 17. Power OFF1 ║");
      Serial.println("║  22. Refresh Status │     MUX           │ 18. Power OFF2 ║");
      Serial.println("║                     │                   │ 19. Power OFF3 ║");
      Serial.println("║                     │                   │ 20. Power OFF4 ║");
      Serial.println("║                     │                   │ 21. Power OFF5 ║");
      Serial.println("║                                                          ║");
      Serial.println("║  Sensor Supply Voltage:                                 ║");
      Serial.println("║  23. Set 3.3V       │ 24. Set 5V        │ 25. Power OFF  ║");
      Serial.println("║                                                          ║");
      Serial.println("║  0. Back to main menu                                   ║");
      Serial.println("╚══════════════════════════════════════════════════════════╝");
      break;
      
    case 2: // SD Card Management  
      Serial.println("╔══════════════════════════════════════════════════════════╗");
      Serial.println("║                  SD CARD MANAGEMENT                     ║");
      Serial.println("╠══════════════════════════════════════════════════════════╣");
      Serial.println("║                                                          ║");
      Serial.println("║  Connect SD Cards:                                      ║");
      Serial.println("║  1. Connect Slot 1 SD    │  4. Connect Slot 4 SD        ║");
      Serial.println("║  2. Connect Slot 2 SD    │  5. Connect Slot 5 SD        ║");
      Serial.println("║  3. Connect Slot 3 SD    │  6. Disconnect All SD        ║");
      Serial.println("║                                                          ║");
      Serial.println("║  0. Back to main menu                                   ║");
      Serial.println("║                                                          ║");
      Serial.println("╚══════════════════════════════════════════════════════════╝");
      break;
      
    case 3: // Configuration
      Serial.println("╔══════════════════════════════════════════════════════════╗");
      Serial.println("║                    CONFIGURATION                        ║");
      Serial.println("╠══════════════════════════════════════════════════════════╣");
      Serial.println("║                                                          ║");
      {
        char configLine1[59], configLine2[59], configLine3[59];
        snprintf(configLine1, sizeof(configLine1), "║  Current HID: %-3d                                        ║", cd.hid);
        snprintf(configLine2, sizeof(configLine2), "║  Default Sensors: %-2d │  System Type: %-3s              ║", 
                 cd.ns, (cd.sensor_Type == SENSOR_TYPE_MFL) ? "MFL" : "EGP");
        snprintf(configLine3, sizeof(configLine3), "║  Mag Axis: %-2d     │  Ang Axis: %-2d                     ║", 
                 cd.mag_axis, cd.ang_axis);
        Serial.println(configLine1);
        Serial.println(configLine2);
        Serial.println(configLine3);
      }
      Serial.println("║                                                          ║");
      Serial.println("║  1. Configure HID       │  4. Configure Ang Axis        ║");
      Serial.println("║  2. Configure SID       │  5. Configure Sensor Type     ║");
      Serial.println("║  3. Configure Mag Axis  │  6. EGP Sensor Heads Config   ║");
      Serial.println("║                         │                               ║");
      Serial.println("║                                                          ║");
      Serial.println("║  0. Back to main menu                                   ║");
      Serial.println("║                                                          ║");
      Serial.println("╚══════════════════════════════════════════════════════════╝");
      break;
      
    case 4: // System Info
      Serial.println("╔══════════════════════════════════════════════════════════╗");
      Serial.println("║                     SYSTEM INFO                         ║");
      Serial.println("╠══════════════════════════════════════════════════════════╣");
      Serial.println("║                                                          ║");
      {
        char infoLines[12][80];  // Expanded array for communication stats with larger buffer
        
        // System Configuration
        snprintf(infoLines[0], sizeof(infoLines[0]), "║  Hub ID: %-3d                                             ║", cd.hid);
        snprintf(infoLines[1], sizeof(infoLines[1]), "║  Number of Sensors: %-3d                                  ║", cd.ns);
        snprintf(infoLines[2], sizeof(infoLines[2]), "║  Sensor Type: %-43s ║", 
                 (cd.sensor_Type == SENSOR_TYPE_MFL) ? "MFL (Magnetic Flux Leakage)" : "EGP (Eddy Current)");
        snprintf(infoLines[3], sizeof(infoLines[3]), "║  Magnetic Axis: %-2d                                       ║", cd.mag_axis);
        snprintf(infoLines[4], sizeof(infoLines[4]), "║  Angular Axis: %-2d                                        ║", cd.ang_axis);
        snprintf(infoLines[5], sizeof(infoLines[5]), "║  Current Mode: %-43s ║", 
                 mode_stat == ACQ_MODE ? "ACQUISITION" : "OTHER");
        snprintf(infoLines[6], sizeof(infoLines[6]), "║  Sample Count: %-43llu ║", sc);
        
        // Communication Statistics
        strcpy(infoLines[7], "║                                                          ║");
        strcpy(infoLines[8], "║  [COMM] COMMUNICATION STATISTICS:                       ║");
        
        // I2C Statistics
        float i2c_success_rate = 0;
        uint32_t total_i2c = comm_stats.i2c_success_count + comm_stats.i2c_error_count;
        if (total_i2c > 0) {
          i2c_success_rate = (float)comm_stats.i2c_success_count / total_i2c * 100.0;
        }
        snprintf(infoLines[9], sizeof(infoLines[9]), "║  I2C: %lu OK / %lu ERR (%.1f%% success)                  ║", 
                 comm_stats.i2c_success_count, comm_stats.i2c_error_count, i2c_success_rate);
        
        // EasyTransfer Statistics  
        float et_success_rate = 0;
        uint32_t total_et = comm_stats.easytransfer_success_count + comm_stats.easytransfer_timeout_count;
        if (total_et > 0) {
          et_success_rate = (float)comm_stats.easytransfer_success_count / total_et * 100.0;
        }
        snprintf(infoLines[10], sizeof(infoLines[10]), "║  EasyTransfer: %lu OK / %lu TO (%.1f%% success)          ║", 
                 comm_stats.easytransfer_success_count, comm_stats.easytransfer_timeout_count, et_success_rate);
        
        // Last Activity
        unsigned long time_since_activity = (millis() - comm_stats.last_comm_activity) / 1000;
        snprintf(infoLines[11], sizeof(infoLines[11]), "║  Last Communication: %lu seconds ago                     ║", time_since_activity);
        
        for(int i = 0; i < 12; i++) {
          Serial.println(infoLines[i]);
        }
      }
      Serial.println("║                                                          ║");
      Serial.println("║  Press any key to return to main menu                   ║");
      Serial.println("║                                                          ║");
      Serial.println("╚══════════════════════════════════════════════════════════╝");
      break;
      
    case 5: // Test Mode
      Serial.println("╔══════════════════════════════════════════════════════════╗");
      Serial.println("║                      TEST MODE                          ║");
      Serial.println("╠══════════════════════════════════════════════════════════╣");
      Serial.printf("║  Current Logging Frequency: %-6u Hz                      ║", dummy_log_freq);
      Serial.printf("║  Dummy Logging Status: %-8s  Sample Count: %-10llu ║", 
                    dummy_logging_active ? "ACTIVE" : "STOPPED", dummy_sample_count);
      Serial.println("║                                                          ║");
      Serial.println("║  DUMMY LOGGING TESTS:                                   ║");
      Serial.println("║  1. Start Dummy Logging                                 ║");
      Serial.println("║  2. Stop Dummy Logging                                  ║");
      Serial.println("║  3. Test All Slots                                      ║");
      Serial.println("║  4. Set Logging Frequency                               ║");
      Serial.println("║  5. Reset Sample Counter                                ║");
      Serial.println("║                                                          ║");
      Serial.println("║                                                          ║");
      Serial.println("║  0. Back to main menu                                   ║");
      Serial.println("║                                                          ║");
      Serial.println("╚══════════════════════════════════════════════════════════╝");
      break;
  }
  Serial.print("Select option: ");
}

/**
 * @brief Process menu input with validation
 */
void processMenuInput()
{
  String input = Serial.readStringUntil('\n');
  input.trim();
  
  if (input.length() == 0) {
    if (inMainMenu) displayMainMenu();
    else displaySubMenu();
    return;
  }
  
  int cmd = input.toInt();
  
  // Check if input is actually a number
  if (cmd == 0 && input != "0") {
    Serial.println("\n❌ ERROR: Invalid input! Please enter a number.");
    if (inMainMenu) displayMainMenu();
    else displaySubMenu();
    return;
  }
  
  if (inMainMenu) {
    if (!isValidInput(cmd, 0, 5)) {
      Serial.println("\n❌ ERROR: Invalid option! Please select 0-5.");
      displayMainMenu();
      return;
    }
    
    if (cmd == 0) {
      Serial.println("\n🔄 Exiting menu system...");
      return;
    }
    
    inMainMenu = false;
    inSubMenu = true;
    currentSubMenu = cmd;
    displaySubMenu();
  }
  else if (inSubMenu) {
    processSubMenuInput(cmd);
  }
}

/**
 * @brief Process submenu input
 */
void processSubMenuInput(int cmd)
{
  if (cmd == 0) {
    inMainMenu = true;
    inSubMenu = false;
    currentSubMenu = 0;
    displayMainMenu();
    return;
  }
  
  switch(currentSubMenu) {
    case 1: // Slot Management
      if (isValidInput(cmd, 1, 25)) {
        processSlotCommand(cmd);
      } else {
        Serial.println("\n❌ ERROR: Invalid option! Please select 0-25.");
        displaySubMenu();
      }
      break;
      
    case 2: // SD Card Management
      if (isValidInput(cmd, 1, 6)) {
        processSDCommand(cmd);
      } else {
        Serial.println("\n❌ ERROR: Invalid option! Please select 0-6.");
        displaySubMenu();
      }
      break;
      
    case 3: // Configuration
      if (isValidInput(cmd, 1, 6)) {
        processConfigCommand(cmd);
      } else {
        Serial.println("\n❌ ERROR: Invalid option! Please select 0-6.");
        displaySubMenu();
      }
      break;
      
    case 4: // System Info
      inMainMenu = true;
      inSubMenu = false;
      currentSubMenu = 0;
      displayMainMenu();
      break;
      
    case 5: // Test Mode
      if (isValidInput(cmd, 1, 5)) {
        processTestCommand(cmd);
      } else {
        Serial.println("\n❌ ERROR: Invalid option! Please select 0-5.");
        displaySubMenu();
      }
      break;
  }
}

/**
 * @brief Process slot management commands
 */
void processSlotCommand(int cmd)
{
  Serial.println();
  switch(cmd) {
    case 1: case 2: case 3: case 4: case 5:
      Serial.printf("🚀 Booting Slot %d...\n", cmd);
      check_Serial_cmd(Hub1_boot + (cmd - 1));
      break;
    case 6: case 7: case 8: case 9: case 10:
      Serial.printf("🎯 Selecting Slot %d...\n", cmd - 5);
      check_Serial_cmd(Hub1_sel + (cmd - 6));
      break;
    case 11:
      Serial.println("🔌 Disconnecting MUX...");
      check_Serial_cmd(MUX_disconnect);
      break;
    case 12: case 13: case 14: case 15: case 16:
      powerOnSlot((slot_t)(cmd - 12));
      break;
    case 17: case 18: case 19: case 20: case 21:
      powerOffSlot((slot_t)(cmd - 17));
      break;
    case 22:
      Serial.println("🔄 Refreshing slot status...");
      break;
      
    case 23:
      Serial.println("⚡ Setting sensor supply voltage to 3.3V...");
      setSensorPwr(pwr_3v3);
      Serial.println("✅ Sensor supply voltage set to 3.3V");
      break;
      
    case 24:
      Serial.println("⚡ Setting sensor supply voltage to 5V...");
      setSensorPwr(pwr_5v);
      Serial.println("✅ Sensor supply voltage set to 5V");
      break;
      
    case 25:
      Serial.println("🔌 Turning OFF sensor supply voltage...");
      setSensorPwr(pwr_off);
      Serial.println("✅ Sensor supply voltage turned OFF");
      break;
  }
  Serial.println("✅ Command executed successfully!\n");
  displaySubMenu();
}

/**
 * @brief Process SD card management commands
 */
void processSDCommand(int cmd)
{
  Serial.println();
  switch(cmd) {
    case 1: case 2: case 3: case 4: case 5:
      Serial.printf("💾 Connecting Slot %d SD Card...\n", cmd);
      check_Serial_cmd(Hub1_SD_conn + (cmd - 1));
      break;
    case 6:
      Serial.println("🔄 Disconnecting all SD Cards...");
      check_Serial_cmd(disconnect_sd);
      break;
  }
  Serial.println("✅ Command executed successfully!\n");
  displaySubMenu();
}

/**
 * @brief Process configuration commands
 */
void processConfigCommand(int cmd)
{
  Serial.println();
  switch(cmd) {
    case 1:
      {
        Serial.println("🔧 Configure HID (Hub ID)");
        Serial.printf("Current HID: %d\n", cd.hid);
        Serial.print("Enter new HID (1-32): ");
        
        // Wait for user input
        while(!Serial.available()) {
          delay(10);
        }
        
        String input = Serial.readStringUntil('\n');
        input.trim();
        int newHID = input.toInt();
        
        if(newHID >= 1 && newHID <= 32) {
          cd.hid = newHID;
          EEPROM.put(set_hid, cd.hid);
          EEPROM.commit();
          Serial.printf("✅ HID set to: %d\n", cd.hid);
          
          // Broadcast new HID to all slots
          check_Serial_cmd(config_hid);
          Serial.println("📡 HID broadcasted to all slots");
        } else {
          Serial.println("❌ Invalid HID! Must be between 1-32");
        }
      }
      break;
      
    case 2:
      {
        Serial.println("🔧 Configure SID (Slot IDs) - Auto Assignment");
        Serial.println("This will assign SID 1-5 to all connected slots...");
        Serial.print("Continue? (y/n): ");
        
        while(!Serial.available()) {
          delay(10);
        }
        
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toLowerCase();
        
        if(input == "y" || input == "yes") {
          Serial.println("📡 Assigning SIDs to slots...");
          check_Serial_cmd(config_sid);
          Serial.println("✅ SID configuration completed!");
          Serial.println("   Slot 1 → SID 1");
          Serial.println("   Slot 2 → SID 2");
          Serial.println("   Slot 3 → SID 3");
          Serial.println("   Slot 4 → SID 4");
          Serial.println("   Slot 5 → SID 5");
        } else {
          Serial.println("❌ SID configuration cancelled");
        }
      }
      break;
      
    case 3:
      Serial.println("🧭 Magnetic Axis configuration not yet implemented.");
      break;
    case 4:
      Serial.println("📐 Angular Axis configuration not yet implemented.");
      break;
      
    case 5:
      {
        Serial.println("🔧 Configure Sensor Type");
        Serial.printf("Current Sensor Type: %s\n", (cd.sensor_Type == SENSOR_TYPE_MFL) ? "MFL" : "EGP");
        Serial.println("Select new sensor type:");
        Serial.println("  0 - MFL (Magnetic Flux Leakage) - 3.3V");
        Serial.println("  1 - EGP (Eddy Gap Probe) - 5V");
        Serial.print("Enter choice (0-1): ");
        
        // Wait for user input
        while(!Serial.available()) {
          delay(10);
        }
        String input = Serial.readStringUntil('\n');
        input.trim();
        int newType = input.toInt();
        
        if (newType == 0 || newType == 1) {
          cd.sensor_Type = newType;
          EEPROM.put(set_sensor_type, cd.sensor_Type);
          if (EEPROM.commit()) {
            // Update power supply based on sensor type
            setSensorPwr((cd.sensor_Type == SENSOR_TYPE_MFL) ? pwr_3v3 : pwr_5v);
            Serial.printf("✅ Sensor type updated to %s\n", (cd.sensor_Type == SENSOR_TYPE_MFL) ? "MFL (3.3V)" : "EGP (5V)");
            Serial.println("   Power supply automatically adjusted");
            
            // Broadcast to slots
            uint8_t configSlave[2];
            configSlave[0] = set_sensor_type;
            configSlave[1] = cd.sensor_Type;
            wireBroadcast(configSlave, 2);
            Serial.println("   Configuration sent to all slots");
          } else {
            Serial.println("❌ Failed to save sensor type to EEPROM");
          }
        } else {
          Serial.println("❌ Invalid selection! Please enter 0 or 1");
        }
      }
      break;
      
    case 6: // EGP Sensor Heads Configuration
      {
        Serial.println("🔧 EGP Sensor Heads Configuration");
        Serial.println("══════════════════════════════════════════════════════════");
        
        // Check if system is in EGP mode
        if(cd.sensor_Type != SENSOR_TYPE_EGP) {
          Serial.println("❌ System is currently configured for MFL sensors.");
          Serial.println("   Change sensor type to EGP first to configure sensor heads.");
          Serial.println("   (Use option 5: Configure Sensor Type)");
          break;
        }
        
        Serial.println("Configure number of ACTIVE sensor heads per EGP slot:");
        Serial.println("(Sample size remains fixed at 112 bytes - unused sensors are padded)");
        Serial.printf("Current system sensor type: %s\n", 
                     (cd.sensor_Type == SENSOR_TYPE_MFL) ? "MFL" : "EGP");
        Serial.printf("Current default sensor heads: %d\n", cd.ns);
        Serial.println();
        
        // TODO: Display current per-slot sensor head config from EEPROM/memory
        Serial.println("Current Slot Configuration (Placeholder):");
        Serial.println("  Slot 1: EGP, 8 active sensors (112 bytes total)");
        Serial.println("  Slot 2: EGP, 6 active sensors (112 bytes total, 2 padded)");  
        Serial.println("  Slot 3: EGP, 4 active sensors (112 bytes total, 4 padded)");
        Serial.println("  Slot 4: EGP, 2 active sensors (112 bytes total, 6 padded)");
        Serial.println("  Slot 5: EGP, 8 active sensors (112 bytes total)");
        Serial.println();
        
        Serial.println("Select slot to configure sensor heads:");
        Serial.println("1. Configure Slot 1 sensor heads");
        Serial.println("2. Configure Slot 2 sensor heads"); 
        Serial.println("3. Configure Slot 3 sensor heads");
        Serial.println("4. Configure Slot 4 sensor heads");
        Serial.println("5. Configure Slot 5 sensor heads");
        Serial.println("6. Set all slots to default count");
        Serial.println("0. Back to Configuration menu");
        Serial.print("Select option (0-6): ");
        
        while(!Serial.available()) { delay(10); }
        String input = Serial.readStringUntil('\n');
        input.trim();
        int slotChoice = input.toInt();
        
        if(slotChoice >= 1 && slotChoice <= 5) {
          Serial.printf("\n🎛️  Configuring Slot %d Sensor Heads\n", slotChoice);
          Serial.println("═══════════════════════════════════════");
          Serial.printf("Current ACTIVE sensor heads for Slot %d: [Placeholder: 8]\n", slotChoice);
          Serial.print("Enter number of ACTIVE sensor heads (1-8): ");
          
          while(!Serial.available()) { delay(10); }
          input = Serial.readStringUntil('\n');
          input.trim();
          int sensorCount = input.toInt();
          
          if(sensorCount >= 1 && sensorCount <= 8) {
            int activeSensorData = sensorCount * 14;
            int paddedSensorData = (8 - sensorCount) * 14;
            int totalDataSize = 8 * 14; // Always 112 bytes for 8 sensor positions
            int totalSampleSize = 112; // SAMPLE_BUFFER_SIZE for EGP is fixed at 112 bytes
            
            Serial.printf("✅ Slot %d configured:\n", slotChoice);
            Serial.printf("   Active sensor heads: %d\n", sensorCount);
            Serial.printf("   Padded sensor positions: %d\n", 8 - sensorCount);
            Serial.printf("   Active sensor data: %d bytes\n", activeSensorData);
            Serial.printf("   Padded sensor data: %d bytes (filled with known values)\n", paddedSensorData);
            Serial.printf("   Total sample size: %d bytes (FIXED)\n", totalSampleSize);
            
            // TODO: Save per-slot sensor count to EEPROM
            // TODO: Send I2C command to specific slot to update its sensor count
            // TODO: Configure slot to fill unused sensor positions with known padding values
            Serial.println("   [TODO] Saving to slot-specific EEPROM...");
            Serial.println("   [TODO] Broadcasting to slot via I2C...");
            Serial.println("   [TODO] Configuring padding values for unused sensors...");
            
          } else {
            Serial.println("❌ Invalid sensor count! Must be 1-8 active sensors");
          }
          
        } else if(slotChoice == 6) {
          Serial.printf("\nSetting all EGP slots to default sensor count: %d\n", cd.ns);
          Serial.print("Continue? (y/n): ");
          while(!Serial.available()) { delay(10); }
          input = Serial.readStringUntil('\n');
          input.trim();
          input.toLowerCase();
          
          if(input == "y" || input == "yes") {
            Serial.printf("✅ All slots set to %d sensor heads\n", cd.ns);
            // TODO: Set all slots to default sensor count
            Serial.println("   [TODO] Updating all slot configurations...");
          } else {
            Serial.println("❌ Operation cancelled");
          }
          
        } else if(slotChoice != 0) {
          Serial.println("❌ Invalid selection!");
        }
      }
      break;
  }
  Serial.println("\nPress any key to continue...");
  while(!Serial.available()) {
    delay(10);
  }
  Serial.readStringUntil('\n'); // Clear buffer
  displaySubMenu();
}

/**
 * @brief Process test mode commands
 */
void processTestCommand(int cmd)
{
  Serial.println();
  switch(cmd) {
    case 1:
      Serial.println("🧪 Starting dummy logging...");
      check_Serial_cmd(dummy_log_start);
      break;
    case 2:
      Serial.println("🛑 Stopping dummy logging...");
      check_Serial_cmd(dummy_log_end);
      break;
    case 3:
      Serial.println("🔍 Testing all slots...");
      for(int i = 0; i < 5; i++) {
        Serial.printf("   Testing Slot %d...\n", i + 1);
        check_Serial_cmd(Hub1_boot + i);
        delay(1000);
      }
      break;
    case 4:
      {
        Serial.println("⚙️  Set Logging Frequency");
        Serial.printf("Current frequency: %u Hz\n", dummy_log_freq);
        Serial.print("Enter new frequency (1-100000 Hz): ");
        
        // Wait for user input
        while(!Serial.available()) {
          delay(10);
        }
        
        String input = Serial.readStringUntil('\n');
        input.trim();
        uint32_t newFreq = input.toInt();
        
        if(newFreq >= 1 && newFreq <= 100000) {
          dummy_log_freq = newFreq;
          
          // Update interval if dummy logging is currently active
          if(dummy_logging_active) {
            dummy_trigger_interval = 1000000 / dummy_log_freq;
            analogWriteFreq(dummy_log_freq); // Update PWM frequency as well
          }
          Serial.printf("✅ Logging frequency set to: %u Hz\n", dummy_log_freq);
        } else {
          Serial.println("❌ Invalid frequency! Must be between 1-100000 Hz");
        }
      }
      break;
    case 5:
      {
        Serial.println("🔄 Reset Sample Counter");
        Serial.printf("Current sample count: %llu\n", dummy_sample_count);
        Serial.print("Reset sample counter to 0? (y/n): ");
        
        // Wait for user input
        while(!Serial.available()) {
          delay(10);
        }
        
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toLowerCase();
        
        if(input == "y" || input == "yes") {
          dummy_sample_count = 0;
          sc = 0;
          Serial.println("✅ Sample counter reset to 0");
        } else {
          Serial.println("❌ Sample counter reset cancelled");
        }
      }
      break;
  }
  Serial.println("✅ Command executed successfully!\n");
  displaySubMenu();
}

/**
 * @brief Validate input range
 */
bool isValidInput(int input, int minVal, int maxVal)
{
  return (input >= minVal && input <= maxVal);
}

/**
 * @brief Power on a specific slot
 */
void powerOnSlot(slot_t slot)
{
  IOEX.digitalWrite(IOEX_pwrPins[slot], LOW);
  Serial.printf("⚡ Slot %d powered ON\n", slot + 1);
}

/**
 * @brief Power off a specific slot
 */
void powerOffSlot(slot_t slot)
{
  IOEX.digitalWrite(IOEX_pwrPins[slot], HIGH);
  Serial.printf("🔌 Slot %d powered OFF\n", slot + 1);
}

/**
 * @brief Get the power status of a slot
 */
bool getSlotPowerStatus(slot_t slot)
{
  return !IOEX.digitalRead(IOEX_pwrPins[slot]);
}

/**
 * @brief Display status of all slots
 */
void displaySlotStatus()
{
  Serial.println("");
  Serial.println("╔══════════════════════════════════════════════════════════╗");
  Serial.println("║                     SLOT STATUS                         ║");
  Serial.println("╠══════════════════════════════════════════════════════════╣");
  for (int i = SLOT1; i <= SLOT5; i++)
  {
    bool powerStatus = getSlotPowerStatus((slot_t)i);
    char statusLine[59];
    if (powerStatus) {
      snprintf(statusLine, sizeof(statusLine), "║  Slot %d: POWERED ON                                     ║", i + 1);
    } else {
      snprintf(statusLine, sizeof(statusLine), "║  Slot %d: POWERED OFF                                    ║", i + 1);
    }
    Serial.println(statusLine);
  }
  Serial.println("╚══════════════════════════════════════════════════════════╝");
}

#endif // BUILD_MASTER