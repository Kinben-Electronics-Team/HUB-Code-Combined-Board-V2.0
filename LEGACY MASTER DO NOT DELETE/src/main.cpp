#include "hub_master.h"

// ShiftRegister SR(3, SRDA_pin, STCLK_pin);
void receiveEvent(int howMany);
void requestEvent();
void Channel_sel(slot_t Channel);
void trigger();
void HUB_BOOT(slot_t HUB);
void check_mode();
void send_I2C_cmd(slot_t slot, const uint8_t *cmd, uint8_t n);

uint8_t wireBroadcast(const uint8_t *data, uint8_t n);
void check_Serial_cmd(uint8_t cmd);
void checkCSB(cmd_t *c);

bool mode_stat = OTHER_MODE, current_mode = 0;
volatile bool Trig_stat = 0;

// const uint8_t I2C_CMD_SD_CON = 0x04;    // command to connect sd card to card reader
//  const uint8_t I2C_CMD_SD_DISCON = 0x05; // command to disconect sd card from sd card reader ic

cmd_t configRX; // struct to receive data from CSB
configData cd;  // struct to store configuration data

EasyTransfer EscRX, // EasyTransfer object to get sample count from CSB
    EconfigRX,      // EasyTransfer object to get Cofiguration from CSB
    EstatTX,        // EasyTransfer object to send config data data to CSB
    EscTX;          // EasyTransfer object to send sample count to CSB
                    // EscTX_I2C;      // EasyTransfer object to send sample count to I2C IO

uint64_t sc = 0; // sample count to be received from CSB

MyI2CDevice IOEX(&IOEX_Wire, 0x74, &Serial); // use actual I²C address macro or value
Adafruit_NeoPixel pixel(1, LED_pin, NEO_GRB + NEO_KHZ800);

void checkCSB(cmd_t *c);

void setup()
{
  set_sys_clock_khz(133000, false); // set system clock to 133MHz
  Serial.begin(BAUDRATE);           // set baudrate for serial communication

  /*CSB serial communication init*/
  CSB_Serial.setRX(DATA_RX_pin);
  CSB_Serial.setTX(DATA_TX_pin);
  CSB_Serial.begin(CSB_BR); // Initialize CSB Serial communication
  // while (!Serial)           // wait for serial port to connect

  //   delay(1000); // wait for serial port to connect
  // Serial.println("Serial port connected");
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
  // attachInterrupt(TRIG_pin, trigger, RISING);

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

  // Channel_sel(SLOT1); // select HUB1 on initialisation
}

uint32_t p = 0, x = 0;
void loop()
{
  check_mode();

  if (mode_stat == ACQ_MODE && Trig_stat) // Acquisition mode and trigger pin is high
  {
    Trig_stat = 0;
    digitalWrite(SLOT_TRIG_pin, HIGH);
    if (!EscRX.receiveData()) // receive sample count from CSB
      sc = 0;                 // reset sample count if not received
    // sc++;             // increment sample count
    EscTX.sendData(); // send sample count to CSB

    delayMicroseconds(200);
    digitalWrite(SLOT_TRIG_pin, LOW);
  }
  else // other mode
  {

    if (Serial.available())
    {
      int cmd = Serial.parseInt();
      while (Serial.available())
      {
        char dump = Serial.read();
      }
      Serial.printf("cmd = %d\n", cmd);
      check_Serial_cmd(cmd);
    }
  }
  if (EconfigRX.receiveData()) // receive configuration from CSB
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
      Serial.end();
      attachInterrupt(TRIG_pin, trigger, RISING);
    }
    else
    {
      pixel.setPixelColor(0, pixel.Color(255, 0, 0)); // Set NeoPixel color to Green
      digitalWrite(TX_EN_pin, TX_EN);
      detachInterrupt(TRIG_pin);
      Serial.begin(BAUDRATE);
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
  // case Hub6_boot:
  //   HUB_BOOT(HUB6);
  //   break;
  // case Hub7_boot:
  //   HUB_BOOT(HUB7);
  //   break;
  // case Hub8_boot:
  //   HUB_BOOT(HUB8);
  //   break;
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
  // case Hub6_SD_conn:
  //   configSlave[0] = cmd_sd_con;
  //   send_I2C_cmd(5, configSlave, 1);
  //   break;
  // case Hub7_SD_conn:
  //   configSlave[0] = cmd_sd_con;
  //   send_I2C_cmd(6, configSlave, 1);
  //   break;
  // case Hub8_SD_conn:
  //   configSlave[0] = cmd_sd_con;
  //   send_I2C_cmd(7, configSlave, 1);
  //   break;
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
  // case Hub6_sel:
  //   Channel_sel(5);
  //   break;
  // case Hub7_sel:
  //   Channel_sel(6);
  //   break;
  // case Hub8_sel:
  //   Channel_sel(7);
  //   break;
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
    Serial.println("Dummy logging started");
    digitalWrite(SLOT_MODE_pin, HIGH); // Set slot mode to logging
    delay(1000);                       // Wait for 1 second
    analogWriteFreq(1000);             // Set PWM frequency to 1kHz
    analogWrite(SLOT_TRIG_pin, 128);   // Set 50% duty cycle

    break;

  case dummy_log_end:
    // Code to end dummy logging (needs to be defined)
    Serial.println("Dummy logging ended");
    digitalWrite(SLOT_MODE_pin, LOW); // Set slot mode to other
    analogWrite(SLOT_TRIG_pin, 0);    // Set duty cycle to 0%
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

      // case cmd_pwr_en:
      //   if (c->value > 0 && c->value < 8)
      //   {
      //     io_expander_set_ldo(c->value - 1, true); // enable power
      //     // SR.set_val_MSB_first(~(1 << ((one_chip_output * LDO_EN_SR) + c->value - 1))); // enable power to the hub board
      //   }
      //   else
      //   {
      //     c->value = 0xff;                                                              // set invalid command
      //     SR.set_val_MSB_first(~(1 << ((one_chip_output * LDO_EN_SR) + c->value - 1))); // enable power to the hub board
      //     Serial.printf("Received invalid command: %d\n", c->param);
      //   }
      //   break;

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

  Wire.endTransmission();

  return Wire.endTransmission(1);
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
  // for(int i =0;i<howMany;i++)
  // {
  //   Serial.print(c[i],HEX);
  //   Serial.print(" ");
  // }
  Serial.println(val);
  if (val - prev_val != 1)
  {
    Serial.print("data loss: ");
    Serial.println(val - prev_val);
  }
  prev_val = val;

  // Serial.println();
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