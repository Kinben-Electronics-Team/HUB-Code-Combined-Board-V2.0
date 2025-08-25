# Hub_master_code
HUB Master board
function to boot any selected board
function  to select channel
MODE_pin setup ; mode 1 : acqusition ; mode 0 : other
trigger receive from csb and send to hub board
get adc data and Broadcast via I2C after SENSOR_ACQ_DELAY
SD card connect disconnect handling via commands
Serial cmd setup
SDCARD_HUB_DET_pin : drive low to detect sd card by card reader IC

Time analysis
time to read adc data = 22us
time to broadcast adc data = 30us