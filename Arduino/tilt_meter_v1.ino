/*
  Title:    Cryologger - Current tilt meter v1
  Date:     January 19, 2020
  Author:   Adam Garbo

  Components:
  - Adafruit Feather M0 Adalogger
  - Adafruit DS3231 RTC Precision Featherwing
  - Adafruit Triple-axis Accelerometer+Magnetometer Board - LSM303

  Comments:
  - Code is currently under development
*/

// Libraries
#include <ArduinoLowPower.h>    // https://github.com/arduino-libraries/ArduinoLowPower
#include <DS3232RTC.h>          // https://github.com/JChristensen/DS3232RTC
#include <math.h>               // https://www.nongnu.org/avr-libc/user-manual/group__avr__math.html
#include <LSM303.h>             // https://github.com/pololu/lsm303-arduino
#include <Statistic.h>          // https://github.com/RobTillaart/Arduino/tree/master/libraries/Statistic
#include <SdFat.h>              // https://github.com/greiman/SdFat
#include <SPI.h>                // https://www.arduino.cc/en/Reference/SPI
#include <Wire.h>               // https://www.arduino.cc/en/Reference/Wire

// Defined constants
#define DEBUG         true  //
#define LED_PIN       8     // Adalogger green LED pin
#define RTC_INT_PIN   5     // RTC interrupt pin
#define SD_CS_PIN     4     // SD card chip select
#define VBAT_PIN      A7    // Battery voltage divider pin

// Object instantiations
LSM303      imu;        // I2C Address: 0x1E (Magnetometer), 0x19 (Accelerometer)
DS3232RTC   rtc(false); // I2C Address: 0x68
SdFat       sd;
SdFile      file;

// User defined global variable declarations
uint32_t  alarmInterval         = 1800;   // Sleep duration (in seconds) between data sample acquisitions. Default = 30 minutes (1800 seconds)
uint16_t  transmitInterval      = 12;     // Number of message to be included in a single transmission (340 byte limit). Default = 12 (Every 6 hours)
uint16_t  maxRetransmitCounter  = 1;      // Maximum number of failed data transmissions to reattempt in a single message (340 byte limit). Default: 10
uint16_t  samplesPerFile        = 40320;  // Maximum number of samples stored in a file before new log file creation (Default: 7 days * 5760 samples per day)

// Declare global variables and constants
volatile bool alarmFlag           = false;                  // RTC alarm interrupt service routine (ISR) flag
volatile bool sleepFlag           = false;                  // Watchdog Timer Early Warning interrupt flag
bool          loggingFlag         = false;                  // SD card logging flag
bool          ledState            = LOW;                    // LED toggle flag for blink() function
char          dirName[9]          = "YYYYMMDD";             // Log file directory name
char          fileName[22]        = "YYYYMMDD/HHMMSS.txt";  // Log file name format limited to 8.3 characters: YYYYMMDD/HHMMSS.ubx
float         heading             = 0;                      // LSM303 heading
float         pitch               = 0;                      // LSM303 pitch
float         roll                = 0;                      // LSM303 roll
float         voltage             = 0;                      // Battery voltage
uint8_t       transmitBuffer[340] = {};                     // RockBLOCK transmission buffer
uint16_t      messageCounter      = 0;                      // Transmission counter
uint16_t      retransmitCounter   = 0;                      // Failed data transmission counter
uint16_t      sampleCounter       = 0;                      // Sensor measurement counter
uint16_t      samplesSaved        = 0;                      // Log file sample counter
uint16_t      transmitCounter     = 0;                      // Transmission interval counter
uint32_t      alarmTime           = 0;                      // RTC alarm time
uint32_t      previousMillis      = 0;                      // Global millis() counter variable
uint32_t      unixtime            = 0;                      // RTC epoch time
time_t        t;
tmElements_t  tm;

// Statistics objects
Statistic batteryStats;   // Battery voltage statistics
Statistic pitchStats;     // Pitch statistics
Statistic rollStats;      // Roll statistics
Statistic headingStats;   // Heading statistics

// Structure and union to store and send data byte-by-byte via RockBLOCK
typedef union {
  struct {
    uint32_t  unixtime;           // Unix epoch                     (4 bytes)
    int16_t   pitch;              // Pitch (°)                      (2 bytes)
    int16_t   roll;               // Roll (°)                       (2 bytes)
    uint16_t  heading;            // Tilt-compensated heading (°)   (2 bytes)
    int32_t   latitude;           // Latitude (DD)                  (4 bytes)
    int32_t   longitude;          // Longitude (DD)                 (4 bytes)
    uint8_t   satellites;         // # of satellites                (1 byte)
    uint16_t  pdop;               // PDOP                           (2 bytes)
    uint16_t  voltage;            // Battery voltage (mV)           (2 bytes)
    uint16_t  transmitDuration;   // Previous message duration      (2 bytes)
    uint16_t  messageCounter;     // Message counter                (2 bytes)
  } __attribute__((packed));                                     // (27-byte message)
  uint8_t bytes[27]; // To do: Look into flexible arrays in structures
} SBDMESSAGE;

SBDMESSAGE message;
size_t messageSize = sizeof(message);   // Size (in bytes) of data message to be transmitted

// Setup
void setup() {

  // Pin assignments
  pinMode(RTC_INT_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_PIN, LOW);

  // Start Serial at 115200 baud
  Serial.begin(115200);
  while (!Serial);    // Wait for user to open Serial Monitor
  //delay(10000);       // Allow 10 seconds for user to open Serial Monitor

  // Configure the Watchdog Timer
  configureWatchdog();

  Serial.println(F("Current Tilt-Meter Prototype"));
  Serial.println(F("----------------------------"));

  // Initialize the RTC
  rtc.begin();
  rtc.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);  // Initialize alarms to known values
  rtc.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
  rtc.alarm(ALARM_1);                         // Clear alarm flags
  rtc.alarm(ALARM_2);
  rtc.alarmInterrupt(ALARM_1, false);         // Clear alarm interrupt flags
  rtc.alarmInterrupt(ALARM_2, false);
  rtc.squareWave(SQWAVE_NONE);                // Configure SQW/INT pin for interrupt operation (disable square wave output)

  // Attach a wakeup interrupt that triggers on the falling edge
  LowPower.attachInterruptWakeup(RTC_INT_PIN, alarmIsr, FALLING);

  /*
    // Manually set the RTC time
    tm.Hour = 11;
    tm.Minute = 59;
    tm.Second = 50;
    tm.Day = 19;
    tm.Month = 1;
    tm.Year = 2020 - 1970;    // tmElements_t.Year is the offset from 1970
    time_t t = makeTime(tm);  // change the tm structure into time_t (seconds since epoch)
    rtc.set(t);
  */

  // Print current date and time
  printDatetime(rtc.get());

  // Initialize the SD card
  if (sd.begin(SD_CS_PIN, SD_SCK_MHZ(4))) {
    Serial.println(F("SD card initialized."));
    loggingFlag = true; // Enable logging
    createLogFile();    // Create new log file
  }
  else {
    Serial.println("Warning: Unable to initialize SD card.");
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    loggingFlag = false; // Disable logging
  }

  // Initialize the IMU
  if (imu.init()) {
    Serial.println(F("LSM303 initalized."));
  }
  else {
    Serial.println(F("Warning: Unable to initialize LSM303."));
    digitalWrite(LED_PIN, HIGH);
    while (1);
  }

  // Set alarm 1
  rtc.setAlarm(ALM1_MATCH_SECONDS, 0, 0, 0, 1); // Set initial alarm to occur at seconds rollover
  //rtc.setAlarm(ALM1_MATCH_MINUTES, 0, 0, 0, 1); // Set initial alarm to occur at minutes rollover (start of new hour)
  rtc.alarm(ALARM_1);                           // Ensure alarm 1 interrupt flag is cleared
  rtc.alarmInterrupt(ALARM_1, true);            // Enable interrupt output for alarm 1
}

// Loop
void loop() {

  // Check if alarm interrupt service routine was triggered
  if (alarmFlag) {

    // Confirm alarm flag is set and not a false trigger (will reset flag if set)
    if (rtc.alarm(ALARM_1)) {

      rtc.read(tm);       // Read current date and time
      t = makeTime(tm);   // Convert tm structure into time_t (seconds since epoch)
      Serial.print(F("Alarm triggered: "));
      printDatetime(t);   // Print ALARM_1 date and time

      sampleCounter++;    // Increment sample counter
      petDog();           // Pet the Watchdog Timer

      // Write data to union
      message.unixtime = t;

      // Perform measurements
      uint32_t loopStartTime = millis();
      // Log data for 2 minutes
      while (millis() - loopStartTime < 2UL * 60UL * 1000UL) {
        readBattery();  // Read battery
        readRtc();      // Read RTC
        readImu();      // Read IMU
        logData();      // Log data to the SD card
        petDog();       // Pet the dog
        blink(LED_PIN, 1, 500);
      }

      // Perform statistics on measurements
      printStatistics();
      calculateStatistics();

      // Write data to transmit buffer array
      writeBuffer();

      // Transmit data
      if (transmitCounter == transmitInterval) {
        transmitData();
        transmitCounter = 0;
      }

      // Set alarm
      alarmTime = t + alarmInterval;  // Calculate next alarm
      rtc.setAlarm(ALM1_MATCH_DATE, 0, minute(alarmTime), hour(alarmTime), day(alarmTime)); // Set alarm
      rtc.alarm(ALARM_1);             // Ensure alarm 1 interrupt flag is cleared
      Serial.print(F("Next alarm: "));
      printDatetime(alarmTime);

    }
    alarmFlag = false; // Clear alarm interrupt service routine flag
  }
  sleepFlag = true;
  blink(LED_BUILTIN, 1, 500);
  //LowPower.deepSleep();   // Enter deep sleep
}

// Measure battery voltage from 100/100 kOhm divider
void readBattery() {

  uint32_t loopStartTime = millis(); // Loop timer

  voltage = 0.0;
  analogReadResolution(12);
  for (uint8_t i = 0; i < 10; i++) {
    voltage += analogRead(VBAT_PIN);
    delay(1);
  }

  voltage /= 10;    // Average readings
  voltage *= 2;     // Multiply back
  voltage *= 3.3;   // Multiply by 3.3V reference voltage
  voltage /= 4096;  // Convert to voltage

  // Add to statistics object
  batteryStats.add(voltage);

  uint32_t loopEndTime = millis() - loopStartTime;
  //Serial.print(F("readBattery() executed in: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));
}

// Read date, time and internal temperature from RTC
void readRtc() {

  uint32_t loopStartTime = millis(); // Loop timer

  unixtime = rtc.get(); // Alarm 1 trigger time

  float temperature = rtc.temperature() / 4.0;  // Read internal temperature of RTC

  // Write data to union
  //message.unixtime = unixtime;

  uint32_t loopEndTime = millis() - loopStartTime;
  //Serial.print(F("readRtc() executed in: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));
}

// RTC alarm interrupt service routine (ISR)
void alarmIsr() {
  alarmFlag = true; // Set alarm flag
}

// Read pitch, roll and tilt-compensated heading
void readImu() {

  uint32_t loopStartTime = millis(); // Loop timer

  imu.enableDefault();  // Enable accelerometer and magnetometer
  delay(1);             // Turn-on delay
  /*
    Calibration values: the default values of +/-32767 for each axis lead to an assumed
    magnetometer bias of 0. Use the Pololu LSM303 library Calibrate example program to
    determine appropriate values for each LSM303 sensor.
  */

  imu.m_min = (LSM303::vector<int16_t>) {
    //-32767, -32767, -32767  // Default
    -702, -778, -802          // Test unit
  };
  imu.m_max = (LSM303::vector<int16_t>) {
    //+32767, +32767, +32767  // Default
    +771, +777, +658          // Test unit
  };

  // Read LSM303
  imu.read();

  // Calculate orientation
  pitch = atan2(-imu.a.x, sqrt((int32_t)imu.a.y * imu.a.y + (int32_t)imu.a.z * imu.a.z)) * 180 / PI;
  roll = atan2(imu.a.y, imu.a.z) * 180 / PI;
  heading = imu.heading((LSM303::vector<int>) {
    1, 0, 0   // PCB orientation
  });

  // Add to statistics object
  pitchStats.add(pitch);
  rollStats.add(roll);
  headingStats.add(heading);

  // Place accelerometer in power-down mode
  imu.writeAccReg(0x20, 0x00); // CTRL_REG1_A = 0b00000000

  // Place magenteometer in sleep-mode
  imu.writeMagReg(0x02, 0x03); // MR_REG_M = 0b00000011

  uint32_t loopEndTime = millis() - loopStartTime; // Loop timer
  //Serial.print(F("readImu() executed in: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));
}

// Create a new log file
void createLogFile() {

  // Check if logging is enabled
  if (loggingFlag == false) {
    return;
  }

  // Close any open files
  if (file.isOpen()) {
    file.close();
  }

  // Read current date and time
  rtc.read(tm);

  // Create new folder
  snprintf(dirName, sizeof(dirName), "%04u%02d%02d", (tm.Year + 1970), tm.Month, tm.Day);
  if (sd.mkdir(dirName)) {
    Serial.print("Created folder: ");
    Serial.println(dirName);
  }
  else {
    Serial.println("Warning: Unable to create new folder.");
  }

  // Create log file
  snprintf(fileName, sizeof(fileName), "%04u%02d%02d/%02d%02d%02d.csv",
           (tm.Year + 1970), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second);

  if (file.open(fileName, O_CREAT | O_WRITE | O_EXCL)) {
    Serial.print("Logging to: ");
    Serial.println(fileName);
  }
  else {
    Serial.println(F("Warning: Unable to open new log file"));
    loggingFlag = false;
  }

  // Set the log file creation time
  if (!file.timestamp(T_CREATE, (tm.Year + 1970), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second)) {
    Serial.println(F("Warning: Unable to set file create timestamp."));
  }

  // Write header to file
  file.println("sample, datetime, pitch, roll, heading, voltage");
  file.sync();

  // Close log file
  file.close();
}

// Log data to the DS card
void logData() {

  // Check if logging is enabled
  if (loggingFlag == false) {
    return;
  }

  // Check if samples per file limit has been exceeded
  if (samplesSaved >= samplesPerFile) {
    createLogFile();
    samplesSaved = 0;
  }

  // Write data to SD card
  if (file.open(fileName, O_APPEND | O_WRITE)) {
    samplesSaved++;   //  Increment sample count of current file
    file.print(samplesSaved);
    file.print(",");
    file.print(unixtime);
    file.print(",");
    file.print(pitch);
    file.print(",");
    file.print(roll);
    file.print(",");
    file.print(heading);
    file.print(",");
    file.println(voltage);
    writeTimestamps();
    file.close();

#if DEBUG
    Serial.print(samplesSaved);
    Serial.print(",");
    Serial.print(unixtime);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(roll);
    Serial.print(",");
    Serial.print(heading);
    Serial.print(",");
    Serial.println(voltage);
#endif
  }
  else {
    Serial.println(F("Warning: Unable to open file. Halting."));
    while (1);
  }

  // Force data to SD and update the directory entry to avoid data loss
  if (!file.sync() || file.getWriteError()) {
    Serial.println(F("Write error"));
  }

  // Close the file
  file.close();
}

// Log file write and access timestamps
void writeTimestamps() {

  // Read current date and time from the RTC
  rtc.read(tm);

  // Set the file's last write/modification date and time
  if (!file.timestamp(T_WRITE, (tm.Year + 1970), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second)) {
    Serial.println(F("Set write time failed"));
  }

  // Set the file's last access date and time
  if (!file.timestamp(T_ACCESS, (tm.Year + 1970), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second)) {
    Serial.println(F("Set access time failed"));
  }
}

// Calculate statistics and clear objects
void calculateStatistics() {

  // Write  statistics data to union
  message.pitch   = pitchStats.average()      * 100;  // Pitch mean
  message.roll    = rollStats.average()       * 100;  // Roll mean
  message.heading = headingStats.average()    * 100;  // Heading mean
  message.voltage = batteryStats.minimum()    * 1000; // Battery voltage min

  // Clear statistics objects
  batteryStats.clear();
  pitchStats.clear();
  rollStats.clear();
  headingStats.clear();
}

// Write union data to transmit buffer in preparation of transmission
void writeBuffer() {
  messageCounter++; // Increment message counter
  message.messageCounter = messageCounter; // Write message counter data to union
  transmitCounter++; // Increment data transmission counter

  // Concatenate current message with existing message(s) stored in transmit buffer
  memcpy(transmitBuffer + (sizeof(message) * (transmitCounter + (retransmitCounter *
                           transmitInterval) - 1)), message.bytes, sizeof(message));

#if DEBUG
  printUnion();
  //printUnionBinary(); // Print union/structure in hex/binary
  //printTransmitBuffer();  // Print transmit buffer in hex/binary
  Serial.println(transmitCounter);
#endif
}

// Transmit data
void transmitData() {
  Serial.println(F("Transmit data"));
}

// Print current time and date
void printDatetime(time_t t) {
  char dateBuffer[25];
  snprintf(dateBuffer, sizeof(dateBuffer), "%04u-%02d-%02dT%02d:%02d:%02d",
           year(t), month(t), day(t), hour(t), minute(t), second(t));
  Serial.println(dateBuffer);
}

// Blink LED (non-blocking)
void blink(uint8_t ledPin, uint8_t flashes, uint16_t interval) {
  uint8_t i = 0;
  while (i < (flashes * 2)) {
    uint32_t currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      if (ledState == LOW)
        ledState = HIGH;
      else
        ledState = LOW;
      digitalWrite(ledPin, ledState);
      i++;
    }
  }
}



// Set up the WDT to perform a system reset if the loop() blocks for more than 16 seconds
void configureWatchdog() {

  // Set up the generic clock (GCLK2) used to clock the watchdog timer at 1.024kHz
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(4) |          // Divide the 32.768kHz clock source by divisor 32, where 2^(4 + 1): 32.768kHz/32=1.024kHz
                    GCLK_GENDIV_ID(2);            // Select Generic Clock (GCLK) 2
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_DIVSEL |        // Set to divide by 2^(GCLK_GENDIV_DIV(4) + 1)
                     GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK2
                     GCLK_GENCTRL_SRC_OSCULP32K | // Set the clock source to the ultra low power oscillator (OSCULP32K)
                     GCLK_GENCTRL_ID(2);          // Select GCLK2
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Feed GCLK2 to WDT (Watchdog Timer)
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK2 to the WDT
                     GCLK_CLKCTRL_GEN_GCLK2 |     // Select GCLK2
                     GCLK_CLKCTRL_ID_WDT;         // Feed the GCLK2 to the WDT
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  WDT->EWCTRL.bit.EWOFFSET = 0xA;                 // Set the Early Warning Interrupt Time Offset to 8 seconds //REG_WDT_EWCTRL = WDT_EWCTRL_EWOFFSET_8K;
  WDT->INTENSET.bit.EW = 1;                       // Enable the Early Warning interrupt //REG_WDT_INTENSET = WDT_INTENSET_EW;
  WDT->CONFIG.bit.PER = 0xB;                      // Set the WDT reset timeout to 16 seconds //REG_WDT_CONFIG = WDT_CONFIG_PER_16K;
  WDT->CTRL.bit.ENABLE = 1;                       // Enable the WDT in normal mode //REG_WDT_CTRL = WDT_CTRL_ENABLE;
  while (WDT->STATUS.bit.SYNCBUSY);               // Await synchronization of registers between clock domains

  // Configure and enable WDT interrupt
  NVIC_DisableIRQ(WDT_IRQn);
  NVIC_ClearPendingIRQ(WDT_IRQn);
  NVIC_SetPriority(WDT_IRQn, 0);                  // Top priority
  NVIC_EnableIRQ(WDT_IRQn);
}

// Pet the Watchdog Timer
void petDog() {
  WDT->CLEAR.bit.CLEAR = 0xA5;      // Clear the Watchdog Timer and restart time-out period //REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.bit.SYNCBUSY); // Await synchronization of registers between clock domains
}

// Watchdog Timer interrupt service routine (ISR)
void WDT_Handler() {

  if (sleepFlag) {
    sleepFlag = false;
    WDT->INTFLAG.bit.EW = 1;          // Clear the Early Warning interrupt flag //REG_WDT_INTFLAG = WDT_INTFLAG_EW;
    WDT->CLEAR.bit.CLEAR = 0xA5;      // Clear the Watchdog Timer and restart time-out period //REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;
    while (WDT->STATUS.bit.SYNCBUSY); // Await synchronization of registers between clock domains
  }
  else {
    //WDT->CTRL.bit.ENABLE = 0; // Debug: Disable Watchdog Timer
    //digitalWrite(13, HIGH);   // Debug: Turn on LED to indicate Watchdog trigger
    while (true);             // Force Watchdog Timer to reset the system
  }
}

// Print statisticsitre
void printStatistics() {
  Serial.println(F("-------------------------------------------------------------------------"));
  Serial.println(F("Statistics"));
  Serial.println(F("-------------------------------------------------------------------------"));
  Serial.println(F("Voltage:"));
  Serial.print(F("Samples: ")); Serial.print(batteryStats.count());
  Serial.print(F("\tMin: "));   Serial.print(batteryStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(batteryStats.maximum());
  Serial.print(F("\tMean: ")); Serial.print(batteryStats.average());
  Serial.print(F("\tSD: ")); Serial.println(batteryStats.unbiased_stdev());
  Serial.println(F("Pitch:"));
  Serial.print(F("Samples: ")); Serial.print(pitchStats.count());
  Serial.print(F("\tMin: ")); Serial.print(pitchStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(pitchStats.maximum());
  Serial.print(F("\tMean: ")); Serial.print(pitchStats.average());
  Serial.print(F("\tSD: ")); Serial.println(pitchStats.unbiased_stdev());
  Serial.println(F("Roll:"));
  Serial.print(F("Samples: ")); Serial.print(rollStats.count());
  Serial.print(F("\tMin: ")); Serial.print(rollStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(rollStats.maximum());
  Serial.print(F("\tMean: ")); Serial.print(rollStats.average());
  Serial.print(F("\tSD: ")); Serial.println(rollStats.unbiased_stdev());
  Serial.println(F("Heading:"));
  Serial.print(F("Samples: ")); Serial.print(headingStats.count());
  Serial.print(F("\tMin: ")); Serial.print(headingStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(headingStats.maximum());
  Serial.print(F("\tMean: ")); Serial.print(headingStats.average());
  Serial.print(F("\tSD: ")); Serial.println(headingStats.unbiased_stdev());
}

// Print union/structure
void printUnion() {
  Serial.println(F("-----------------------------------"));
  Serial.println(F("Union/structure"));
  Serial.println(F("-----------------------------------"));
  Serial.print(F("unixtime:\t\t")); Serial.println(message.unixtime);
  Serial.print(F("pitch:\t\t\t")); Serial.println(message.pitch);
  Serial.print(F("roll:\t\t\t")); Serial.println(message.roll);
  Serial.print(F("heading:\t\t")); Serial.println(message.heading);
  Serial.print(F("latitude:\t\t")); Serial.println(message.latitude);
  Serial.print(F("longitude:\t\t")); Serial.println(message.longitude);
  Serial.print(F("satellites:\t\t")); Serial.println(message.satellites);
  Serial.print(F("pdop:\t\t\t")); Serial.println(message.pdop);
  Serial.print(F("voltage:\t\t")); Serial.println(message.voltage);
  Serial.print(F("transmitDuration:\t")); Serial.println(message.transmitDuration);
  Serial.print(F("messageCounter:\t\t")); Serial.println(message.messageCounter);
  Serial.println(F("-----------------------------------"));
}

// Print contents of union/structure
void printUnionBinary() {
  Serial.println(F("Union/structure "));
  Serial.println(F("-----------------------------------"));
  Serial.println(F("Byte\tHex\tBinary"));
  for (uint16_t i = 1; i <= sizeof(message); ++i) {
    Serial.print(i);
    Serial.print("\t");
    Serial.print(message.bytes[i], HEX);
    Serial.print("\t");
    Serial.println(message.bytes[i], BIN);
  }
  Serial.println(F("-----------------------------------"));
}

// Print contents of transmiff buffer array
void printTransmitBuffer() {
  Serial.println(F("Transmit buffer"));
  Serial.println(F("-----------------------------------"));
  Serial.println(F("Byte\tHex\tBinary"));
  for (uint16_t i = 1; i <= 340; i++) {
    Serial.print(i);
    Serial.print(F("\t"));
    Serial.print(transmitBuffer[i], HEX);
    Serial.print("\t");
    Serial.println(transmitBuffer[i], BIN);
  }
}
