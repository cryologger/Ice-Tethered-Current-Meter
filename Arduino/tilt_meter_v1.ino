/*
  Title:    Cryologger - Current tilt meter V1
  Date:     January 29, 2020
  Author:   Adam Garbo

    Components:
    - SparkFun Qwiic Micro - SAMD21 Development Board
    - SparkFun Real Time Clock Module - RV-1805 (Qwiic)
    - SparkFun Atmospheric Sensor Breakout - BME280 (Qwiic)
    - SparkFun GPS Breakout - ZOE-M8Q (Qwiic)
    - SparkFun Qwiic Iridium 9603N
    - Maxtena M1621HCT-P-SMA Iridium antenna
    - Maxtena M1516HCT-P-SMA antenna
    - Adafruit Triple-axis Accelerometer+Magnetometer Board - LSM303
    - SparkFun Buck-Boost Converter

  Comments:
  - Code is currently under development
*/

// Libraries
#include <Wire.h>                           // https://www.arduino.cc/en/Reference/Wire
#include <TimeLib.h>                        // https://github.com/PaulStoffregen/Time
#include <SparkFun_Ublox_Arduino_Library.h> // https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
#include <SparkFun_RV1805.h>                // https://github.com/sparkfun/SparkFun_RV-1805_Arduino_Library
#include <SparkFunLSM9DS1.h>                // https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library
#include <SparkFunBME280.h>                 // https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
#include <IridiumSBD.h>                     // https://github.com/PaulZC/IridiumSBD
#include <ArduinoLowPower.h>                // https://github.com/arduino-libraries/ArduinoLowPower

// Defined constants
#define Serial        SerialUSB   // Required by SparkFun Qwiic Micro 
#define IridiumWire   Wire        // 
#define GPS_INT_PIN   5           // Pin to control ON/OFF operation of ZOE-M8Q
#define RTC_INT_PIN   4           // RTC interrupt pin on D4
#define DEBUG         true        // Output debugging messages to Serial Monitor
#define DIAGNOSTICS   true        // Output Iridium diagnostic messages to Serial Monitor

// Object instantiations
IridiumSBD    modem(IridiumWire); // I2C Address: 0x63
RV1805        rtc;                // I2C Address: 0x69
SFE_UBLOX_GPS gps;                // I2C Address: 0x42

// User defined global variable declarations
uint32_t      alarmInterval         = 1800;   // RTC sleep duration in seconds (Default: 3600 seconds)
uint32_t      sampleInterval        = 120;    // Duration in seconds of current tilt measurements
uint8_t       sampleFrequency       = 1;      // Sampling frequecny in seconds of current tilt measurements
uint8_t       transmitInterval      = 1;      // Number of messages in each Iridium transmission (Limit: 340 bytes)
uint8_t       maxRetransmitCounter  = 13;     // Number of failed messages to reattempt in each Iridium transmission (Limit: 340 bytes)

// Global variable and constant declarations
bool          ledState              = LOW;    // Flag to toggle LED in blinkLed() function
bool          setRtcFlag            = true;   // Flag to determine if RTC should be set using GPS time
volatile bool sleepFlag             = false;  // Flag to indicate to Watchdog Timer if in deep sleep mode
volatile bool alarmFlag             = false;  // Flag for alarm interrupt service routine
uint8_t       resetFlag             = 0;      // Flag to force system reset using Watchdog Timer
uint8_t       transmitBuffer[340]   = {};     // Qwiic 9603N transmission buffer
uint16_t      messageCounter        = 0;      // Qwiic 9603N transmitted message counter
uint16_t      retransmitCounter     = 0;      // Qwiic 9603N failed data transmission counter
uint16_t      transmitCounter       = 0;      // Qwiic 9603N transmission interval counter
uint32_t      previousMillis        = 0;      // GLobal millis() timer variable
time_t        alarmTime             = 0;
time_t        unixtime              = 0;
tmElements_t  tm;

// Configure u-blox M8Q receiver for ON/OFF operation (UBX-CFG-PM2)
static uint8_t ubxCfgPm2_payload[] = {
  0x01, 0x06, 0x00, 0x00, 0x6E, 0x00, 0x42, 0x01, 0xE8, 0x03, 0x00,
  0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x86,
  0x02, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00
};

ubxPacket ubxCfgPm2 = {
  0x06, 0x3b, 44, 0, 0, ubxCfgPm2_payload, 0, 0, false
};

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
  pinMode(GPS_INT_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(GPS_INT_PIN, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  //while (!Serial);      // Prevent execution of script until Serial Monitor is open
  delay(5000);         // Delay to allow for opening of Serial Monitor

  // Configure Watchdog Timer
  configureWatchdog();

  Serial.println(F("Cryologger - Current Tilt Metre V1"));
  Serial.println(F("----------------------------------"));

  // I2C Configuration
  Wire.begin();           // Initialize I2C
  Wire.setClock(400000);  // Set I2C clock speed to 400kHz

  // SparkFun RV-1805 RTC Configuration
  /*
    Alarm trigger modes:
    0: Disabled
    1: seconds, minutes, hours, date and month match (once per year)
    2: seconds, minutes, hours and date match (once per month)
    3: seconds, minutes, hours and weekday match (once per week)
    4: seconds, minutes and hours match (once per day)
    5: seconds and minutes match (once per hour)
    6: seconds match (once per minute)
    7: once per second
  */
  if (rtc.begin()) {
    Serial.println(F("SparkFun RV-1805 RTC detected."));
    //rtc.disableTrickleCharge();             // Disable capacitor trickle charger
    rtc.set24Hour();                        // Set RTC to 24-hour format
    rtc.setAlarm(0, 0, 0, 0, 0);            // Set the alarm (seconds, minutes, hours, day, month)
    rtc.setAlarmMode(5);                    // Set alarm mode (select mode 5 for deployments)
    rtc.enableInterrupt(INTERRUPT_AIE);     // Enable the Alarm Interrupt
    rtc.updateTime();                       // Update time variable from RTC registers

    // Configure and attach interrupt on the CLK/INT pin
    pinMode(RTC_INT_PIN, INPUT_PULLUP);
    LowPower.attachInterruptWakeup(RTC_INT_PIN, alarmIsr, FALLING);
  }
  else {
    Serial.println(F("SparkFun RV-1805 RTC not connected! Please check wiring."));
  }

  // Adafruit LSM303 Configuration
  if (imu.init()) {
    Serial.println(F("LSM303 initalized."));
  }
  else {
    Serial.println(F("Warning: Unable to initialize LSM303."));
    digitalWrite(LED_PIN, HIGH);
    while (1);
  }

  // SparkFun ZOE-M8Q Configuration
  if (gps.begin() == true) {
    Serial.println(F("SparkFun ZOE-M8Q detected."));
    //gps.enableDebugging(Serial);    // Enable debug messages over Serial
    gps.setI2COutput(COM_TYPE_UBX); // Set I2C port to output UBX only (turn off NMEA noise)
    gps.sendCommand(ubxCfgPm2);     // Configure UBX-CFG-PM2 for ON/OFF operation
    gps.saveConfiguration();        // Save current settings to flash and BBR
  }
  else {
    Serial.println(F("u-blox ZOE-M8Q not detected at default I2C address. Please check wiring."));
  }

  // SparkFun Qwiic Iridium 9603N Configuration
  if (modem.isConnected()) {
    Serial.println(F("SparkFun Qwiic Iridium 9603N detected."));
    //modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE); // Assume 'USB' power (slow recharge)
    modem.adjustSendReceiveTimeout(180);   // Set send/receive timeout (Default = 300 seconds)
    modem.enable841lowPower(true);        // Enable ATtiny841 low-power mode
  }
  else {
    Serial.println(F("Qwiic Iridium 9603N not connected! Please check wiring."));
  }

#if DEBUG
  Serial.print("Setup: "); Serial.println(rtc.stringTimeStamp());  // Print date and time in ISO 8601 format
  Serial.print("Alarm: "); alarmTimeStamp();
#endif
}


// Loop
void loop() {

  // Check if alarm interrupt service routine was triggered
  if (alarmFlag) {

    // Confirm alarm flag was set and not a false trigger (will reset alarm flag if set)
    if (rtc.status() & 0x04) { // Alarm Flag bit in register 0Fh will be set to 1: 0b00000100 (0x04)

      // Read the RTC
      readRtc();
      
      // Pet the Watchdog Timer
      petDog(); 
      
      // Print date and time in ISO 8601 format
      Serial.print("Alarm trigger: "); Serial.println(rtc.stringTimeStamp());
      
      // Perform measurements for 2 minutes
      uint32_t loopStartTime = millis();
      while (millis() - loopStartTime < sampleInterval * 60UL * 1000UL) {
        petDog();       // Pet the dog
        readBattery();  // Read battery
        readImu();      // Read IMU
        blinkLed(LED_PIN, 1, sampleFrequency);
      }
      
      // Read GPS
      readGps();
      
      // Perform statistics on measurements
      printStatistics();
      calculateStatistics();

      // Write data to transmit buffer array
      writeBuffer();

      // Check if data should be transmitted
      if (transmitCounter == transmitInterval) {
        transmitData();
        transmitCounter = 0;
      }
      
      // Set RTC alarm
      alarmTime = unixtime + alarmInterval; // Calculate next alarm
      rtc.setAlarm(0, minute(alarmTime), hour(alarmTime), day(alarmTime), month(alarmTime));
      rtc.setAlarmMode(2); // Set alarm mode to seconds, minutes, hours and day match (once per month)

      // Check if alarm was set in the past
      if (alarmTime <= getEpoch()) {
        Serial.println(F("Warning! Alarm set in the past."));
        alarmTime = getEpoch() + alarmInterval; // Calculate new alarm

        // Set alarm to next closest alarm interval
        rtc.setAlarm(0, 0, (hour(alarmTime) + ((alarmInterval / 3600) - (hour(alarmTime) % (alarmInterval / 3600)))) % 24, 0, 0);
        rtc.setAlarmMode(4); // Set alarm mode to seconds, minutes and hours match (once per day)
        setRtcFlag = false;
      }
      rtc.enableInterrupt(INTERRUPT_AIE); // Enable Alarm Interrupt
      Serial.print(F("Next alarm: ")); alarmTimeStamp();
    }
    alarmFlag = false; // Clear alarm interrupt service routine flag
  }
  sleepFlag = true; // Set Watchdog Timer sleep flag
  blinkLed(1, 10);
  LowPower.deepSleep(); // Enter deep sleep
}

// Measure battery voltage from 100/100 kOhm divider
void readBattery() {

  // Start loop timer
  uint32_t loopStartTime = millis(); 

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

  // Stop loop timer
  uint32_t loopEndTime = millis() - loopStartTime;
  Serial.print(F("readBattery() executed in: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));
}

// Read SparkFun RV-1805 RTC
void readRtc() {

  // Start loop timer
  uint32_t loopStartTime = micros();

  // Get UNIX Epoch time
  unixtime = getEpoch();

  // Write data to union
  message.unixtime = unixtime;
  Serial.print("Epoch time: "); Serial.println(unixtime);

  // Stop loop timer
  uint32_t loopEndTime = micros() - loopStartTime;
  Serial.print(F("readRtc() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" μs"));
}

// Convert date and time to Unix Epoch time
uint32_t getEpoch() {

  // Update the local array with the RTC registers
  rtc.updateTime();

  // Convert to Unix epoch (tm to time_t)
  tmElements_t tm;
  tm.Second = rtc.getSeconds();
  tm.Minute = rtc.getMinutes();
  tm.Hour   = rtc.getHours();
  tm.Day    = rtc.getDate();
  tm.Month  = rtc.getMonth();
  tm.Year   = rtc.getYear() + 30; // Offset from 2000 - 1970
  time_t t = makeTime(tm);

  return t;
}

// Print alarm timestamp
void alarmTimeStamp () {
  char alarmBuffer[25];
  snprintf(alarmBuffer, sizeof(alarmBuffer), "2020-%02d-%02dT%02d:%02d:%02d",
           rtc.BCDtoDEC(rtc.readRegister(0x0D)),  // Alarm month register
           rtc.BCDtoDEC(rtc.readRegister(0x0C)),  // Alarm day register
           rtc.BCDtoDEC(rtc.readRegister(0x0B)),  // Alarm hour register
           rtc.BCDtoDEC(rtc.readRegister(0x0A)),  // Alarm minute register
           rtc.BCDtoDEC(rtc.readRegister(0x09))); // Alarm second register
  Serial.println(alarmBuffer);
}

// RTC alarm interrupt service routine
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

// Read SparkFun ZOE-M8Q
void readGps() {

  // Start loop timer
  uint32_t loopStartTime = millis();

  bool fixFound = false;
  uint8_t fixCounter = 0;

  // Wake up the receiver
  digitalWrite(GPS_INT_PIN, HIGH);

  // Blink LED
  blinkLed(2, 100);

  // Begin listening to the GPS
  Serial.println(F("Beginning to listen for GPS traffic..."));

  // Look for GPS signal for up to 2 minutes
  while (!fixFound && millis() - loopStartTime < 1UL * 60UL * 1000UL) {

#ifdef DEBUG
    int32_t latitude = gps.getLatitude();
    int32_t longitude = gps.getLongitude();
    uint16_t pdop = gps.getPDOP();
    uint8_t fix = gps.getFixType();
    uint8_t satellites = gps.getSIV();

    char datetime[20]; // GNSS date time buffer
    snprintf(datetime, sizeof(datetime), "%04u-%02d-%02d %02d:%02d:%02d",
             gps.getYear(), gps.getMonth(), gps.getDay(),
             gps.getHour(), gps.getMinute(), gps.getSecond());

    Serial.print(datetime);
    Serial.print(F(" Latitude: ")); Serial.print(latitude);
    Serial.print(F(" Longitude: ")); Serial.print(longitude);
    Serial.print(F(" Satellites: ")); Serial.print(satellites);
    Serial.print(F(" Fix: ")); Serial.print(fix);
    Serial.print(F(" PDOP: ")); Serial.println(pdop);
#endif

    // Check for GNSS 2D/3D position fix
    if (gps.getFixType() >= 2) {
      fixCounter += 1; // Increment fixCounter
    }

    // Check if enough 2D/3D position fixes were collected
    if (fixCounter >= 10) {
      Serial.println(F("A GPS fix was found!"));
      fixFound = true;

      // Write data to union
      message.latitude = gps.getLatitude();
      message.longitude = gps.getLongitude();
      message.satellites = gps.getSIV();
      message.pdop = gps.getPDOP();
      message.fix = gps.getFixType();

      // Check if RTC time should be set using GPS time
      if (setRtcFlag) {
        rtc.updateTime();
        Serial.print("Old time: "); Serial.println(rtc.stringTimeStamp());
        rtc.setTime(0, gps.getSecond(), gps.getMinute(), gps.getHour(),
                    gps.getDay(), gps.getMonth(), gps.getYear(), 0);
        rtc.updateTime(); 
        Serial.print("RTC time set: "); Serial.println(rtc.stringTimeStamp());
      }
    }
    ISBDCallback();
  }

  // Did we get a GPS fix?
  if (!fixFound) {
    Serial.println(F("No GPS fix was found."));
  }

  // Stop loop timer
  uint32_t loopEndTime = millis() - loopStartTime;
  Serial.print(F("readGps() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));

  // Place receiver in sleep/backup mode
  digitalWrite(GPS_INT_PIN, LOW);
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

// Write union data to transmit buffer in preparation of data transmission
void writeBuffer() {
  messageCounter++;                         // Increment message counter
  message.messageCounter = messageCounter;  // Write message counter data to union
  transmitCounter++;                        // Increment data transmission counter

  // Concatenate current message with existing message(s) stored in transmit buffer
  memcpy(transmitBuffer + (sizeof(message) * (transmitCounter + (retransmitCounter * transmitInterval) - 1)),
         message.bytes, sizeof(message)); // Copy message to transmit buffer

#if DEBUG
  printUnion();
  //printUnionBinary(); // Print union/structure in hex/binary
  //printTransmitBuffer();  // Print transmit buffer in hex/binary
#endif
}

// Transmit data
void transmitData() {

  // Start loop timer
  uint32_t loopStartTime = millis();
  int16_t err;

  // Enable the supercapacitor charger
  Serial.println(F("Enabling the supercapacitor charger..."));
  modem.enableSuperCapCharger(true);

  // Wait for the supercapacitor charger PGOOD signal to go high for no more than 2 minutes
  Serial.println(F("Waiting for supercapacitors to charge..."));
  while (!modem.checkSuperCapCharger() && millis() - loopStartTime < 2UL * 60UL * 1000UL) {
    blinkLed(1, 500);
    petDog();
  }
  Serial.println(F("Supercapacitors charged!"));

  // Enable power to the Qwiic 9603N
  Serial.println(F("Enabling 9603N power..."));
  modem.enable9603Npower(true);

  // Begin satellite modem operation
  Serial.println(F("Starting modem..."));
  err = modem.begin();
  if (err == ISBD_SUCCESS) {
    uint8_t inBuffer[240];  // Buffer to store incoming transmission (240 byte limit)
    size_t inBufferSize = sizeof(inBuffer);
    memset(inBuffer, 0x00, sizeof(inBuffer)); // Clear inBuffer array

    /*
        // Test the signal quality
        int signalQuality = -1;
        err = modem.getSignalQuality(signalQuality);
        if (err != ISBD_SUCCESS) {
          Serial.print(F("SignalQuality failed: error "));
          Serial.println(err);
          return;
        }
      Serial.print(F("On a scale of 0 to 5, signal quality is currently: "));
      Serial.println(signalQuality);
    */

    // Transmit and receieve data in binary format
    Serial.println(F("Attempting to transmit data..."));
    err = modem.sendReceiveSBDBinary(transmitBuffer, (sizeof(message) * (transmitCounter + (retransmitCounter * transmitInterval))), inBuffer, inBufferSize);

    // Check if transmission was successful
    if (err == ISBD_SUCCESS) {
      retransmitCounter = 0;
      memset(transmitBuffer, 0x00, sizeof(transmitBuffer)); // Clear transmit buffer array

      // Check for incoming message. If no inbound message is available, inBufferSize will be zero
      if (inBufferSize > 0) {

        // Print inBuffer size and values of each incoming byte of data
        Serial.print(F("Inbound buffer size is: ")); Serial.println(inBufferSize);
        for (uint8_t i = 0; i < inBufferSize; i++) {
          Serial.print(F("Address: "));
          Serial.print(i);
          Serial.print(F("\tValue: "));
          Serial.println(inBuffer[i], HEX);
        }

        // Recompose bits using bitshift
        uint8_t  resetFlagBuffer             = (((uint8_t)inBuffer[8] << 0) & 0xFF);
        uint16_t maxRetransmitCounterBuffer = (((uint16_t)inBuffer[7] << 0) & 0xFF) +
                                              (((uint16_t)inBuffer[6] << 8) & 0xFFFF);
        uint16_t transmitIntervalBuffer     = (((uint16_t)inBuffer[5] << 0) & 0xFF) +
                                              (((uint16_t)inBuffer[4] << 8) & 0xFFFF);
        uint32_t alarmIntervalBuffer        = (((uint32_t)inBuffer[3] << 0) & 0xFF) +
                                              (((uint32_t)inBuffer[2] << 8) & 0xFFFF) +
                                              (((uint32_t)inBuffer[1] << 16) & 0xFFFFFF) +
                                              (((uint32_t)inBuffer[0] << 24) & 0xFFFFFFFF);

        // Check if incoming data is valid
        if ((alarmIntervalBuffer        >= 300  && alarmIntervalBuffer        <= 1209600) &&
            (transmitIntervalBuffer     >= 1    && transmitIntervalBuffer     <= 24) &&
            (maxRetransmitCounterBuffer >= 0    && maxRetransmitCounterBuffer <= 24) &&
            (resetFlagBuffer            == 0    || resetFlagBuffer            == 255)) {

          // Update variables
          alarmInterval         = alarmIntervalBuffer;        // Update alarm interval
          transmitInterval      = transmitIntervalBuffer;     // Update transmit interval
          maxRetransmitCounter  = maxRetransmitCounterBuffer; // Update max retransmit counter
          resetFlag             = resetFlagBuffer;            // Update force reset flag
        }
      }

    }
    else {
      Serial.print(F("Transmission failed: error "));
      Serial.println(err);
    }
  }
  else {
    Serial.print(F("Begin failed: error "));
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED) {
      Serial.println(F("Warning: No modem detected. Please check wiring."));
    }
    return;
  }

  // If transmission or modem begin fails
  if (err != ISBD_SUCCESS) {
    retransmitCounter++;

    // Reset counter if retransmit counter is exceeded
    if (retransmitCounter == maxRetransmitCounter) {
      retransmitCounter = 0;
      memset(transmitBuffer, 0x00, sizeof(transmitBuffer));   // Clear transmitBuffer array
    }
  }

  // Power down the modem
  Serial.println(F("Putting the Qwiic 9603N to sleep."));
  err = modem.sleep();
  if (err != ISBD_SUCCESS) {
    Serial.print(F("Sleep failed: error "));
    Serial.println(err);
  }

  // Disable 9603N power
  Serial.println(F("Disabling 9603N power..."));
  modem.enable9603Npower(false);

  // Disable the supercapacitor charger
  Serial.println(F("Disabling the supercapacitor charger..."));
  modem.enableSuperCapCharger(false);

  // Stop loop timer
  uint32_t loopEndTime = millis() - loopStartTime;
  Serial.print(F("transmitData() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));

  // Write data to union
  message.transmitDuration = loopEndTime / 1000;

  Serial.print(F("transmitDuration: ")); Serial.println(loopEndTime / 1000);
  Serial.print(F("retransmitCounter: ")); Serial.println(retransmitCounter);

  // Print inBuffer variables
  Serial.print(F("alarmInterval: ")); Serial.println(alarmInterval);
  Serial.print(F("transmitInterval: ")); Serial.println(transmitInterval);
  Serial.print(F("maxRetransmitCounter: ")); Serial.println(maxRetransmitCounter);
  Serial.print(F("resetFlag: ")); Serial.println(resetFlag);

  // Check if reset flag was transmitted
  if (resetFlag == 255) {
    Serial.println(F("System reset..."));
    digitalWrite(LED_BUILTIN, HIGH);  // Turn on LED
    while (1);                        // Wait for Watchdog Timer to reset system
  }
}

// Blink LED (non-blocking)
void blinkLed(uint8_t flashes, uint32_t interval) {
  uint8_t i = 0;
  while (i <= flashes) {
    uint32_t currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      if (ledState == LOW) {
        ledState = HIGH;
      }
      else {
        ledState = LOW;
      }
      digitalWrite(LED_BUILTIN, ledState);
      i++;
    }
  }
}

// Blink LED
void blinkLED(unsigned long interval) {
  digitalWrite(LED_BUILTIN, (millis() / interval) % 2 == 1 ? HIGH : LOW);
}

// RockBLOCK callback function
bool ISBDCallback() {
  petDog(); // Pet the Watchdog
  blinkLED(500);
  return true;
}

// Callback to sniff the conversation with the Iridium modem
void ISBDConsoleCallback(IridiumSBD * device, char c) {
#if DIAGNOSTICS
  Serial.write(c);
#endif
}

// Callback to to monitor Iridium modem library's run state
void ISBDDiagsCallback(IridiumSBD * device, char c) {
#if DIAGNOSTICS
  Serial.write(c);
#endif
}

// Print union/structure
void printUnion() {
  Serial.println(F("-----------------------------------"));
  Serial.println(F("Union/structure"));
  Serial.println(F("-----------------------------------"));
  Serial.print(F("unixtime:\t\t")); Serial.println(message.unixtime);
  Serial.print(F("temperature:\t\t")); Serial.println(message.temperature);
  Serial.print(F("pressure:\t\t")); Serial.println(message.pressure);
  Serial.print(F("humidity:\t\t")); Serial.println(message.humidity);
  //Serial.print(F("pitch:\t\t\t")); Serial.println(message.pitch);
  //Serial.print(F("roll:\t\t\t")); Serial.println(message.roll);
  //Serial.print(F("heading:\t\t")); Serial.println(message.heading);
  Serial.print(F("latitude:\t\t")); Serial.println(message.latitude);
  Serial.print(F("longitude:\t\t")); Serial.println(message.longitude);
  Serial.print(F("satellites:\t\t")); Serial.println(message.satellites);
  Serial.print(F("pdop:\t\t\t")); Serial.println(message.pdop);
  Serial.print(F("fix:\t\t\t")); Serial.println(message.fix);
  //Serial.print(F("voltage:\t\t")); Serial.println(message.voltage);
  Serial.print(F("transmitDuration:\t")); Serial.println(message.transmitDuration);
  Serial.print(F("messageCounter:\t\t")); Serial.println(message.messageCounter);
  Serial.println(F("-----------------------------------"));
}

// Print contents of union/structure
void printUnionBinary() {
  Serial.println(F("Union/structure "));
  Serial.println(F("-----------------------------------"));
  Serial.println(F("Byte\tHex\tBinary"));
  for (uint16_t i = 0; i < sizeof(message); ++i) {
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
  for (uint16_t i = 0; i < sizeof(transmitBuffer); i++) {
    Serial.print(i);
    Serial.print(F("\t"));
    Serial.print(transmitBuffer[i], HEX);
    Serial.print("\t");
    Serial.println(transmitBuffer[i], BIN);
  }
}

// Configure the WDT to perform a system reset if loop() blocks for more than 8-16 seconds
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

  WDT->EWCTRL.bit.EWOFFSET = 0xA;                 // Set the Early Warning Interrupt Time Offset to 8 seconds // REG_WDT_EWCTRL = WDT_EWCTRL_EWOFFSET_8K;
  WDT->INTENSET.bit.EW = 1;                       // Enable the Early Warning interrupt                       // REG_WDT_INTENSET = WDT_INTENSET_EW;
  WDT->CONFIG.bit.PER = 0xB;                      // Set the WDT reset timeout to 16 seconds                  // REG_WDT_CONFIG = WDT_CONFIG_PER_16K;
  WDT->CTRL.bit.ENABLE = 1;                       // Enable the WDT in normal mode                            // REG_WDT_CTRL = WDT_CTRL_ENABLE;
  while (WDT->STATUS.bit.SYNCBUSY);               // Await synchronization of registers between clock domains

  // Configure and enable WDT interrupt
  NVIC_DisableIRQ(WDT_IRQn);
  NVIC_ClearPendingIRQ(WDT_IRQn);
  NVIC_SetPriority(WDT_IRQn, 0);                  // Top priority
  NVIC_EnableIRQ(WDT_IRQn);
}

// Pet the Watchdog Timer
void petDog() {
  WDT->CLEAR.bit.CLEAR = 0xA5;        // Clear the Watchdog Timer and restart time-out period //REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;
  while (WDT->STATUS.bit.SYNCBUSY);   // Await synchronization of registers between clock domains
}

// Watchdog Timer interrupt service routine
void WDT_Handler() {
  if (sleepFlag) {
    sleepFlag = false;
    WDT->INTFLAG.bit.EW = 1;          // Clear the Early Warning interrupt flag //REG_WDT_INTFLAG = WDT_INTFLAG_EW;
    WDT->CLEAR.bit.CLEAR = 0xA5;      // Clear the Watchdog Timer and restart time-out period //REG_WDT_CLEAR = WDT_CLEAR_CLEAR_KEY;
    while (WDT->STATUS.bit.SYNCBUSY); // Await synchronization of registers between clock domains
  }
  else {
    //WDT->CTRL.bit.ENABLE = 0;         // For debugging only: Disable Watchdog
    //digitalWrite(LED_BUILTIN, HIGH);  // For debugging only: Turn on LED to indicate Watchdog trigger
    while (true);                     // Force Watchdog Timer to reset the system
  }
}
