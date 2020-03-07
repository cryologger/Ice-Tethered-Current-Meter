/*
  Title:    Cryologger - Ice Tethered Current Meter v1
  Date:     March 7, 2020
  Author:   Adam Garbo

    Components:
    - Adafruit Feather M0 Proto
    - Adafruit DS3231 RTC Precision Featherwing
    - Adafruit Ultimate GPS Featherwing
    - Adafruit Triple-axis Accelerometer + Magnetometer Board - LSM303
    - Rock Seven RockBLOCK 9603
    - Maxtena M1621HCT-P-SMA Iridium antenna

  Comments:
  - Code is currently under development
*/

// Libraries
#include <Arduino.h>                // https://github.com/arduino/ArduinoCore-samd (required before wiring_private.h)
#include <ArduinoLowPower.h>        // https://github.com/arduino-libraries/ArduinoLowPower
#include <DS3232RTC.h>              // https://github.com/JChristensen/DS3232RTC
#include <IridiumSBD.h>             // https://github.com/PaulZC/IridiumSBD
//#include <SAMD_AnalogCorrection.h>  // https://github.com/arduino/ArduinoCore-samd/tree/master/libraries/SAMD_AnalogCorrection
//#include <SparkFunLSM9DS1.h>        // https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library
#include <LSM303.h>                 // https://github.com/pololu/lsm303-arduino
#include <Statistic.h>              // https://github.com/RobTillaart/Arduino/tree/master/libraries/Statistic
#include <TinyGPS++.h>              // https://github.com/mikalhart/TinyGPSPlus
#include <wiring_private.h>         // https://github.com/arduino/ArduinoCore-samd/blob/master/cores/arduino/wiring_private.h (required for pinPeripheral() function)

// Pin constants
#define GPS_EN_PIN          A5  // GPS enable pin
#define RTC_INT_PIN         5   // RTC interrupt pin
#define ROCKBLOCK_RX_PIN    10  // RockBLOCK 9603 RX pin
#define ROCKBLOCK_TX_PIN    11  // RockBLOCK 9603 TX pin
#define ROCKBLOCK_SLEEP_PIN 12  // RockBLOCK 9603 sleep pin
#define VBAT_PIN            A7  // Battery voltage measurement pin

// Debugging constants
#define DEBUGGING   false // Output debugging messages to Serial Monitor
#define DIAGNOSTICS true  // Output Iridium diagnostic messages to Serial Monitor
#define DEPLOYED    true  // Disable debugging messages for deployment

// Create a new Serial/UART instance, assigning it to pins 10 and 11
// For more information see: https://www.arduino.cc/en/Tutorial/SamdSercom
Uart Serial2 (&sercom1, ROCKBLOCK_RX_PIN, ROCKBLOCK_TX_PIN, SERCOM_RX_PAD_2, UART_TX_PAD_0);
#define IridiumSerial Serial2
#define GpsSerial     Serial1

// Attach the interrupt handler to the SERCOM
void SERCOM1_Handler() {
  Serial2.IrqHandler();
}

// Object instantiations
DS3232RTC   rtc(false); // I2C Address: 0x68
IridiumSBD  modem(IridiumSerial, ROCKBLOCK_SLEEP_PIN);
//LSM9DS1     imu; // I2C Address: 0x1E (Magnetometer), 0x6B (Accelerometer)
LSM303      imu;
TinyGPSPlus gps;

// User-defined global variable declarations
uint32_t  alarmInterval         = 1800;  // RTC sleep duration in seconds (Default: 3600 seconds)
uint16_t  sampleInterval        = 120;    // Sampling duration of current tilt measurements (seconds)
uint8_t   sampleFrequency       = 1;    // Sampling frequency of current tilt measurements (seconds)
uint8_t   transmitInterval      = 1;    // Number of messages in each Iridium transmission (340-byte limit)
uint8_t   maxRetransmitCounter  = 4;    // Number of failed messages to reattempt in each Iridium transmission (340-byte limit)

// Global variable and constant declarations
bool          ledState            = LOW;    // Flag to toggle LED in blinkLed() function
volatile bool sleepFlag           = false;  // Flag to indicate to Watchdog Timer if in deep sleep mode
volatile bool alarmFlag           = false;  // Flag for alarm interrupt service routine
uint8_t       resetFlag           = 0;      // Flag to force Watchdog Timer system reset
uint8_t       transmitBuffer[340] = {};     // RockBLOCK 9603 transmission buffer
uint16_t      messageCounter      = 0;      // RockBLOCK 9603 transmitted message counter
uint16_t      retransmitCounter   = 0;      // RockBLOCK 9603 failed data transmission counter
uint16_t      transmitCounter     = 0;      // RockBLOCK 9603 transmission interval counter
uint32_t      previousMillis      = 0;      // Global millis() timer variable
time_t        alarmTime           = 0;      // UNIX Epoch alarm time variable
time_t        t                   = 0;      // UNIX Epoch time variable
tmElements_t  tm;                           // tmElements_t time structure

// Statistics objects
Statistic pitchStats;
Statistic rollStats;
Statistic headingStats;
Statistic batteryStats;
Statistic axStats;
Statistic ayStats;
Statistic azStats;
Statistic mxStats;
Statistic myStats;
Statistic mzStats;
Statistic gxStats;
Statistic gyStats;
Statistic gzStats;

// Structure and union to store and send data byte-by-byte via RockBLOCK
typedef union {
  struct {
    uint32_t  unixtime;           // Unix epoch time                (4 bytes)
    int16_t   temperature;        // Temperature                    (2 bytes)
    int16_t   pitch;              // Pitch                          (2 bytes)
    int16_t   roll;               // Roll                           (2 bytes)
    uint16_t  heading;            // Heading                        (2 bytes)
    /*
      int16_t   axMean;             // Accelerometer x                (2 bytes)
      int16_t   ayMean;             // Accelerometer y                (2 bytes)
      int16_t   azMean;             // Accelerometer z                (2 bytes)
      int16_t   mxMean;             // Magnetometer x                 (2 bytes)
      int16_t   myMean;             // Magnetometer y                 (2 bytes)
      int16_t   mzMean;             // Magnetometer z                 (2 bytes)
      int16_t   gxMean;             // Gyroscope x                    (2 bytes)
      int16_t   gyMean;             // Gyroscope y                    (2 bytes)
      int16_t   gzMean;             // Gyroscope z                    (2 bytes)
      int16_t   axStdev;            // Accelerometer x                (2 bytes)
      int16_t   ayStdev;            // Accelerometer y                (2 bytes)
      int16_t   azStdev;            // Accelerometer z                (2 bytes)
      int16_t   mxStdev;            // Magnetometer x                 (2 bytes)
      int16_t   myStdev;            // Magnetometer y                 (2 bytes)
      int16_t   mzStdev;            // Magnetometer z                 (2 bytes)
      int16_t   gxStdev;            // Gyroscope x                    (2 bytes)
      int16_t   gyStdev;            // Gyroscope y                    (2 bytes)
      int16_t   gzStdev;            // Gyroscope z                    (2 bytes)
    */
    int32_t   latitude;           // Latitude                       (4 bytes)
    int32_t   longitude;          // Longitude                      (4 bytes)
    uint16_t  satellites;         // # of satellites                (2 bytes)
    uint16_t  hdop;               // HDOP                           (2 bytes)
    uint16_t  voltage;            // Battery voltage (mV)           (2 bytes)
    uint16_t  transmitDuration;   // Previous transmission duration (2 bytes)
    uint16_t  messageCounter;     // Message counter                (2 bytes)
  } __attribute__((packed));                                      // Total: 30 bytes
  uint8_t bytes[30]; // To do: Research changes to flexible arrays in structures
} SBDMESSAGE;

SBDMESSAGE message;
size_t messageSize = sizeof(message); // Size (in bytes) of data message to be transmitted

// Setup
void setup() {

  // Digital pin configuration
  pinMode(GPS_EN_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(VBAT_PIN, INPUT);
  digitalWrite(GPS_EN_PIN, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

  // Open serial port and set data rate to 115200 bps
  Serial.begin(115200);

#if DEBUGGING
  while (!Serial);  // Prevent execution of script until Serial Monitor is open
#else if DEPLOYED
  delay(10000);     // Delay to allow for opening of Serial Monitor
#endif

  // Configure Watchdog Timer
  configureWatchdog();

  Serial.println(F("Cryologger - Ice Tethered Current Meter v1"));
  Serial.println(F("------------------------------------------"));

  // Configure I2C
  Wire.begin();           // Initialize I2C bus
  Wire.setClock(100000);  // Set I2C clock speed to 100kHz

  // Analog-to-digital converter (ADC) Configuration
  analogReadResolution(12); // Change the ADC resolution to 12 bits
  //analogReadCorrection(15, 2056); // ADC offset and gain correction values

  // Adafruit DS3231 Precision RTC FeatherWing Configuration
  rtc.begin();
  rtc.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);  // Initialize alarm 1 to known values
  rtc.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);  // Initialize alarm 2 to known values
  rtc.alarm(ALARM_1);                         // Clear alarm 1 flag
  rtc.alarm(ALARM_2);                         // Clear alarm 2 flag
  rtc.alarmInterrupt(ALARM_1, false);         // Clear the alarm 1 interrupt flag
  rtc.alarmInterrupt(ALARM_2, false);         // Clear the alarm 2 interrupt flag
  rtc.squareWave(SQWAVE_NONE);                // Disable square wave output

  /*
    // Manually set the RTC time
    tm.Hour = 10;
    tm.Minute = 01;
    tm.Second = 40;
    tm.Day = 20;
    tm.Month = 7;
    tm.Year = 2020 - 1970; // tmElements_t.Year is the offset from 1970
    t = makeTime(tm); // Change  tm structure into time_t (UNIX Epoch time)
    rtc.set(t);
  */

  pinMode(RTC_INT_PIN, INPUT_PULLUP); // Enable pullup on interrupt pin (INT/SQW pin is open drain)
  LowPower.attachInterruptWakeup(RTC_INT_PIN, alarmIsr, FALLING); // Attach a wakeup interrupt on INT/SQW pin

#if DEBUGGING
  rtc.setAlarm(ALM1_MATCH_SECONDS, 0, 0, 0, 1);     // Set initial alarm to occur at seconds rollover
#else if DEPLOYED
  rtc.setAlarm(ALM1_MATCH_MINUTES, 0, 0, 0, 1);     // Set initial alarm to occur at minutes rollover
#endif
  rtc.alarm(ALARM_1);                               // Clear alarm 1 interrupt flag
  rtc.alarmInterrupt(ALARM_1, true);                // Enable interrupt output for alarm 1
  Serial.println(F("Adafruit DS3231 Precision RTC FeatherWing detected."));

  /*
    // SparkFun 9DoF Sensor Stick Configuration
    if (imu.begin() == true) {
      Serial.println(F("SparkFun 9DoF Sensor Stick detected."));
    }
    else {
      Serial.println(F("Warning: SparkFun 9DoF Sensor Stick not detected. Please check wiring."));
    }
  */
  // Adafruit LSM303DLHC Configuration
  if (imu.init()) {
    Serial.println(F("Adafruit LSM303DLHC detected."));
  }
  else {
    Serial.println(F("Warning: Adafruit LSM303DLHC not detected. Please check wiring."));
  }

  // Adafruit Ultimate GPS FeatherWing Configuration


  // RockBLOCK 9603 Configuration
  if (modem.isConnected()) {
    modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE); // Assume battery power
    modem.adjustSendReceiveTimeout(180); // Default = 300 seconds
    modem.adjustATTimeout(20);
    Serial.println(F("RockBLOCK 9603 detected."));
  }
  else {
    Serial.println(F("Warning: RockBLOCK 9603N not detected. Please check wiring."));
  }

  // Print current date and time
  Serial.print(F("Time: ")); printDatetime(rtc.get());

  // Print operating mode
  Serial.print(F("Mode: "));
#if DEBUGGING
  Serial.println(F("DEBUGGING"));
#else if DEPLOYED
  Serial.println(F("DEPLOYMENT"));
#endif

}

// Loop
void loop() {

  // Check if alarm interrupt service routine was triggered
  if (alarmFlag) {

    // Check to see if the alarm flag is set (resets the flag if set)
    if (rtc.alarm(ALARM_1)) {

      // Read the RTC
      readRtc();

      // Print ALARM_1 trigger date and time
      Serial.print(F("ALARM_1: ")); printDatetime(t);

      // Pet the Watchdog Timer
      petDog();

      // Read the battery voltage
      readBattery();

      // Perform current measurements for sampling duration (e.g. 2 minutes)
      uint32_t loopStartTime = millis();
      while (millis() - loopStartTime < sampleInterval * 1000UL) {
        petDog();         // Pet the Watchdog Timer
        readBattery();    // Read the battery voltage
        readImu();        // Read IMU
        //blinkLed(1, 100); // Blink LED
        LowPower.sleep(1000); // Go to sleep to reduce current draw
      }

      // Read GPS
      readGps();

      // Perform statistics on measurements
      printStatistics();
      calculateStatistics();

      // Write data to buffer
      writeBuffer();

      // Check if data should be transmitted
      if (transmitCounter == transmitInterval) {
        transmitData();       // Transmit data
        transmitCounter = 0;  // Reset transmit counter
      }

      // Set RTC alarm
      alarmTime = t + alarmInterval; // Calculate next alarm

      // Check if alarm is set in the past
      if (rtc.get() >= alarmTime) {
        t = rtc.get(); // Read current date and time
        alarmTime = t + alarmInterval; // Calculate next alarm
        Serial.println(F("Warning! Alarm set in the past."));
      }
      rtc.setAlarm(ALM1_MATCH_DATE, 0, minute(alarmTime), hour(alarmTime), day(alarmTime)); // Set alarm 1
      rtc.alarm(ALARM_1); // Clear alarm 1 interrupt flag
      Serial.print(F("Next alarm: ")); printDatetime(alarmTime);
    }
    alarmFlag = false; // Clear alarm interrupt service routine flag
  }
  sleepFlag = true; // Set Watchdog Timer sleep flag

#if DEBUGGING
  blinkLed(1, 500);
#endif

#if DEPLOYED
  blinkLed(1, 10);
  LowPower.deepSleep(); // Enter deep sleep
#endif
}

// Measure battery voltage from 100/100 kOhm voltage divider
void readBattery() {

  // Start loop timer
  uint32_t loopStartTime = millis();
  float voltage = 0.0;

  // Average measurements
  for (uint8_t i = 0; i < 10; ++i) {
    voltage += analogRead(VBAT_PIN);
    delay(1);
  }
  voltage /= 10;    // Average measurements
  voltage *= 2;     // Divided by 2, so multiply back
  voltage *= 3.3;   // Multiply by 3.3V reference voltage
  voltage /= 4096;  // Convert to voltage

  // Add to statistics object
  batteryStats.add(voltage);

  // Stop loop timer
  uint32_t loopEndTime = millis() - loopStartTime;
  Serial.print("readBattery() function execution: "); Serial.print(loopEndTime); Serial.println(F(" ms"));
}

// Read date and time and internal temperature from RTC
void readRtc() {

  // Start loop timer
  uint32_t loopStartTime = millis();

  // Read current date and time
  rtc.read(tm);

  // Change the tm structure into time_t (UNIX Epoch time)
  t = makeTime(tm);

  // Read RTC internal temperature
  float temperature = rtc.temperature() / 4.0;

  // Write data to union
  message.unixtime = t;
  message.temperature = temperature * 100;

  // Stop loop timer
  uint32_t loopEndTime = millis() - loopStartTime;
  Serial.print(F("readRtc() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));
}

// RTC alarm interrupt service routine (ISR)
void alarmIsr() {
  alarmFlag = true; // Set alarm flag
}

// Print date and time
void printDatetime(time_t t) {
  char datetimeBuffer[20];
  snprintf(datetimeBuffer, sizeof(datetimeBuffer), "%04u-%02d-%02dT%02d:%02d:%02d",
           year(t), month(t), day(t), hour(t), minute(t), second(t));
  Serial.println(datetimeBuffer);
}

/*
  // Read inertial measurement unit (IMU)
  void readImu() {

  // Start loop timer
  uint32_t loopStartTime = millis();

  float ax, ay, az, mx, my, mz, gx, gy, gz, pitch, roll, heading;

  // Insert IMU code here



  // Add to statistics object
    axStats.add(imu.ax);
    ayStats.add(imu.ay);
    azStats.add(imu.az);
    mxStats.add(imu.mx);
    myStats.add(imu.my);
    mzStats.add(imu.mz);
    gxStats.add(imu.gx);
    gyStats.add(imu.gy);
    gzStats.add(imu.gz);

  // Stop loop timer
  uint32_t loopEndTime = millis() - loopStartTime;
  Serial.print(F("readImu() executed in: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));
  }
*/

// Read pitch, roll and tilt-compensated heading from LSM303 (°)
void readImu()
{
  // Start loop timer
  uint32_t loopStartTime = millis();

  const float alpha = 0.10;   // Accelerometer low-pass filter alpha value

  imu.enableDefault(); // Turn on accelerometer and magnetometer
  /*
    Calibration values: the default values of +/-32767 for each axis lead to an assumed
    magnetometer bias of 0. Use the Pololu LSM303 library Calibrate example program to
    determine appropriate values for each LSM303 sensor.
  */

  imu.m_min = (LSM303::vector<int16_t>) {
    //-32767, -32767, -32767 // Default
    -3493, -3422, -3688   // Test unit
  };

  imu.m_max = (LSM303::vector<int16_t>) {
    //+32767, +32767, +32767 // Default
    +3090, +3434, +3665   // Test unit
  };

  // Apply low-pass filter to accelerometer data
  float fXa, fYa, fZa = 0.0;

  // Read and average accelerometer and magnetometer data
  for (byte i = 0; i < 30; i++) {
    imu.read();
    fXa = imu.a.x * alpha + (fXa * (1.0 - alpha));
    fYa = imu.a.y * alpha + (fYa * (1.0 - alpha));
    fZa = imu.a.z * alpha + (fZa * (1.0 - alpha));
    delay(1);
  }

  // Write registers to put accelerometer and magenteometer in power-down mode
  imu.writeAccReg(0x20, 0x07);
  imu.writeMagReg(0x26, 0x03);

  // Calculate orientation
  float pitch = (atan2(-fXa, sqrt((int32_t)fYa * fYa + (int32_t)fZa * fZa))) * 180 / PI;
  float roll = (atan2(fYa, fZa)) * 180 / PI;
  float heading = imu.heading((LSM303::vector<int>) {
    1, 0, 0   // PCB orientation
  });

  // Add to statistics object
  pitchStats.add(pitch);
  rollStats.add(roll);
  headingStats.add(heading);

  /*
    // Write orientation data to union
    message.pitch = pitch * 100;
    message.roll = roll * 100;
    message.heading = heading * 10;
  */

  uint32_t loopEndTime = millis() - loopStartTime;
  Serial.print("readLsm303() function execution: "); Serial.print(loopEndTime); Serial.println(F(" ms"));
}

// Read latitude, longitude, number of satellites and HDOP from GPS
void readGps() {

  // Start loop timer
  uint32_t loopStartTime = millis();

  bool fixFound = false;
  bool charsSeen = false;
  uint8_t fixCounter = 0;

  // Enable GPS
  digitalWrite(GPS_EN_PIN, LOW);

  Serial.println("Beginning to listen for GPS traffic...");
  GpsSerial.begin(9600);
  blinkLed(2, 500);

  // Configure GPS
  GpsSerial.println("$PMTK220,1000*1F");  // Set NMEA port update rate to 1Hz
  delay(100);
  GpsSerial.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");   // Set NMEA sentence output frequencies to GGA and RMC
  delay(100);
  //GpsSerial.println("$PGCMD,33,1*6C");  // Enable antenna updates
  GpsSerial.println("$PGCMD,33,0*6D");    // Disable antenna updates

  // Look for GPS signal for up to 2 minutes
  while (!fixFound && millis() - loopStartTime < 3UL * 60UL * 1000UL) {
    if (GpsSerial.available()) {
      charsSeen = true;
      char c = GpsSerial.read();
#if DEBUGGING
      Serial.write(c); // Echo NMEA sentences to serial
#endif
      if (gps.encode(c)) {
        if ((gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) &&
            (gps.location.isUpdated() && gps.date.isUpdated() && gps.time.isUpdated())) {
          fixCounter++;
          if (fixCounter == 10) {
            fixFound = true;

            message.latitude = gps.location.lat() * 1000000;
            message.longitude = gps.location.lng() * 1000000;
            message.satellites = gps.satellites.value();
            message.hdop = gps.hdop.value();

            time_t currentTime = rtc.get(); // Get current date and time

            // Sync the RTC with GPS time
            tm.Hour = gps.time.hour();
            tm.Minute = gps.time.minute();
            tm.Second = gps.time.second();
            tm.Day = gps.date.day();
            tm.Month = gps.date.month();
            tm.Year = gps.date.year() - 1970; // tmElements_t.Year is the offset from 1970
            time_t gpsTime = makeTime(tm);

            // Calculate drift of RTC (in seconds)
            int drift = gpsTime - currentTime;

            // Synchronize RTC time with GPS if drift exceeds threshold
            if (drift > 5 || drift < -5) {
              rtc.set(gpsTime);
              Serial.print("Current time: "); printDatetime(currentTime);
              Serial.print("Updated time: "); printDatetime(gpsTime);
            }
            Serial.print(F("Drift: ")); Serial.print(drift); Serial.println(F(" seconds"));
          }
        }
      }
    }

    ISBDCallback();

    if ((millis() - loopStartTime) > 5000 && gps.charsProcessed() < 10) {
      Serial.println(F("No GPS data received: check wiring"));
      break;
    }
  }
  Serial.println(charsSeen ? fixFound ? F("A GPS fix was found!") : F("No GPS fix was found.") : F("Wiring error: No GPS data seen."));

  // Stop loop timer
  uint32_t loopEndTime = millis() - loopStartTime;
  Serial.print(F("readGps() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));

  // Disable GPS
  digitalWrite(GPS_EN_PIN, HIGH);
}

// Calculate statistics and clear objects
void calculateStatistics() {

  // Write data to union
  /*
    message.axMean = axStats.average() * 100;
    message.ayMean = ayStats.average() * 100;
    message.azMean = azStats.average() * 100;
    message.mxMean = mxStats.average() * 100;
    message.myMean = myStats.average() * 100;
    message.mzMean = mzStats.average() * 100;
    message.gxMean = gxStats.average() * 100;
    message.gyMean = gyStats.average() * 100;
    message.gzMean = gzStats.average() * 100;
    message.axStdev = axStats.pop_stdev() * 100;
    message.ayStdev = ayStats.pop_stdev() * 100;
    message.azStdev = azStats.pop_stdev() * 100;
    message.mxStdev = mxStats.pop_stdev() * 100;
    message.myStdev = myStats.pop_stdev() * 100;
    message.mzStdev = mzStats.pop_stdev() * 100;
    message.gxStdev = gxStats.pop_stdev() * 100;
    message.gyStdev = gyStats.pop_stdev() * 100;
    message.gzStdev = gzStats.pop_stdev() * 100;
  */
  message.pitch = pitchStats.average() * 100;
  message.roll = rollStats.average() * 100;
  message.heading = headingStats.average()  * 10;
  message.voltage = batteryStats.average() * 1000;


  // Clear statistics objects
  axStats.clear();
  ayStats.clear();
  azStats.clear();
  mxStats.clear();
  myStats.clear();
  mzStats.clear();
  gxStats.clear();
  gyStats.clear();
  gzStats.clear();
  pitchStats.clear();
  rollStats.clear();
  headingStats.clear();
  batteryStats.clear();
}

// Write union data to transmit buffer in preparation of data transmission
void writeBuffer() {
  messageCounter++;                         // Increment message counter
  message.messageCounter = messageCounter;  // Write message counter data to union
  transmitCounter++;                        // Increment data transmission counter

  // Concatenate current message with existing message(s) stored in transmit buffer
  memcpy(transmitBuffer + (sizeof(message) * (transmitCounter + (retransmitCounter * transmitInterval) - 1)),
         message.bytes, sizeof(message)); // Copy message to transmit buffer

#if DEBUGGING
  printUnion();
  //printUnionBinary(); // Print union/structure in hex/binary
  //printTransmitBuffer();  // Print transmit buffer in hex/binary
#endif
}

// Transmit data via the RockBLOCK 9603 transceiver
void transmitData() {

  // Start loop timer
  uint32_t loopStartTime = millis();
  uint16_t err;

  // Start the serial power connected to the RockBLOCK modem
  IridiumSerial.begin(19200);

  // Assign pins 10 & 11 SERCOM functionality
  pinPeripheral(ROCKBLOCK_RX_PIN, PIO_SERCOM);
  pinPeripheral(ROCKBLOCK_TX_PIN, PIO_SERCOM);

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

      // Check for incoming message. inBufferSize = 0 if no inbound message is available
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
        uint8_t resetFlagBuffer             = (((uint8_t)inBuffer[8] << 0) & 0xFF);
        uint16_t maxRetransmitCounterBuffer = (((uint16_t)inBuffer[7] << 0) & 0xFF) +
                                              (((uint16_t)inBuffer[6] << 8) & 0xFFFF);
        uint16_t transmitIntervalBuffer     = (((uint16_t)inBuffer[5] << 0) & 0xFF) +
                                              (((uint16_t)inBuffer[4] << 8) & 0xFFFF);
        uint32_t alarmIntervalBuffer        = (((uint32_t)inBuffer[3] << 0) & 0xFF) +
                                              (((uint32_t)inBuffer[2] << 8) & 0xFFFF) +
                                              (((uint32_t)inBuffer[1] << 16) & 0xFFFFFF) +
                                              (((uint32_t)inBuffer[0] << 24) & 0xFFFFFFFF);

        // Check if incoming data is valid
        if ((alarmIntervalBuffer >= 300 && alarmIntervalBuffer <= 1209600) &&
            (transmitIntervalBuffer >= 1 && transmitIntervalBuffer <= 24) &&
            (maxRetransmitCounterBuffer >= 0 && maxRetransmitCounterBuffer <= 24) &&
            (resetFlagBuffer == 0  || resetFlagBuffer == 255)) {

          // Update global variables
          alarmInterval = alarmIntervalBuffer;                // Update alarm interval
          transmitInterval = transmitIntervalBuffer;          // Update transmit interval
          maxRetransmitCounter = maxRetransmitCounterBuffer;  // Update max retransmit counter
          resetFlag = resetFlagBuffer;                        // Update force reset flag

          Serial.print(F("alarmInterval: ")); Serial.println(alarmInterval);
          Serial.print(F("transmitInterval: ")); Serial.println(transmitInterval);
          Serial.print(F("maxRetransmitCounter: ")); Serial.println(maxRetransmitCounter);
          Serial.print(F("resetFlag: ")); Serial.println(resetFlag);
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
    if (retransmitCounter >= maxRetransmitCounter) {
      retransmitCounter = 0;
      memset(transmitBuffer, 0x00, sizeof(transmitBuffer));   // Clear transmitBuffer array
    }
  }

  // Put RockBLOCK 9603 to sleep
  Serial.println(F("Putting the RockBLOCK 9603 to sleep."));
  err = modem.sleep();
  if (err != ISBD_SUCCESS) {
    Serial.print(F("Sleep failed: error "));
    Serial.println(err);
  }

  // Close RockBLOCK 9603 serial port
  IridiumSerial.end();

  // Stop loop timer
  uint32_t loopEndTime = millis() - loopStartTime;
  Serial.print(F("transmitData() function execution: ")); Serial.print(loopEndTime); Serial.println(F(" ms"));

  // Write data to union
  message.transmitDuration = loopEndTime / 1000;

  Serial.print(F("retransmitCounter: ")); Serial.println(retransmitCounter);

  // Check if reset flag was transmitted
  if (resetFlag == 255) {
    while (1); // Wait for Watchdog Timer to reset system
  }

  // Print current date and time
  Serial.print(F("Current time: ")); printDatetime(rtc.get());
}

// Blink LED
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
void blinkLED(uint32_t interval) {
  digitalWrite(LED_BUILTIN, (millis() / interval) % 2 == 1 ? HIGH : LOW);
}

// RockBLOCK callback function
bool ISBDCallback() {
  petDog();         // Pet the Watchdog
  //readBattery();  // Read battery voltage
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

// Print statistics
void printStatistics() {
  Serial.println(F("-------------------------------------------------------------------------"));
  Serial.println(F("Statistics"));
  Serial.println(F("-------------------------------------------------------------------------"));
  Serial.println(F("Voltage:"));
  Serial.print(F("Samples: ")); Serial.print(batteryStats.count());
  Serial.print(F("\tMin: "));   Serial.print(batteryStats.minimum());
  Serial.print(F("\tMax: ")); Serial.print(batteryStats.maximum());
  Serial.print(F("\tMean: ")); Serial.print(batteryStats.average());
  Serial.print(F("\tSD: ")); Serial.println(batteryStats.pop_stdev());
}

// Print union/structure
void printUnion() {
  Serial.println(F("-----------------------------------"));
  Serial.println(F("Union/structure"));
  Serial.println(F("-----------------------------------"));
  Serial.print(F("unixtime:\t\t")); Serial.println(message.unixtime);
  Serial.print(F("temperature:\t\t")); Serial.println(message.temperature);
  Serial.print(F("pitch:\t\t\t")); Serial.println(message.pitch);
  Serial.print(F("roll:\t\t\t")); Serial.println(message.roll);
  Serial.print(F("heading:\t\t")); Serial.println(message.heading);
  /*
    Serial.print(F("ax:\t\t\t")); Serial.println(message.axMean);
    Serial.print(F("ay:\t\t\t")); Serial.println(message.ayMean);
    Serial.print(F("az:\t\t\t")); Serial.println(message.azMean);
    Serial.print(F("mx:\t\t\t")); Serial.println(message.mxMean);
    Serial.print(F("my:\t\t\t")); Serial.println(message.myMean);
    Serial.print(F("mz:\t\t\t")); Serial.println(message.mzMean);
    Serial.print(F("gx:\t\t\t")); Serial.println(message.gxMean);
    Serial.print(F("gy:\t\t\t")); Serial.println(message.gyMean);
    Serial.print(F("gz:\t\t\t")); Serial.println(message.gzMean);
  */
  Serial.print(F("latitude:\t\t")); Serial.println(message.latitude);
  Serial.print(F("longitude:\t\t")); Serial.println(message.longitude);
  Serial.print(F("satellites:\t\t")); Serial.println(message.satellites);
  Serial.print(F("hdop:\t\t\t")); Serial.println(message.hdop);
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
    while (true);                     // Force Watchdog Timer to reset the system
  }
}
