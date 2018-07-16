/*
   Copyright 2017, Helium Systems, Inc.
   All Rights Reserved. See LICENCE.txt for license information
*/

#include "Arduino.h"
#include "Board.h"
#include "Helium.h"
#include "HeliumUtil.h"
// Need it for the serial port 2
#include "wiring_private.h" // pinPeripheral() function
// need it for the I2C sensor
#include <Wire.h>
#include "HDC1080JS.h"
#include <Adafruit_MPL3115A2.h>
#include "ArduinoJson.h"
#include <RTCZero.h>
#include "Time.h"
#include "TimeLib.h"
#include "Timezone.h"
#include "Adafruit_VEML6070.h"

#define TimeStampSize   40

/* Create an rtc object */
RTCZero rtc;
time_t time_epoch;
time_t eastern, utc;
TimeChangeRule EDT = { "EDT", Second, Sun, Mar, 2, -240 };    //Daylight time = UTC - 4 hours
TimeChangeRule EST = { "EST", First, Sun, Nov, 2, -300 };     //Standard time = UTC - 5 hours
Timezone Eastern(EDT, EST);
TimeChangeRule CDT = { "CDT", Second, Sun, Mar, 2, -300 };    //Daylight time = UTC - 5 hours
TimeChangeRule CST = { "CST", First, Sun, Nov, 2, -360 };     //Standard time = UTC - 6 hours
Timezone Central(CDT, CST);
TimeChangeRule MDT = { "MDT", Second, Sun, Mar, 2, -360 };    //Daylight time = UTC - 6 hours
TimeChangeRule MST = { "MST", First, Sun, Nov, 2, -420 };     //Standard time = UTC - 7 hours
Timezone Mountain(MDT, MST);
TimeChangeRule PDT = { "PDT", Second, Sun, Mar, 2, -420 };    //Daylight time = UTC - 7 hours
TimeChangeRule PST = { "PST", First, Sun, Nov, 2, -480 };     //Standard time = UTC - 8 hours
Timezone Pacific(PDT, PST);
Timezone* timezones[] = { &Eastern, &Central, &Mountain, &Pacific };
String confTZnames[] = { "Eastern", "Central", "Mountain", "Pacific" };
Timezone* tz;                   //pointer to the time zone
uint8_t tzIndex;                //indexes the timezones[] array
TimeChangeRule* tcr;            //pointer to the time change rule, use to get TZ abbrev
char  TimeStampBuf[TimeStampSize];

/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 0;
const byte hours = 19;

/* Change these values to set the current initial date */
const byte tday = 11;
const byte tmonth = 7;
const byte tyear = 18;

#define ATOM
// NOTE: Please ensure you've created a channel with the above
// CHANNEL_NAME as it's name.
#define CHANNEL_NAME "Helium MQTT"
#define CONFIG_INTERVAL_KEY "config.interval_ms"
#define CONFIG_TIMEZONE_KEY "config.time_zone"

#define ERROR_PT100  -1000.0

//#define PT100
#define RS485
//#define ONEWIRE
#define UV_SENSOR

#define DEBUG_SERIAL    // This will be undefined if the interface is not defined

#ifdef PT100
#if  defined(RS485)
#error "cannot use RS485 and PT100 - share I/O terminal block conflict"
#elif defined(ONEWIRE)
#error "cannot use ONEWIRE and PT100 - share I/O terminal block conflict"
#elif defined(ATOM)
#undef DEBUG_SERIAL
#endif
#endif

#ifdef RS485
#ifdef ONEWIRE
#error "cannot use RS485 and ONEWIRE - share I/O terminal block conflict"
#endif
#endif

#ifdef UV_SENSOR
Adafruit_VEML6070 uv = Adafruit_VEML6070();
#endif

#ifdef ATOM
Helium  helium(&atom_serial);
Channel channel(&helium);
Channel GoogleChannel(&helium);
Config  config(&channel);
struct helium_info info;
#endif

bool baro_stat = false;
HDC1080JS tempsensor;
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

const int rs485en = 4;  // Digital Pin 2 control the RS485 EN - /RE pins

// this have been move to arduino variant.cpp since we need it in the HeliumUtils.cpp
//#ifdef RS485
//Uart Serial2 (&sercom2, 3, 2, SERCOM_RX_PAD_1, UART_TX_PAD_2);
//void SERCOM2_Handler()
//{
//  Serial2.IrqHandler();
//}
//#endif

#ifdef DEBUG_SERIAL
#ifdef RS485
Uart *debugSerial = &Serial2;
//#elif not defined(ATOM)
//Uart *debugSerial = &Serial1;
#endif
#endif


// SENSOR VARIABLES
char tzone[20];
int32_t send_interval = 10000;
double altm;
double pascals;
double humid;
double temp;

#ifdef PT100
float rtd_temp;
#endif



#ifdef ATOM
int gstatus;

// Introducing these here as a workaround for hetting float into json string here
// https://github.com/bblanchon/ArduinoJson/issues/566
// rounds a number to 2 decimal places
// example: round(3.14159) -> 3.14
double round2(double value) {
  return round(value * 100) / 100.0;
}

int report_status_result(int status, int result)
{
  if (helium_status_OK == status)
  {
    if (result == 0)
    {
      debugSerial->println("Succeeded");
    }
    else {
      debugSerial->println("Failed - ");
      debugSerial->println(result);
    }
  }
  else
  {
    debugSerial->println("Failed");
  }
  return status;
}

void update_config(bool stale)
{
  if (stale)
  {
    debugSerial->print("Fetching Config - ");
    debugSerial->println(CONFIG_INTERVAL_KEY);
    int status = config.get(CONFIG_INTERVAL_KEY, &send_interval, 60000);    // 60 seconds default
    report_status(status);
    debugSerial->print("Value: ");
    debugSerial->println(send_interval);
    debugSerial->print("Fetching Config - ");
    debugSerial->println(CONFIG_TIMEZONE_KEY);
    memset(tzone, 0, sizeof(tzone));
    status = config.get(CONFIG_TIMEZONE_KEY, tzone, sizeof(tzone), (char*)"Eastern");
    report_status(status);
    debugSerial->print("Value: ");
    debugSerial->println(tzone);

    if (status == helium_status_OK)
    {
      // CONFIG VALUE INTERVAL
      debugSerial->println("Updating Config - ");
      debugSerial->print(CONFIG_INTERVAL_KEY);
      debugSerial->print(" : ");
      debugSerial->println(send_interval);

      status = config.set(CONFIG_INTERVAL_KEY, send_interval);
      report_status(status);
    }
  }
}
#endif

#ifdef PT100
float get_pt100_temperature() {
  int sensorValue;
  float temperature = ERROR_PT100;      // error condition
  float current;
  float voltage;
  String err;
  // read the analog voltage of current sense amplifier
  analogReadResolution(12);
  sensorValue = analogRead(A1);
  voltage = sensorValue * (3.29 / 4095.0);    // 1 OHM Shunt gives a Full Scale of ma units
  current = sensorValue * (32.9 / 4095.0);    // 1 OHM Shunt gives a Full Scale of ma units
  if (current < 4.0) {
    err = String("ERR: Open Sensor");
  } else if (current > 20.0) {
    err = String("ERR: Overcurrent Condition");
  } else {
    current = current - 4.0;
    temperature = -50.0 + 12.50 * current;  // 200C/16mA is the slope
    String tmp_string = String(temperature);
    err = String("PT100 Temp: " + tmp_string + " C");
  }
#ifdef DEBUG_SERIAL
  debugSerial->println(err);
#endif
  return temperature;
}
#endif

void print2digits(int number) {
  if (number < 10) {
    debugSerial->print("0"); // print a 0 before if the number is < than 10
  }
  debugSerial->print(number);
}

void
setup()
{
  // GREEN LED
  pinMode(26, OUTPUT);
  digitalWrite(26, LOW);    // turn the LED on

#ifdef RS485
  // RS-485 DE - /RE Control Pin
  pinMode(rs485en, OUTPUT);
  digitalWrite(rs485en, HIGH);   // enable the driver
  // Assign pins 3 & 4 SERCOM functionality
  pinPeripheral(2, PIO_SERCOM);       // TX on PA14
  pinPeripheral(3, PIO_SERCOM_ALT);   // RX on PA9
  delay(100);
#endif

#ifdef DEBUG_SERIAL
  debugSerial->begin(9600);
  debugSerial->println("starting helium sensor node");
#endif

  if (! baro.begin()) {
#ifdef DEBUG_SERIAL
    debugSerial->println("Could not find sensor");
#endif
    baro_stat = false;
  }
  else {
#ifdef DEBUG_SERIAL
    debugSerial->println("Found I2C sensor");
#endif
    baro_stat = true;
  }

  tempsensor = HDC1080JS();
  tempsensor.config();

#ifdef UV_SENSOR
uv.begin(VEML6070_1_T);  // pass in the integration time constant
#endif

#ifdef ATOM
  // Begin communication with the Helium Atom
  // The baud rate differs per supported board
  // and is configured in Board.h
  helium.begin(HELIUM_BAUD_RATE);

  // Connect the Atom to the Helium Network
  debugSerial->println("Connecting - ");
  helium_connect(&helium);

  // Begin communicating with the channel. This should only need to
  // be done once. The HeliumUtil functions add simple retry logic
  // to re-create a channel if it disconnects.
  channel_create(&channel, CHANNEL_NAME);

  // Get the initial interval
  update_config(true);

  for (tzIndex = 0; tzIndex < sizeof(confTZnames); tzIndex++) {
    if (confTZnames[tzIndex].equals(String(tzone))) {
      break;
    }
  }
  if (tzIndex > sizeof(confTZnames)) {
    tzIndex = 0;
  }
  // Now tz has the Timezone accordingly to the configuraed value for the Atom Node
  tz = timezones[tzIndex];

  // Configure RTC as per the Network Epoch Value
  helium.info(&info);
  time_epoch = (time_t)info.time;
  eastern = (*tz).toLocal(time_epoch);
  TimeElements tm;
  breakTime(eastern, tm);

  rtc.begin(); // initialize RTC

  // Set the time
  debugSerial->print("epoch: ");
  debugSerial->println(time_epoch);

  rtc.setHours(tm.Hour);
  rtc.setMinutes(tm.Minute);
  rtc.setSeconds(tm.Second);

  // Set the date
  rtc.setDay(tm.Day);
  rtc.setMonth(tm.Month);
  rtc.setYear(tm.Year - 30);

  // Print date in Local Time
  print2digits(rtc.getDay());
  debugSerial->print("/");
  print2digits(rtc.getMonth());
  debugSerial->print("/");
  print2digits(rtc.getYear() + 2000);
  debugSerial->print(" ");

  // ...and Local time
  print2digits(rtc.getHours());
  debugSerial->print(":");
  print2digits(rtc.getMinutes());
  debugSerial->print(":");
  print2digits(rtc.getSeconds());

  debugSerial->println();

  int8_t result;
  debugSerial->println("Creating MyGoogleChannel - ");
  gstatus = GoogleChannel.begin("MyGoogleChannel", &result);
  // Print status and result
  report_status_result(gstatus, result);


#endif
}

void
loop()
{
  if (baro_stat == true) {
    pascals = baro.getPressure();
    altm = baro.getAltitude();
    float tempC = baro.getTemperature();
#ifdef DEBUG_SERIAL
    debugSerial->print(pascals / 3377); debugSerial->println(" Inches (Hg)");
    debugSerial->print(altm); debugSerial->println(" meters");
    debugSerial->print(tempC); debugSerial->println("*C");
#endif
  }

  tempsensor.readTempHumid();
  temp = tempsensor.getTemp();
  humid = tempsensor.getRelativeHumidity();

#ifdef PT100
  rtd_temp = get_pt100_temperature();
#endif

#ifdef UV_SENSOR
  uint16_t uv_level = uv.readUV();
  debugSerial->print("UV light level: "); debugSerial->println(uv_level);
#endif

  //#ifdef DEBUG_SERIAL
  //  debugSerial->print("T=");
  //  debugSerial->print(temp);
  //  debugSerial->print("C, RH=");
  //  debugSerial->print(humid);
  //  debugSerial->println("%");
  //#endif

#ifdef ATOM
  size_t used;
  int8_t result;

  // This requires beta version of JsonLibrary
  DynamicJsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  root[F("interval")] = send_interval;
  char buffer[HELIUM_MAX_DATA_SIZE];
  used = serializeJson(doc, buffer);
  // Send data to channel
  channel.send(buffer, strlen(buffer), &result);
  // Print status and result
  update_config(config.is_stale());

  if (gstatus == helium_status_OK) {
    // Get the TimeStamp Value
    snprintf(TimeStampBuf, TimeStampSize, "%d-%02d-%02d %02d:%02d:%02d", rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
    // Send some data to the configured Google channel
    DynamicJsonDocument gdoc;
    JsonObject groot = gdoc.to<JsonObject>();
    groot[F("RH")] = lround(humid);           // Humidity
    groot[F("Ti")] = round2(temp);            // Internal Temperature
    groot[F("ALT")] = lround(altm);           // Altitude in meters
    groot[F("PHg")] = round2(pascals / 3377); // Pressure in Hg
    groot[F("Te")] = round2(temp);            // Internal Temperature
    groot[F("DT")] = TimeStampBuf;            // Datetime Value
    groot[F("LS")] = uv_level;                // Light Sensor Value

    char gbuffer[HELIUM_MAX_DATA_SIZE];
    memset(gbuffer, 0, sizeof(gbuffer));
    used = serializeJson(gdoc, gbuffer);
#ifdef DEBUG_SERIAL
    debugSerial->println("Sending - ");
    debugSerial->println(gbuffer);
#endif
    int status = GoogleChannel.send(gbuffer, strlen(gbuffer), &result);
    // Print status and result
    report_status_result(status, result);
  }

#endif

  // Wait the configured interval in config Helium device
  long wait_time = send_interval;
  while(wait_time > 0){
    digitalWrite(26, HIGH);       // turn the LED off (HIGH is the voltage level)
    delay(1000);     // wait for half the interval
    digitalWrite(26, LOW);        // turn the LED on by making the voltage LOW
    delay(1000);     // wait for half the interval
    wait_time = wait_time - 2000;
  }
  
}


