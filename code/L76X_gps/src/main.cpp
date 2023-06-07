#include <Arduino.h>
#include <sstream>
#include <string>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define SERIAL_BAUD 115200

#define GPS_BAUD 9600
#define GPS_RX 17
#define GPS_TX 16

#define TASK_SERIAL_RATE 10 * 1000

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5

HardwareSerial SerialGPS(1);
TinyGPSPlus gps;
uint32_t nextSerialTaskTs = 0;
RTC_DATA_ATTR int bootCount = 0;

void logGps()
{
  Serial.println("Logging GPS");
  char timestamp[17];
  sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02dZ",
          gps.date.year(),
          gps.date.month(),
          gps.date.day(),
          gps.time.hour(),
          gps.time.minute(),
          gps.time.second());
  Serial.println("Timestamp created");
  std::stringstream logLine;
  logLine << timestamp << ","
          << String(gps.location.lat(), 8).c_str() << ","
          << String(gps.location.lng(), 8).c_str() << ","
          << gps.altitude.meters() << ","
          << String(gps.hdop.hdop(), 2).c_str() << ","
          << gps.satellites.value() << "\r\n";
  Serial.println("Line created");
  Serial.println(logLine.str().c_str());
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);  

  
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

void loop()
{
  while (SerialGPS.available() > 0)
  {
    //Serial.println("Loop");
    char c = SerialGPS.read();
    gps.encode(c);
    //Serial.println(c);
  }

  //Serial.println(gps.satellites.value());
  if (nextSerialTaskTs < millis() && gps.satellites.value() > 3)
  {
    nextSerialTaskTs = millis() + TASK_SERIAL_RATE;
    logGps();

    if(gps.hdop.hdop() < 2){
      Serial.println("Going to sleep now");
      delay(1000);
      Serial.flush(); 
      esp_deep_sleep_start();
    }
  }
}