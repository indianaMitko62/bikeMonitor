#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <HardwareSerial.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_HMC5883_U.h>
#include <TinyGPS++.h>
#include "ThingSpeak.h"
#include <sstream>
#include <string>

#define SERIAL_BAUD 115200

#define SDA_PIN 21
#define SCL_PIN 22

#define MPU9250_ADDRESS 0x68
#define GYRO_FULL_SCALE_2000_DPS 0x18
#define ACC_FULL_SCALE_16_G 0x18

#define GPS_BAUD 9600
#define GPS_RX 17
#define GPS_TX 16

#define TASK_SERIAL_RATE 10 * 1000

#define uS_TO_S_FACTOR 1000000
#define TIME_TO_SLEEP  5
#define TIME_DIFFERENCE_H 3
#define LOCAL_SEA_LEVEL_PRESSURE 1018.25

#define GPS_READING_INTERVAL 10000
int lastGPSReading = 0;
int gps_reading = 0;

const char* ssid = "Indiana";
const char* password = "mitko123";
int SpeedChannelNumber = 1;
int AltitudeChannelNumber = 2;
int PowerChannelNumber = 3;
const char * myWriteAPIKey = "6QRJ1KHMQVXHH30T";
WiFiClient  client;

HardwareSerial SerialGPS(1);
TinyGPSPlus gps;
uint32_t nextSerialTaskTs = 0;
RTC_DATA_ATTR int bootCount = 0;

Adafruit_BMP280 bme;
int stat_bme = 0;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

char timestamp[17];
String logFile = "/track.csv";
std::stringstream logLineGPS;
std::stringstream logLineGY91;
std::stringstream logLineGY271;

struct CurrentData
{
	float distance;
	float speed;
	float power;
	float altitude;
	float lon;
	float lat;
} curr_data;

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
	Wire.beginTransmission(Address);
	Wire.write(Register);
	Wire.endTransmission();

	Wire.requestFrom(Address, Nbytes);
	uint8_t index = 0;

	while (Wire.available())
	{
		Data[index++] = Wire.read();
	}
}

void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
	Wire.beginTransmission(Address);
	Wire.write(Register);
	Wire.write(Data);
	Wire.endTransmission();
}

int readFile(fs::FS &fs, const char * path)
{
	File file = fs.open(path);
	if(!file)
	{
		Serial.println("Failed to open file");
		return 1;
	}
	Serial.print("Read from file: ");
	while(file.available())
	{
		Serial.write(file.read());
	}
	return 0;
	file.close();
}

int writeFile(fs::FS &fs, const char * path, const char * message)
{
	File file = fs.open(path, FILE_WRITE);
	if(!file)
	{
		Serial.println("Failed to open file");
		return 1;
	}
	if(file.print(message))
	{
		Serial.println("write successful");
	} 
	else 
	{
		Serial.println("write failed");
		return 1;
	}
	file.close();
	return 0;
}

int appendFile(fs::FS &fs, const char * path, const char * message)
{
	File file = fs.open(path, FILE_APPEND);
	if(!file)
	{
		Serial.println("Failed to open file");
		return 1;
	}
	if(file.print(message))
	{
		Serial.println("Message appended");
	} 
	else 
	{
		Serial.println("Append failed");
		return 1;
	}
	file.close();
	return 0;
}

int renameFile(fs::FS &fs, const char * path1, const char * path2)
{
	if (fs.rename(path1, path2))
	{
		Serial.println("File renamed");
		return 0;
	} 
	else 
	{
		Serial.println("Rename failed");
		return 1;
	}
}

int deleteFile(fs::FS &fs, const char * path)
{
if(fs.remove(path))
{
	Serial.println("File deleted");
	return 0;
} 
else 
{
	Serial.println("Delete failed");
	return 1;
}
}

void printSDInfo()
{
	uint8_t cardType = SD.cardType();

	if(cardType == CARD_NONE)
	{
		Serial.println("No SD card attached");
		return;
	}

	Serial.print("SD Card Type: ");
	if(cardType == CARD_MMC)
	{
		Serial.println("MMC");
	} 
	else if(cardType == CARD_SD)
	{
		Serial.println("SDSC");
	} 
	else if(cardType == CARD_SDHC)
	{
		Serial.println("SDHC");
	}
	else 
	{
		Serial.println("UNKNOWN");
	}

	uint64_t cardSize = SD.cardSize() / (1024 * 1024);
	Serial.printf("SD Card Size: %lluMB\n", cardSize);

}

void gy91_setup()
{
	I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);
	I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);

	stat_bme = bme.begin(0x76,0x58);
	Serial.print("BMP280 status init: ");
	Serial.println(stat_bme);
}

void gps_setup()
{
	SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);  
	++bootCount;
	Serial.println("Boot number: " + String(bootCount));
	esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}

void sd_setup()
{
	if(!SD.begin(5))
	{
		Serial.println("Card Mount Failed");
	}
}

void current_data_setup()
{
	curr_data.altitude = 0;
	curr_data.distance = 0;
	curr_data.power = 0;
	curr_data.speed = 0;	
}

void logGY91()
{
	Serial.println("Logging GY-91");

	uint8_t Buf[14];
	I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

	int16_t ax = -(Buf[0] << 8 | Buf[1]);
	int16_t ay = -(Buf[2] << 8 | Buf[3]);
	int16_t az = Buf[4] << 8 | Buf[5];

	int16_t gx = -(Buf[8] << 8 | Buf[9]);
	int16_t gy = -(Buf[10] << 8 | Buf[11]);
	int16_t gz = Buf[12] << 8 | Buf[13];
	logLineGY91.str("");
  	logLineGY91 << timestamp << ",GY91:,"
			<< ax << ","
			<< ay << ","
			<< az << ","
			<< gx << ","
			<< gy << ","
			<< gz << ","
			<< bme.readTemperature() << ","
			<< bme.readAltitude(LOCAL_SEA_LEVEL_PRESSURE) << ",";

	Serial.println(logLineGY91.str().c_str());
	//appendFile(SD, logFile.c_str(), logLineGY91.str().c_str());
}
void logGY271()
{
	Serial.println("Logging GY-271");

	sensors_event_t event;
    mag.getEvent(&event);
    float heading = atan2(event.magnetic.y, event.magnetic.x);
	if(heading < 0)
	{
		heading += 2*PI;
	}
    if(heading > 2*PI)
	{
		heading -= 2*PI;
	}
    float headingDegrees = heading * 180/PI;
	logLineGY271.str("");
  	logLineGY271 << timestamp << ",GY271:,"
			<< event.magnetic.x << ","
			<< event.magnetic.y << ","
			<< event.magnetic.z << ","
			<< headingDegrees << ",";

	Serial.println(logLineGY271.str().c_str());
	//appendFile(SD, logFile.c_str(), logLineGY271.str().c_str());
}

void logGPS()
{
	Serial.println("Logging GPS");
	sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02dZ",
			gps.date.year(),
			gps.date.month(),
			gps.date.day(),
			gps.time.hour() + TIME_DIFFERENCE_H,
			gps.time.minute(),
			gps.time.second());

	logLineGPS.str("");
	logLineGPS << timestamp << ",GPS:,"
			<< gps.course.deg() << ","
			<< String(gps.location.lat(), 8).c_str() << ","
			<< String(gps.location.lng(), 8).c_str() << ","
			<< gps.speed.kmph() << ","
			<< gps.altitude.meters() << ","
			<< String(gps.hdop.hdop(), 2).c_str() << ","
			<< gps.satellites.value() << ",";


	if(logFile == "")
	{
		logFile = "/" + gps.date.day() + (gps.time.hour() + TIME_DIFFERENCE_H) + gps.time.minute();
	}
	//appendFile(SD, logFile.c_str(), logLineGPS.str().c_str());
	Serial.println(logLineGPS.str().c_str());
}

float calculate_power()
{
	
}

float calculate_distance()
{
	float lat1 = curr_data.lat * PI / 180.0;
    float lon1 = curr_data.lon * PI / 180.0;
    float lat2 = gps.location.lat() * PI / 180.0;
    float lon2 = gps.location.lng() * PI / 180.0;

    double r = 6378100;

    double rho1 = r * cos(lat1);
    double z1 = r * sin(lat1);
    double x1 = rho1 * cos(lon1);
    double y1 = rho1 * sin(lon1);

    double rho2 = r * cos(lat2);
    double z2 = r * sin(lat2);
    double x2 = rho2 * cos(lon2);
    double y2 = rho2 * sin(lon2);

    double dot = (x1 * x2 + y1 * y2 + z1 * z2);
    double cos_theta = dot / (r * r);
    double theta = acos(cos_theta);
    return r * theta;
}

void update_current_data_by_GPS()
{
	if(gps_reading > 1)
	{
		curr_data.distance = curr_data.distance + calculate_distance();

	}
	curr_data.lon = gps.location.lng();
	curr_data.lat = gps.location.lat();
	curr_data.speed = gps.speed.kmph();
	curr_data.altitude = gps.altitude.meters();
	//curr_data.power = calculate_power();
	curr_data.power = 200;
}

void wifi_setup()
{
	if(WiFi.status() != WL_CONNECTED)
	{
		Serial.print("Attempting to connect");
		while(WiFi.status() != WL_CONNECTED)
		{
			WiFi.begin(ssid, password); 
			delay(5000);     
		} 
		Serial.println("\nConnected.");
	}
	ThingSpeak.begin(client);
}

void upload_to_TS()
{
	ThingSpeak.setField(1, curr_data.speed);
	ThingSpeak.setField(2, curr_data.altitude);
  	ThingSpeak.setField(3, curr_data.power);

	int res = ThingSpeak.writeFields(1, myWriteAPIKey);
	if(res == 200) //Serial.println("Channel update successful.");
	{
		Serial.println("Channel update successful.");
	}
	else
	{
		Serial.println("Problem updating channel. HTTP error code " + String(res));
	}

}

void setup()
{
  	Serial.begin(SERIAL_BAUD);
	Wire.begin(SDA_PIN, SCL_PIN);
	Wire.begin();
	gy91_setup();
	mag.begin();
	gps_setup();
	sd_setup();
	current_data_setup();
	WiFi.mode(WIFI_STA);
	wifi_setup();
}


void loop()
{
	logGY271();
	logGY91();
	if(millis() - lastGPSReading > GPS_READING_INTERVAL)
	{
		gps_reading++;
		lastGPSReading = millis();
		while (SerialGPS.available() > 0)
		{
			char c = SerialGPS.read();
			gps.encode(c);
		}
		logGPS();
		update_current_data_by_GPS();
		appendFile(SD, logFile.c_str(), logLineGPS.str().c_str());
		appendFile(SD, logFile.c_str(), logLineGY271.str().c_str());
		appendFile(SD, logFile.c_str(), logLineGY91.str().c_str());
		appendFile(SD, logFile.c_str(), "\n");
		upload_to_TS();
	}
	Serial.println();
	delay(1000);
};