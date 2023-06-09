#include <Arduino.h>
#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
void setup(void)
{
    Serial.begin(115200);
    /* Initialise the sensor */
    if(!mag.begin())
    {
        /* There was a problem detecting the HMC5883 ... check your connections */
        Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
        while(1);
    }
}

void loop(void)
{
    //strange data, does not match phone magnetometer data
    sensors_event_t event;
    mag.getEvent(&event);
    Serial.print("X: ");
    Serial.print(event.magnetic.x);
    Serial.print(" ");
    Serial.print("Y: ");
    Serial.print(event.magnetic.y);
    Serial.print(" ");
    Serial.print("Z: ");
    Serial.print(event.magnetic.z);
    Serial.print(" ");
    Serial.println("uT");
    float heading = atan2(event.magnetic.y, event.magnetic.x);
    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    float declinationAngle = 0.08;
    heading += declinationAngle;
    // Correct for when signs are reversed.
    if(heading < 0)
    heading += 2*PI;
    // // Check for wrap due to addition of declination.
    if(heading > 2*PI)
    heading -= 2*PI;
    // Convert radians to degrees for readability.
    float headingDegrees = heading * 180/PI;
    Serial.print("Heading (degrees): ");
    Serial.println(headingDegrees);
    delay(500);
}