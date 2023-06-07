
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

#define MPU9250_ADDRESS 0x68

#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_16_G 0x18

Adafruit_BMP280 bme;

int stat_bme = 0;


void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  // Serial.print("Wire available: "); // debug prints
  // Serial.println(Wire.available());

  while (Wire.available())
  {
    Data[index++] = Wire.read();
    //Serial.println("Data array data:"); //debug prints
    //Serial.print(index-1);
    //Serial.print("\t");
    //Serial.println(Data[index-1]);
  }
}

void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void setup()
{
  Wire.begin(21, 22);
  Wire.begin();
  Serial.begin(115200);

  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);

  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);

  stat_bme = bme.begin(0x76,0x58);
  Serial.print("BMP280 status init: ");
  Serial.println(stat_bme);
}

void loop()
{
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

  int16_t ax = -(Buf[0] << 8 | Buf[1]);
  int16_t ay = -(Buf[2] << 8 | Buf[3]);
  int16_t az = Buf[4] << 8 | Buf[5];

  int16_t gx = -(Buf[8] << 8 | Buf[9]);
  int16_t gy = -(Buf[10] << 8 | Buf[11]);
  int16_t gz = Buf[12] << 8 | Buf[13];

  Serial.print("A:");
  Serial.print("\t");
  Serial.print(ax, DEC);
  Serial.print("\t");
  Serial.print(ay, DEC);
  Serial.print("\t");
  Serial.print(az, DEC);
  Serial.print("\t");

  Serial.print("G:");
  Serial.print("\t");
  Serial.print(gx, DEC);
  Serial.print("\t");
  Serial.print(gy, DEC);
  Serial.print("\t");
  Serial.print(gz, DEC);
  Serial.print("\t");


  Serial.println("");

  Serial.print("\t");

  Serial.print(millis());

  Serial.print("\tTemperature(*C): ");
  Serial.print(bme.readTemperature());

  Serial.print("\tPressure(Inches(Hg)): ");
  Serial.print(bme.readPressure()/3377);

  Serial.print("\tApproxAltitude(m): ");
  Serial.print(bme.readAltitude(1018.25)); // local sea level pressure (for Bulgaria - 1018.25)

  Serial.println(""); 

  delay(1000);
}