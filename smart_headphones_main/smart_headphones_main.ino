#include<Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

const int MPU_addr=0x68;  // I2C address of the MPU-6050 Gyro Accellerometer
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float acX,acY,acZ,gyX,gyY,gyZ = 0;
float gValue = 16384; // Value of 1g force when range is +-2g, see datasheet for values range
float dValue = 131; // Value of 1 degree force when range is +-250 deg/sec, see datasheet for values range


void setup() {
  Wire.begin();
  Serial.begin(9600);

  setupMPU6050();
  
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void loop() {

  getMPU6050();
  getBME280();
  
  delay(1000);
}


void getBME280(){
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");


  //Serial.print("Pressure = ");

  //Serial.print(bme.readPressure() / 100.0F);
  //Serial.println(" hPa");

  //Serial.print("Approx. Altitude = ");
  //Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  //Serial.println(" m");

  Serial.println();
}


void setupMPU6050(){
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
}

void getMPU6050(){

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
      
  acX = (float)AcX/gValue;
  acY = (float)AcY/gValue;
  acZ = (float)AcZ/gValue;
  gyX = (float)GyX/dValue;
  gyY = (float)GyY/dValue;
  gyZ = (float)GyZ/dValue;
    
  Serial.print("AcX = "); Serial.print(acX);
  Serial.print(" | acY = "); Serial.print(acY);
  Serial.print(" | acZ = "); Serial.print(acZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | gyX = "); Serial.print(gyX);
  Serial.print(" | gyY = "); Serial.print(gyY);
  Serial.print(" | gyZ = "); Serial.println(gyZ);  
  
}

