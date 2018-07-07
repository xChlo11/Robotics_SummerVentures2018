#include "Wire.h"

#define MPU9255_ADDRESS 0x68
#define WHO_AM_I 0x75
#define ACCEL_XOUT 0x3B
#define ACCEL_YOUT 0x3D
#define ACCEL_ZOUT 0x3F
#define GYRO_XOUT 0x43
#define GYRO_YOUT 0x45
#define GYRO_ZOUT 0x47

const double pi = 3.14159265358979;
const double gyro_deg2rad = 500.0 * pi / 5898240.0;
float rollGyro;
float rollAcc;
float alpha = .97;
int i,j;

int32_t a[3];
int32_t g[3];

byte device_id;
byte error;
byte acc_address[] = {ACCEL_XOUT, ACCEL_YOUT, ACCEL_ZOUT};
byte gyro_address[] = {GYRO_XOUT, GYRO_YOUT, GYRO_ZOUT};

double roll, pitch, rollRate, pitchRate, rollAngle, pitchAngle, dt;

void setup() 
{
  Wire.begin();
  
  Serial.begin(115200);
  Serial.println("Initializing I2C communication with MPU9255...");
  //Check that the device ID stored in the WHO_AM_I register matches the known ID of 0x71 for the MPU9255
  //This is to ensure that the device is properly connected
  device_id = read(MPU9255_ADDRESS, WHO_AM_I, 1);
 // Serial.println(device_id == 0x71 ? "Communication with MPU9255 successful" : "MPU9255 not found");
  //while(!(device_id == 0x71));

  Wire.beginTransmission(MPU9255_ADDRESS);
  Wire.write(0x3B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU9255_ADDRESS);
  Wire.write(0x3C);
  Wire.write(0x00);
  Wire.endTransmission();
  
  dt = micros();
}

void loop() 
{
for (int i=0; i<=2; i++)
{
  a[i] = read(MPU9255_ADDRESS, acc_address[i], 2);
  g[i] = read(MPU9255_ADDRESS, gyro_address[i], 2);
}
/*Serial.print(a[0]); Serial.print(" ");
Serial.print(a[1]); Serial.print(" ");
Serial.print(a[2]); Serial.print(" ");

Serial.print(g[0]); Serial.print(" ");
Serial.print(g[1]); Serial.print(" ");
Serial.println(g[2]);*/

rollRate= g[1]*gyro_deg2rad;
rollGyro = rollGyro + (rollRate * ((micros() - dt))/1000000.0);
rollAcc=atan2(a[0], a[2]);
dt= micros();

rollAngle = alpha * (rollAngle + rollRate * (micros() - dt)/1000000.0) + (1 - alpha) * rollAcc;
Serial.print(rollAngle); Serial.print(" "); Serial.println(rollRate);// Serial.print(" "); Serial.println(rollAcc);

}

int16_t read(byte deviceAddress, byte valueAddress, int bytesToRead)
{
  //Specify the register address from which to read
  Wire.beginTransmission(deviceAddress);
  Wire.write(valueAddress);
  error = Wire.endTransmission();
  
  //Read value from specified register
  //When reading the device ID, only one read needs to be performed as the ID is one byte
  //The acc and gyro values are two bytes with the high and low bytes alternating e.g. ACC_XOUT_H, ACC_XOUT_L, ACC_YOUT_H, etc
  //Since the high byte is read first, the value is bit shifted right 8 times before being added to the low byte
  Wire.beginTransmission(deviceAddress);
  Wire.requestFrom(deviceAddress, bytesToRead);
  
  if(bytesToRead == 1){
   return Wire.read();
  }
  else{
   return (Wire.read() << 8) + Wire.read();
  }
  
  error = Wire.endTransmission();
}

void write(byte deviceAddress, byte valueAddress, byte data)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(valueAddress);
  Wire.write(data);
  Wire.endTransmission();
}


