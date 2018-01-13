 // MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include <SPI.h>

#include <SD.h>

#include <Wire.h>
File myFile;

byte address = 0x00;
int CS1=10;
int CS2=9;

const int MPU_addr=0x69;  // I2C address of the MPU-6050
#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

long int cpt=0;

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  

  
  Serial.begin(9600);
  Serial.print("Initializing SD card...");
  pinMode(CS1, OUTPUT);
  pinMode(CS2,OUTPUT);
  if (!SD.begin(10)) {
  Serial.println("initialization failed!");
  return;
  }
  Serial.println("initialization done.");

  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) {
   Serial.print("Writing to test.txt...");
 

   } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
   }
    // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_16_G);
  // Set by pass mode for the magnetometers
  //I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
}
void loop(){

  for (int i = 0; i <= 128; i++)
  {
  digitalPotWrite(i);
  delay(10);
  }
  delay(500);
  for (int i = 128; i >= 0; i--)
  {
  digitalPotWrite(i);
  delay(10);
  }
  
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
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
    
  myFile.println("AcX1 = ");myFile.println(AcX);
  myFile.println("AcY1 = ");myFile.println(AcY);
  myFile.println("AcZ1 = \t");myFile.println(AcZ);

  

  //Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  //Serial.print(" | GyX = "); Serial.print(GyX);
  //Serial.print(" | GyY = "); Serial.print(GyY);
  //Serial.print(" | GyZ = "); Serial.println(GyZ);
    // Display data counter
  Serial.print (cpt++,DEC);
  Serial.print ("\t");
  
 
 
  // ____________________________________
  // :::  accelerometer and gyroscope ::: 

  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  
  
  // Create 16 bits values from 8 bits data
  
  // Accelerometer
  int16_t ax=-(Buf[0]<<8 | Buf[1]);
  int16_t ay=-(Buf[2]<<8 | Buf[3]);
  int16_t az=Buf[4]<<8 | Buf[5];

  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]);
  int16_t gz=Buf[12]<<8 | Buf[13];
  
    // Display values
  
  // Accelerometer
  Serial.print("AcX2 = "); Serial.print((ax/229.009233979591837)+0.221094007,DEC);
  Serial.print(" | AcY2 = "); Serial.print((ay/229.009233979591837)+0.065941677,DEC);
  Serial.print(" | AcZ2 = "); Serial.print((az/229.009233979591837),DEC);
  myFile.println(" | AcX2 = ");myFile.println((ax/229.009233979591837)+0.221094007,DEC);
  myFile.println(" | AcY2 = ");myFile.println((ay/229.009233979591837)+0.065941677,DEC);
  myFile.println(" | AcZ2 = ");myFile.println((az/229.009233979591837),DEC);
  
  //Serial.print ((ax/229.009233979591837)+0.221094007 ,DEC); 
  //Serial.print ("\t");
  //Serial.print ((ay/229.009233979591837)+0.065941677,DEC);
  //Serial.print ("\t");
  //Serial.print (az/229.009233979591837,DEC);  
  //Serial.print ("\t");
  
  // Gyroscope
  //Serial.print (gx,DEC); 
  //Serial.print ("\t");
  //Serial.print (gy,DEC);
  //Serial.print ("\t");
  //Serial.print (gz,DEC);  
  //Serial.print ("\t");
  Serial.println("");
  
  delay(100);

  
  


}

int digitalPotWrite(int value)
  {
  digitalWrite(CS2, LOW);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CS2, HIGH);
  }



 

