#include <Wire.h>
#include <SD.h>
 
File myFile;


#define DEVICE_A (0x1D)    //first ADXL345 device address
#define DEVICE_B (0x53)    //second ADXL345 device address
#define TO_READ (6)        //num of bytes we are going to read each time (two bytes for each axis)

byte buff[TO_READ] ;      //6 bytes buffer for saving data read from the device
char str[512];   

// Initializations
void setup()
{
  // Arduino initializations
  Wire.begin();
  Serial.begin(9600);

   //Turning on the both ADXL345s
  writeTo(DEVICE_A, 0x2D, 24);   
  writeTo(DEVICE_B, 0x2D, 24);

  Serial.begin(9600);
  Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work. 
   pinMode(10, OUTPUT);
 
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
 
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("tests.txt", FILE_WRITE);
 
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
  // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

}

int regAddress = 0x32;      //first axis-acceleration-data register on the ADXL345
int xa = 0, ya = 0, za = 0;  
int xb = 0, yb = 0, zb = 0;

int long cpt=0;

// Main loop, read and display data
void loop()
{
  cpt++;
  // Display data counter
  Serial.print (cpt,DEC);
  Serial.print ("\t");
  
  readFrom(DEVICE_A, regAddress, TO_READ, buff); //read the acceleration data from the ADXL345  
   //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
   //thus we are converting both bytes in to one int
  xa = (((int)buff[1]) << 8) | buff[0];   
  ya = (((int)buff[3])<< 8) | buff[2];
  za = (((int)buff[5]) << 8) | buff[4];
  
  readFrom(DEVICE_B, regAddress, TO_READ, buff); //read the acceleration data from the second ADXL345
  xb = (((int)buff[1]) << 8) | buff[0];   
  yb = (((int)buff[3])<< 8) | buff[2];
  zb = (((int)buff[5]) << 8) | buff[4];
 
  
  // Accelerometer
  myFile = SD.open("tests.txt", FILE_WRITE);
  
  myFile.print (cpt,DEC);
  myFile.print ("\t");
  
  Serial.print("AcX1 = "); Serial.print((xa),DEC);
  Serial.print("\t");
  Serial.print(" | AcY1 = "); Serial.print((ya),DEC);
  Serial.print("\t");
  Serial.print(" | AcZ1 = "); Serial.print((za-500),DEC);
  Serial.print("\t");

  Serial.print("AcX2 = "); Serial.print((xb),DEC);
  Serial.print("\t");
  Serial.print(" | AcY2 = "); Serial.print((yb),DEC);
  Serial.print("\t");
  Serial.print(" | AcZ2 = "); Serial.print((zb-500),DEC);
  Serial.print("\t");

  myFile.print("AcX1 = "); myFile.print((xa),DEC);
  myFile.print("\t");
  myFile.print(" | AcY1 = "); myFile.print((ya),DEC);
  myFile.print("\t");
  myFile.print(" | AcZ1 = "); myFile.print((za),DEC);
  myFile.print("\t");
  
  myFile.print("AcX2 = "); myFile.print((xb),DEC);
  myFile.print("\t");
  myFile.print("AcY2 = "); myFile.print((yb),DEC);
  myFile.print("\t");
  myFile.print("AcZ2 = "); myFile.print((zb),DEC);
  myFile.print("\t");
  myFile.println("");
  myFile.close();

  
  // End of line
  Serial.println("");
 
  
  delay(100);    
}

//---------------- Functions
//Writes val to address register on device
void writeTo(int device, byte address, byte val) {
   Wire.beginTransmission(device); //start transmission to device 
   Wire.write(address);        // send register address
   Wire.write(val);        // send value to write
   Wire.endTransmission(); //end transmission
}

//reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(device); //start transmission to device
  Wire.requestFrom(device, num);    // request 6 bytes from device
  
  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  { 
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}


