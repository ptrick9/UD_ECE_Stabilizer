// -------------------------------------------------------------------------------------------
// Teensy3.0/3.1/LC I2C Scanner
// 08Mar13 Brian (nox771 at gmail.com)
// -------------------------------------------------------------------------------------------
//
// This creates an I2C master device which will scan the address space and report all
// devices which ACK.  It does not attempt to transfer data, it only reports which devices
// ACK their address.
//
// Pull the control pin low to initiate the scan.  Result will output to Serial.
//
// This example code is in the public domain.
// -------------------------------------------------------------------------------------------

#include <i2c_t3.h>
//#include "I2Cdev.h"

// Function prototypes
int16_t medianSort(char c);
uint8_t target = 0x68;
uint8_t buffer[14];


//filter variables
double alpha = .25;
int16_t rawX[31];
int16_t rawY[31];
int16_t rawZ[31];

short window = 31;

short scanX = 0;
short compX = 0;
short scanY = 0;
short compY = 0;
short scanZ = 0;
short compZ = 0;

int setx = -1;
int sety = 0;
int setz = 0;

short trackerX = 0;
short trackerY = 0;
short trackerZ = 0;
short filter = 0;

//offset values
int16_t floatax = 0; //22500
int16_t floatay = 0; //28000
int16_t floataz = 0; //16300
int16_t asens = 4096; //4096


int16_t flogtgx = 22500;
int16_t flogtgy = -2048;
int16_t flogtgz = -21248;

double x = 0.0;
double y = 0.0;
double z = 0.0;

void setup()
{
    pinMode(LED_BUILTIN,OUTPUT);    // LED
    pinMode(12,INPUT_PULLUP);       // Control for Test1
    pinMode(11,INPUT_PULLUP);       // Control for Test2

    Serial.begin(250000); //115200

    // Setup for Master mode, pins 18/19, external pullups, 400kHz
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
    delay(1000);
    Serial.println("setting modes");
    Wire.beginTransmission(target);
    Wire.send(0x6B); //mode control
    Wire.send(0x00); // wake it up
    Wire.endTransmission();
    Wire.beginTransmission(target);
    Wire.send(0x1C); //mode control
    Wire.send(0x10); //8G mode
    Wire.endTransmission();
    Wire.beginTransmission(target);
    Wire.send(0x1A); //access CONFIG register
    Wire.send(0x6); //low pass filter hardcore
    Wire.endTransmission();
}

void loop()
{
    
    delay(10);
    elapsedMicros timer = 0;
    Wire.beginTransmission(target);
    Wire.send(0x3C); //read X axis
    Wire.endTransmission();
    Wire.beginTransmission(target);
    Wire.requestFrom(target, 6, I2C_STOP); //request 1 byte
    //unsigned char buffer;
    if(Wire.available()){
      int i = 0;
      while(Wire.available()) {
        buffer[i] = Wire.readByte();
        i += 1;
      }
      //Serial.printf("i: %d\n", i);
      //Serial.print("X: ");
      //Serial.print(buffer[0]);
      if(compX)
        x = (1-alpha) * x +  alpha *(giveAccel(buffer[0], buffer[1], 'x'));
      else
        x = giveAccel(buffer[0], buffer[1], 'x');
      //Serial.print("\n");
      
      //Serial.print("Y: ");
      //Serial.print(buffer[2]);
      if(compY)
        y = (1-alpha) * y + alpha * giveAccel(buffer[2], buffer[3], 'y');
      else
        y = giveAccel(buffer[2], buffer[3], 'y');
      //Serial.print("\n");

      //Serial.print("Z: ");
      //Serial.print(buffer[4]);
      if(compZ)
        z = (1-alpha) * z + alpha * giveAccel(buffer[4], buffer[5], 'z');
      else
        z = giveAccel(buffer[4], buffer[5], 'z');
      /*Serial.print("\n");
      Serial.print("time: ");
      Serial.print(timer);*/
      

      double xAngle = (atan(x / (sqrt(y*y + z*z)))) * 57.29577;
      double yAngle = (atan(y / (sqrt(x*x + z*z)))) * 57.29577;
      double zAngle = (atan(sqrt(x * x  + y * y) / z)) * 57.29577;
      //Serial.printf(("%d\t%d\t%f\n"), (x, y, z));
      Serial.print(x);
      Serial.print("\t");
      Serial.print(y);
      Serial.print("\t");
      Serial.print(z);
      Serial.print("\n");
      /*Serial.print(" Xangle: ");
      Serial.print(xAngle);
      Serial.print(" Yangle: ");
      Serial.print(yAngle);
      Serial.print(" Zangle: ");
      Serial.print(zAngle);
      Serial.print("\n");*/
      /*
      Serial.print("XG: ");
      //Serial.print(buffer[8]);
      Serial.print(((int16_t)((buffer[8]) << 8 | buffer[9])));
      Serial.print("\n");
      
      Serial.print("YG: ");
      //Serial.print(buffer[10]);
      Serial.print(((int16_t)((buffer[10]) << 8 | buffer[11])));
      Serial.print("\n");

      Serial.print("ZG: ");
      //Serial.print(buffer[12]);
      Serial.print(((int16_t)((buffer[12]) << 8 | buffer[13])));
      Serial.print("\n\n\n");
      */
    }
    Wire.endTransmission();

   
}

//
// print scan status
//

double giveAccel(uint8_t high, uint8_t low, char axis) {
  int16_t raw = ((high << 8) + low);
  int16_t compensated;
  if(axis == 'x') {
    compensated = raw - floatax;
    rawX[trackerX] = compensated;
    /*Serial.print("unsmoothed: ");
    Serial.print((double(compensated))/asens);*/
    trackerX += 1;
    if (trackerX == window) {
      filter = 1;
      trackerX = 0;
    }
    scanX += 1;
    if(scanX == (2 * window) && compX == 0) {
      floatax = average('x') - (setx * asens);
      compX = 1;
      /*Serial.print(" compensated  X ");
      Serial.print(floatax);
      Serial.print(" ");*/
    }
  }
  else if(axis == 'y') {
    compensated = raw - floatay;
    rawY[trackerY] = compensated;
    /*Serial.print("unsmoothed: ");
    Serial.print((double(compensated))/asens);*/
    trackerY += 1;
    if (trackerY == window) {
      filter = 1;
      trackerY = 0;
    }
    scanY += 1;
    if(scanY == (2 * window) && compY == 0) {
      floatay = average('y') - (sety * asens);
      compY = 1;
      /*Serial.print(" compensated  Y: ");
      Serial.print(floatay);
      Serial.print(" ");*/
    }
  }
  else if(axis == 'z') {
    compensated = raw - floataz;
    rawZ[trackerZ] = compensated;
    /*Serial.print("unsmoothed: ");
    Serial.print((double(compensated))/asens);*/
    trackerZ += 1;
    if (trackerZ == window) {
      filter = 1;
      trackerZ = 0;
    }
    scanZ += 1;
    if(scanZ == (2 * window) && compZ == 0) {
      floataz = average('z') - (setz * asens);
      compZ = 1;
      /*Serial.print(" compensated  Z ");
      Serial.print(floataz);
      Serial.print(" ");*/
    }
  }
  
  if (filter == 1) {
    compensated = average(axis);//medianSort(axis);
  }
  
  /*Serial.print("tracker: ");
  Serial.print(tracker);*/
  /*Serial.print(" raw: ");
  Serial.print(raw);
  Serial.print(" compensated: ");
  Serial.print(compensated);*/
  
  double corrected = (double(compensated))/asens;
  /*Serial.print(" corrected: ");
  Serial.print(corrected);
  Serial.print("\n");*/
  return corrected;
}

int16_t medianSort(char axis) {
  int16_t tempArray[31] = {};
  int i = 0;
  if(axis == 'x') {
    while (i < window) {
      tempArray[i] = rawX[i];
      i += 1;
    }
  }
  else if(axis == 'y') {
    while (i < window) {
      tempArray[i] = rawY[i];
      i += 1;
    }
  }
  else {
    while (i < window) {
      tempArray[i] = rawZ[i];
      i += 1;
    }
  }
  for(int i = 0; i < window; i++) {
    int16_t temp = tempArray[i];
    for(int j = i+1; j < window; j++) {
       if(temp > tempArray[j]) {
        tempArray[i] = tempArray[j];
        tempArray[j] = temp; 
        temp = tempArray[i];
       }
    }
  }
  return tempArray[15];
}

int16_t average(char axis) {
  int sum = 0;
  int i = 0;
  if(axis == 'x') {
    while (i < window) {
      sum += rawX[i];
      i += 1;
    }
  }
  else if(axis == 'y') {
    while (i < window) {
      sum += rawY[i];
      i += 1;
    }
  }
  else {
    while (i < window) {
      sum += rawZ[i];
      i += 1;
    }
  }

  return (sum/window);
  
}

