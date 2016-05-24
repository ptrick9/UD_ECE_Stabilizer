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

//#include <i2c_t3.h>
#include "I2Cdev.h"
//#include "Wire.h"
#include "MPU9150.h"

#define sampleFreq  50.0f    // sample frequency in Hz
#define betaDef   0.01f    // 2 * proportional gain

float beta = betaDef;                // 2 * proportional gain (Kp)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame

// Function prototypes
int16_t medianSort(char c);
void SetTunings(double Kp, double Ki, double Kd);
uint8_t target = 0x68;
uint8_t buffer[14];

unsigned long lastTimePitch;
double InputPitch, OutputPitch, SetpointPitch;
double ITermPitch, lastInputPitch;
unsigned long lastTimeRoll;
double InputRoll, OutputRoll, SetpointRoll;
double ITermRoll, lastInputRoll;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
double outMin, outMax;

float value = 400;
char serData[4];
char old[4];
char num[3];
int val = 0;
char characters = 'a';
//filter variables gyro
double alpha = .5;
int16_t rawX[71];
int16_t rawY[70];
int16_t rawZ[70];

int16_t rawMX[10];
int16_t rawMY[10];
int16_t rawMZ[10];


int16_t rawGX[70];
int16_t rawGY[71];
int16_t rawGZ[71];

short window = 71;
int counter = 0;
short scanX = 0;
short compX = 0;
short scanY = 0;
short compY = 0;
short scanZ = 0;
short compZ = 0;

int setx = 0;
int sety = 0;
int setz = 1;

short trackerX = 0;
short trackerY = 0;
short trackerZ = 0;
short filter = 0;

//gyro variables
short scanGX = 0;
short compGX = 0;
short scanGY = 0;
short compGY = 0;
short scanGZ = 0;
short compGZ = 0;

int setGx = 0;
int setGy = 0;
int setGz = 0;

short trackerGX = 0;
short trackerGY = 0;
short trackerGZ = 0;
short filterG = 0;
unsigned int motorSet = 0;
//offset values
int16_t floatax = 0; //22500
int16_t floatay = 0; //28000
int16_t floataz = 0; //16300
int16_t asens = 16384; //4096


int16_t floatGx = 0;
int16_t floatGy = 0;
int16_t floatGz = 0;
int16_t gsens = 250; 


double x = 0.0;
double y = 0.0;
double z = 0.0;
char control = 'a';
double pitch = 0;
double roll = 0;
MPU9150 accelgyro(0x68);

int16_t ax, ay, az,gx, gy, gz, mx, my, mz;

void setup()
{
    pinMode(3,OUTPUT);
    pinMode(4,OUTPUT);
    pinMode(5,OUTPUT);
    pinMode(6,OUTPUT);
    analogWriteFrequency(3,400);
    analogWriteFrequency(4,400);
    analogWriteFrequency(5,400);
    analogWriteFrequency(6,400);
    analogWriteResolution(10);
    SetOutputLimits(-40.0, 40.0);
    SetSampleTime(5);
    SetTunings(0.03, 0.1, 0.01);
    //SetTunings(0.3, 0.7, 0.0);
    SetpointPitch = 0.0;
    SetpointRoll = 0.0;
    ComputePitch();
    ComputeRoll();
    pinMode(LED_BUILTIN,OUTPUT);    // LED
    pinMode(12,INPUT_PULLUP);       // Control for Test1
    pinMode(11,INPUT_PULLUP);       // Control for Test2

    Serial.begin(250000); //115200

    // Setup for Master mode, pins 18/19, external pullups, 400kHz
    //Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
    Wire.begin();
    delay(1000);
    Serial.println("setting modes");
   
    
    accelgyro.initialize();
    accelgyro.setDLPFMode(4);
    //Wire.endTransmission();
    

    int i = 0;
    while (i < 100) {
      delay(10);
      Wire.beginTransmission(target);
      Wire.requestFrom(target, 14); //request 1 byte
      Wire.endTransmission();
      i += 1;
    }
}

void loop()
{
    
    delay(5);
    motorSet += 1;
    

    if (counter < 4*window) {
      counter += 1;
    }
    
    
      accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    
      double x = giveAccel(ax, 'x');
      double y = giveAccel(ay, 'y');
      double z = giveAccel(az, 'z');
      double xg = giveGyro(xg, 'x');
      double yg = giveGyro(yg, 'y');
      double zg = giveGyro(zg, 'z');


      pitch += xg * .01;
      roll -= yg * .01;
      
      double forceMag = abs(x) + abs(y) + abs(z);
      if(forceMag > .5 && forceMag < 2) {
        //double pitchAcc = atan2(y, x) * 57.29577;
        double pitchAcc = atan2(y,z) * 57.29577;
        pitch = pitch * .5 + pitchAcc * .5;

        double rollAcc = atan2(x,z) * 57.29577;
        roll = roll * .5 + rollAcc * .5;
      }
      InputPitch = pitch;
      InputRoll = roll;

      rawMX[motorSet % 10] = mx;
      rawMY[motorSet % 10] = my;
      rawMZ[motorSet % 10] = mz;
      if (motorSet > 20) {
        int32_t sum = 0;
        int i = 0;
        while (i < 10) {
          sum += rawMX[i];
          i += 1;
        }
        mx = sum/10;
        i = 0;
        sum = 0;
        while (i < 10) {
          sum += rawMY[i];
          i += 1;
        }
        my = sum/10;
        i = 0;
        sum = 0;
        while (i < 10) {
          sum += rawMZ[i];
          i += 1;
        }
        mz = sum/10;
      }
      mx -= -2550;
      my -= 9950;
      mz -= -22125;
      
      //Serial.print("\n");
      Serial.print("r");
      Serial.print(roll);
      Serial.print("\tp");
      Serial.print(pitch);
      Serial.print("\t");
      Serial.print("mx:\t");
      Serial.print(mx);
      Serial.print("\tmy:\t");
      Serial.print(my);
      Serial.print("\tmz:\t");
      Serial.print(mz);
      Serial.print("\theading:\t");
      Serial.print(atan2(-my, mx) * 180 / 3.14159);
      Serial.print("\n");

      

   
}

//
// print scan status
//


/*working variables*/



void ComputePitch()
{
   unsigned long now = millis();
   int timeChangePitch = (now - lastTimePitch);
   if(timeChangePitch>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = SetpointPitch - InputPitch;
      //int error = SetpointPitch - InputPitch;
      Serial.print("\t pitch: ");
      Serial.print(error);
      ITermPitch+= (ki * error);
      if(ITermPitch> outMax) ITermPitch= outMax;
      else if(ITermPitch< outMin) ITermPitch= outMin;
      double dInput = (InputPitch - lastInputPitch);
 
      /*Compute PID Output*/
      OutputPitch = kp * error + ITermPitch- kd * dInput;
      if(OutputPitch > outMax) OutputPitch = outMax;
      else if(OutputPitch < outMin) OutputPitch = outMin;
      Serial.print("\t op:");
      Serial.print(OutputPitch);
      /*Remember some variables for next time*/
      lastInputPitch = InputPitch;
      lastTimePitch = now;
   }
}

void ComputeRoll()
{
   unsigned long now = millis();
   int timeChangeRoll = (now - lastTimeRoll);
   if(timeChangeRoll>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = SetpointRoll - InputRoll;
      //int error = SetpointRoll - InputRoll;
      Serial.print("\t roll: ");
      Serial.print(error);
      ITermRoll+= (ki * error);
      if(ITermRoll> outMax) ITermRoll= outMax;
      else if(ITermRoll< outMin) ITermRoll= outMin;
      double dInput = (InputRoll - lastInputRoll);
 
      /*Compute PID Output*/
      OutputRoll = kp * error + ITermRoll- kd * dInput;
      if(OutputRoll > outMax) OutputRoll = outMax;
      else if(OutputRoll < outMin) OutputRoll = outMin;
      Serial.print("\t or:");
      Serial.print(OutputRoll);
      /*Remember some variables for next time*/
      lastInputRoll = InputRoll;
      lastTimeRoll = now;
   }
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}
 
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 
void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;
    
   if(OutputPitch > outMax) {
    OutputPitch = outMax;
   }
   else if(OutputPitch < outMin) {
    OutputPitch = outMin;
   }

   if(OutputRoll > outMax) {
    OutputRoll = outMax;
   }
   else if(OutputRoll < outMin) {
    OutputRoll = outMin;
   }
 
   if(ITermPitch> outMax){
    ITermPitch= outMax;
   }
   else if(ITermPitch < outMin){
    ITermPitch = outMin;
   }
   if(ITermRoll < outMin) {
    ITermRoll = outMin;
   }
   else if(ITermRoll > outMax) {
    ITermRoll = outMax;
   }
}







double giveAccel(int16_t val, char axis) {
  int16_t raw = val;
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
      x = setx;
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
      y = sety;
      /*Serial.print(" compensated  Y: ");
      Serial.print(floatay);
      Serial.print(" ");*/
    }
  }
  else if(axis == 'z') {
    compensated = raw - floataz;
    rawZ[trackerZ] = compensated;
    /*Serial.print("raw z: ");
    Serial.print(raw);
    Serial.print("\t");*/
    //Serial.print((double(compensated))/asens);*/
    trackerZ += 1;
    if (trackerZ == window) {
      filter = 1;
      trackerZ = 0;
    }
    scanZ += 1;
    if(scanZ == (2 * window) && compZ == 0) {
      floataz = average('z') - (setz * asens);
      compZ = 1;
      z = setz;
      /*Serial.print(" compensated  Z ");
      Serial.print(floataz);
      Serial.print(" ");*/
    }
  }
  
  if (filter == 1) {
    compensated = medianSort(axis);
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

double giveGyro(int16_t val, char axis) {
  int16_t raw = val;
  int16_t compensated;
  if(axis == 'x') {
    compensated = raw - floatGx;
    rawGX[trackerGX] = compensated;
    /*Serial.print("unsmoothed: ");
    Serial.print((double(compensated))/asens);*/
    trackerGX += 1;
    if (trackerGX == window) {
      filterG = 1;
      trackerGX = 0;
    }
    scanGX += 1;
    if(scanGX == (2 * window) && compGX == 0) {
      floatGx = averageG('x') - (setGx * gsens);
      compGX = 1;
      /*Serial.print(" compensated  X ");
      Serial.print(floatax);
      Serial.print(" ");*/
    }
  }
  else if(axis == 'y') {
    compensated = raw - floatGy;
    rawGY[trackerGY] = compensated;
    /*Serial.print("unsmoothed: ");
    Serial.print((double(compensated))/asens);*/
    trackerGY += 1;
    if (trackerGY == window) {
      filterG = 1;
      trackerGY = 0;
    }
    scanGY += 1;
    if(scanGY == (2 * window) && compGY == 0) {
      floatGy = averageG('y') - (setGy * gsens);
      compGY = 1;
      /*Serial.print(" compensated  Y: ");
      Serial.print(floatay);
      Serial.print(" ");*/
    }
  }
  else if(axis == 'z') {
    compensated = raw - floatGz;
    rawGZ[trackerGZ] = compensated;
    /*Serial.print("unsmoothed: ");
    Serial.print((double(compensated))/asens);*/
    trackerGZ += 1;
    if (trackerGZ == window) {
      filterG = 1;
      trackerGZ = 0;
    }
    scanGZ += 1;
    if(scanGZ == (2 * window) && compGZ == 0) {
      floatGz = averageG('z') - (setGz * gsens);
      compGZ = 1;
      /*Serial.print(" compensated  Z ");
      Serial.print(floataz);
      Serial.print(" ");*/
    }
  }
  
  if (filterG == 1) {
    compensated = medianSortG(axis);
  }
  
  /*Serial.print("tracker: ");
  Serial.print(tracker);*/
  /*Serial.print(" raw: ");
  Serial.print(raw);
  Serial.print(" compensated: ");
  Serial.print(compensated);*/
  
  double corrected = (double(compensated))/gsens;
  /*Serial.print(" corrected: ");
  Serial.print(corrected);
  Serial.print("\n");*/
  return corrected;
}

int16_t medianSort(char axis) {
  int16_t tempArray[71] = {};
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
  return tempArray[35];
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

int16_t medianSortG(char axis) {
  int16_t tempArray[71] = {};
  int i = 0;
  if(axis == 'x') {
    while (i < window) {
      tempArray[i] = rawGX[i];
      i += 1;
    }
  }
  else if(axis == 'y') {
    while (i < window) {
      tempArray[i] = rawGY[i];
      i += 1;
    }
  }
  else {
    while (i < window) {
      tempArray[i] = rawGZ[i];
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
  return tempArray[35];
}

int16_t averageG(char axis) {
  int sum = 0;
  int i = 0;
  if(axis == 'x') {
    while (i < window) {
      sum += rawGX[i];
      i += 1;
    }
  }
  else if(axis == 'y') {
    while (i < window) {
      sum += rawGY[i];
      i += 1;
    }
  }
  else {
    while (i < window) {
      sum += rawGZ[i];
      i += 1;
    }
  }

  return (sum/window);
  
}

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update


