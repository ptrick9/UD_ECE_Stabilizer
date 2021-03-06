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
#include "MPU6050.h"


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
int16_t rawY[71];
int16_t rawZ[71];
int32_t oldTime;
int16_t rawGX[71];
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
MPU6050 accelgyro(0x68);

double TpwmTop = 1906; //transmitter top val
double TpwmLow = 1070; //transmitter low val
double PpwmTop = 1906; //transmitter top val
double PpwmLow = 1070; //transmitter low val
double RpwmTop = 1906; //transmitter top val
double RpwmLow = 1070; //transmitter low val
double RpwmStepsTop = 761; //steps at 400Hz 10bit resolution
double RpwmStepsLow = 434; //steps at 400Hz 10bit resolution
double RLow = 1060;
double RTop = 1860;
double Rdif = RpwmStepsTop-RpwmStepsLow; //steps at 400Hz 10bit resolution
double RpwmArm = 400; //steps at 400Hz 10bit resolution


double Thigh = 0;
double Tlow = 0;
double Ttotal = 0;
double Tpercent = 0;
int ThrottleVal = 0;

double Rhigh = 0;
double Rlow = 0;
double Rtotal = 0;
double Rpercent = 0;

double Phigh = 0;
double Plow = 0;
double Ptotal = 0;
double Ppercent = 0;
int8_t mod = 0;

uint32_t throt_in;
uint32_t pitch_in;
uint32_t roll_in;



int16_t ax, ay, az,gx, gy, gz;

void setup()
{
    pinMode(3,OUTPUT);
    pinMode(4,OUTPUT);
    pinMode(5,OUTPUT);
    pinMode(6,OUTPUT);
    pinMode(10,INPUT); //throttle
    pinMode(11,INPUT); //pitch
    pinMode(12,INPUT); //roll
    attachInterrupt(10, isr_throttle, CHANGE);
    attachInterrupt(11, isr_pitch, CHANGE);
    attachInterrupt(12, isr_roll, CHANGE);
    analogWriteFrequency(3,400);
    analogWriteFrequency(4,400);
    analogWriteFrequency(5,400);
    analogWriteFrequency(6,400);
    analogWriteResolution(10);

    analogWrite(3, RpwmArm);
    analogWrite(4, RpwmArm);
    analogWrite(5, RpwmArm);
    analogWrite(6, RpwmArm);
    
    SetOutputLimits(-40.0, 40.0);  //changed from 40
    SetSampleTime(5); //was 5
    SetTunings(0.03, 0.1, 0.01);
    //SetTunings(0.062, 0.018, 0.01);
    SetpointPitch = 0.0;
    SetpointRoll = 0.0;
    ComputePitch();
    ComputeRoll();
    //pinMode(LED_BUILTIN,OUTPUT);    // LED
    //pinMode(12,INPUT_PULLUP);       // Control for Test1
    //pinMode(11,INPUT_PULLUP);       // Control for Test2

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
  Serial.printf("%i\n", micros() - oldTime);
    oldTime = micros();
    /*
    if (counter % 20 != 0) {
      delay(20);
    }
    if (counter % 20 == 0 && mod == 0) {
      Thigh = pulseIn(10, HIGH);
      Tlow = pulseIn(10, LOW);
      Ttotal = Tlow + Thigh;
      Serial.printf("total: %f high: %f low: %f pwmLow: %f pwmTop: %f\n", Ttotal, Thigh, TpwmLow, TpwmLow, TpwmTop);
      Tpercent = abs((Thigh-TpwmLow)/(TpwmTop-TpwmLow));
      mod += 1;
    }
    else if (counter % 20 == 0 && mod == 1) {
      Rhigh = pulseIn(12, HIGH);
      Rlow = pulseIn(12, LOW);
      Rtotal = Rlow + Rhigh;
      Rpercent = abs((Rhigh-RpwmLow)/(RpwmTop-RpwmLow));
      mod += 1;
    }
    else if (counter % 20 == 0 && mod == 2) {
      Phigh = pulseIn(11, HIGH);
      Plow = pulseIn(11, LOW);
      Ptotal = Plow + Phigh;
      Ppercent = abs((Phigh-PpwmLow)/(PpwmTop-PpwmLow));
      mod = 0;
    }
    */
    delay(5);
    SetpointPitch = int(Ppercent * 10) - 5; 
    SetpointRoll = int(Rpercent * 10) - 5;
    if(Tpercent > 1.0) {
      ThrottleVal = RpwmStepsTop;
    }
    else if(Tpercent < 0.05) {
      ThrottleVal = RpwmArm;
    }
    else {
      ThrottleVal = RpwmStepsLow+Rdif*Tpercent;
    }

    //Serial.printf("Tpercent: %f counter%i \n" ,Tpercent, counter);
    //Serial.printf("rpercent: %f\n",Rpercent);
    motorSet += 1;
    /*
    if(Serial.available() > 0) {
      Serial.printf("%i\n",Serial.available());
      Serial.readBytesUntil(characters, old, 5);
      Serial.println("we are reading");
      if(old[0] == ' ') {
        Serial.println("working");
      }
      else {
        int i = 0;
        while (i < 4) {
          serData[i] = old[i];
          i += 1;
        }
      }
      Serial.printf("%c%c%c\n", serData[0], serData[1], serData[2]);
      if (serData[0] == 'x') {
        control = 'a';
      }
      else if (serData[0] == 'p') {
        control = 'p';
        //SetpointPitch = val;
      }
      else if (serData[0] == 'r') {
        control = 'r';
      }
      else if (serData[0] == '0' || serData[0] == '-') {
        val = atoi(serData);
        Serial.print(val);
        if (control == 'p') {
          SetpointPitch = val;
        }
        else if (control == 'r') {
          SetpointRoll = val;
        }
      }
      
      Serial.printf("%i\n", val);
    }
    */
    if (counter < 4*window) {
      counter += 1;
    }
    else {
      ComputePitch();
      ComputeRoll();
      counter += 1;
    }
    
    //left is negative degrees
    //3 is left motor
    if (counter >= 4*window) {
      //if (motorSet % 2 == 0) {
        float motorOutPitch = OutputPitch/100;
        float motorOutRoll = OutputRoll/100;
        analogWrite(4, ThrottleVal - 8 - 325 * motorOutRoll);
        analogWrite(3, ThrottleVal + 325 * motorOutRoll);
        
        //analogWrite(5, ThrottleVal + 1 - 325 * motorOutPitch);
        //analogWrite(6, ThrottleVal + 325 * motorOutPitch);
        /*analogWrite(4, ThrottleVal-8); //good
        analogWrite(3, ThrottleVal);
        
        analogWrite(5, ThrottleVal + 1);
        analogWrite(6, ThrottleVal); *///good
      //}
      
      
      //float left = .4 - motorOut;
      //float right = .4 - left;

      
      
    }
    /*
    else if (control == 'a') {
      analogWrite(3, 434);
      analogWrite(4, 434);
      analogWrite(5, 434);
      analogWrite(6, 434);
    }*/
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
      double x = giveAccel(ax, 'x');
      double y = giveAccel(ay, 'y');
      double z = giveAccel(az, 'z');
      double xg = giveGyro(xg, 'x');
      double yg = giveGyro(yg, 'y');
      double zg = giveGyro(zg, 'z');


      pitch += xg * .01; //this might be wrong, should maybe be .005? (5 msec sample rate)
      roll -= yg * .01;
      
      double forceMag = abs(x) + abs(y) + abs(z);             //play with the values of the .5
      if(forceMag > .5 && forceMag < 2) {
        //double pitchAcc = atan2(y, x) * 57.29577;
        double pitchAcc = atan2(y,z) * 57.29577;
        pitch = pitch * .5 + pitchAcc * .5;

        double rollAcc = atan2(x,z) * 57.29577;
        roll = roll * .5 + rollAcc * .5;
      }
      InputPitch = pitch;
      InputRoll = roll;
      //Serial.print("\n");
      /*Serial.print("r");
      Serial.print(roll);
      Serial.print("\tp");
      Serial.print(pitch);
      //Serial.print("\t");
      //Serial.print(val);
      Serial.print("\tp");
      Serial.print(SetpointPitch);
      Serial.print("\tr");
      Serial.print(SetpointRoll);
      //Serial.print("\t");
      //Serial.print(control);
      Serial.print("\n");*/
      
    
    

   
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
      //Serial.print("\t pe: ");
      //Serial.print(error);
      ITermPitch+= (ki * error);
      if(ITermPitch> outMax) ITermPitch= outMax;
      else if(ITermPitch< outMin) ITermPitch= outMin;
      double dInput = (InputPitch - lastInputPitch);
 
      /*Compute PID Output*/
      OutputPitch = kp * error + ITermPitch- kd * dInput;
      if(OutputPitch > outMax) OutputPitch = outMax;
      else if(OutputPitch < outMin) OutputPitch = outMin;
      //Serial.print("\t op:");
      //Serial.print(OutputPitch);
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
      /*Serial.print("\t re: ");
      Serial.print(error);
      Serial.print("\n"); */
      ITermRoll+= (ki * error);
      if(ITermRoll> outMax) ITermRoll= outMax;
      else if(ITermRoll< outMin) ITermRoll= outMin;
      double dInput = (InputRoll - lastInputRoll);
 
      /*Compute PID Output*/
      OutputRoll = kp * error + ITermRoll- kd * dInput;
      if(OutputRoll > outMax) OutputRoll = outMax;
      else if(OutputRoll < outMin) OutputRoll = outMin;
      //Serial.print("\t or:");
      //Serial.print(OutputRoll);
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
  
  if (filter == 1) { //potentially get rid of this to drastically speed up readings time 
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

void isr_throttle() {
  if(digitalRead(10) == HIGH) {
    throt_in = micros();
    //Serial.printf("throt_in: %i\n", throt_in);
  }
  else {
    /*if(throt_in < 1066) {
      throt_in = 1067;
    }
    else if (throt_in > 1906) {
      throt_in = 1905;
    }*/
    //Serial.printf("diff: %i\n", micros() - throt_in);
    Tpercent = abs((float)(micros() - throt_in-1066)/836);
    if (Tpercent < 0.20) {
      Tpercent = 0.0;
    }
    else if (Tpercent > 1.0) {
      Tpercent = 1.0;
    }
  }
}

void isr_pitch() {
  if(digitalRead(11) == HIGH) {
    pitch_in = micros();
  }
  else {
    Ppercent = abs((float)(micros() - pitch_in-1066)/836);
    if (Ppercent < 0.02) {
      Ppercent = 0.0;
    }
    else if (Ppercent > 1.0) {
      Ppercent = 1.0;
    }
  }
}

void isr_roll() {
  if(digitalRead(12) == HIGH) {
    roll_in = micros();
  }
  else {
    Rpercent = abs((float)(micros() - roll_in-1066)/836);
    if (Rpercent < 0.02) {
      Rpercent = 0.0;
    }
    else if (Rpercent > 1.0) {
      Rpercent = 1.0;
    }
    
  }
}



float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update


