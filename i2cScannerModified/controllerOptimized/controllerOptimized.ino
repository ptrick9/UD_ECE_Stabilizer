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
unsigned long lastTimeYaw;
double InputYaw, OutputYaw, SetpointYaw;
double ITermYaw, lastInputYaw;
double kp, ki, kd;
double kpy, kiy, kdy;
int SampleTime = 1000; //1 sec
double outMin, outMax;
double outMinYaw, outMaxYaw;

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

int16_t rawGX[71];
int16_t rawGY[71];
int16_t rawGZ[71];

int32_t AX_sum;
int32_t AY_sum;
int32_t AZ_sum;

int32_t GX_sum;
int32_t GY_sum;
int32_t GZ_sum;

float AXFiltered;
float AYFiltered;
float AZFiltered;

float GXFiltered;
float GYFiltered;
float GZFiltered;



int32_t window = 4;
int counter = 0;
int32_t scanX = 0;
int32_t compX = 0;
int32_t scanY = 0;
int32_t compY = 0;
int32_t scanZ = 0;
int32_t compZ = 0;
int32_t oldTime = 0;
int setx = 0;
int sety = 0;
int setz = 1;

int32_t trackerX = 0;
int32_t trackerY = 0;
int32_t trackerZ = 0;
int32_t filter = 0;

//gyro variables
int32_t scanGX = 0;
int32_t compGX = 0;
int32_t scanGY = 0;
int32_t compGY = 0;
int32_t scanGZ = 0;
int32_t compGZ = 0;

int setGx = 0;
int setGy = 0;
int setGz = 0;

int32_t trackerGX = 0;
int32_t trackerGY = 0;
int32_t trackerGZ = 0;
int32_t filterG = 0;
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
double yaw = 0;
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
    
    SetOutputLimits(-20.0, 20.0);  //changed from 40
    SetOutputLimitsYaw(-10.0, 10.0);
    SetSampleTime(4); //was 5
    //SetTunings(0.03, 0.1, 0.01);
    //SetTunings(0.079, 0.12, 0.02);
    SetTunings(0.079, .12, 0.023);
    SetTuningsYaw(0.9, 0.4, 0.01);
    SetpointPitch = 0.0;
    SetpointRoll = 0.0;
    SetpointYaw = 0;
    ComputePitch();
    ComputeRoll();
    ComputeYaw();
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
    accelgyro.setDLPFMode(6);
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
    /*Serial.printf("%i\n", micros() - oldTime);
    oldTime = micros();*/
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
    delay(4);
    SetpointPitch = int(Ppercent * 10) - 5; 
    SetpointRoll = int(Rpercent * 10) - 5;
    SetpointYaw = 0;
    if(Tpercent > 1.0) {
      ThrottleVal = RpwmStepsTop;
    }
    else if(Tpercent < 0.05) {
      ThrottleVal = RpwmArm;
    }
    else {
      ThrottleVal = RpwmStepsLow+Rdif*Tpercent;
    }

    //Serial.printf("Tpercent: %f\n" ,Tpercent);
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
      ComputeYaw();
      counter += 1;
    }
    
    //left is negative degrees
    //3 is left motor
    if (counter >= 4*window) {
      //if (motorSet % 2 == 0) {
        float motorOutPitch = OutputPitch/100;
        float motorOutRoll = OutputRoll/100;
        float motorOutYaw = OutputYaw/100;
        analogWrite(4, ThrottleVal + 10 + 325* motorOutRoll - 325 * motorOutYaw);
        analogWrite(3, ThrottleVal + 0 - 325 * motorOutRoll - 325 * motorOutYaw);
        
        analogWrite(5, ThrottleVal + 12 - 325 * motorOutPitch + 325 * motorOutYaw);
        analogWrite(6, ThrottleVal + 7 + 325 * motorOutPitch + 325 * motorOutYaw);
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
      double xg = giveGyro(gx, 'x');
      double yg = giveGyro(gy, 'y');
      double zg = giveGyro(gz, 'z');
      
      yaw += zg * .005;
      pitch += xg * .005; //this might be wrong, should maybe be .005? (5 msec sample rate)
      roll -= yg * .005;
      
      double forceMag = abs(x) + abs(y) + abs(z);             //play with the values of the .5
      if(forceMag > .5 && forceMag < 2) {
        //double pitchAcc = atan2(y, x) * 57.29577;
        double pitchAcc = atan2(y,z) * 57.29577;
        pitch = pitch * .98 + pitchAcc * .02;

        double rollAcc = atan2(x,z) * 57.29577;
        roll = roll * .98 + rollAcc * .02;
      }
      InputPitch = pitch;
      InputRoll = roll;
      InputYaw = yaw;
      //Serial.print("\n");
      //Serial.print("r");
      Serial.print(roll);
      Serial.print("\t");
      Serial.print(pitch);
      Serial.print("\t");
      //Serial.print(yaw);
      //Serial.print("\t");
      Serial.print(SetpointPitch);
      Serial.print("\t");
      Serial.print(SetpointRoll);
      //Serial.print("\t");
      //Serial.print(OutputYaw);
      Serial.print("\n");
      //Serial.print(val);
      /*Serial.print("\tp");
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
      Serial.print("\n");*/
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

void ComputeYaw()
{
   unsigned long now = millis();
   int timeChangeYaw = (now - lastTimeYaw);
   if(timeChangeYaw>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = SetpointYaw - InputYaw;
      //int error = SetpointPitch - InputPitch;
      //Serial.print("\t pe: ");
      //Serial.print(error);
      ITermYaw+= (kiy * error);
      if(ITermYaw> outMaxYaw) ITermYaw= outMaxYaw;
      else if(ITermYaw< outMinYaw) ITermYaw= outMinYaw;
      double dInput = (InputYaw - lastInputYaw);
 
      /*Compute PID Output*/
      OutputYaw = kpy * error + ITermYaw- kdy * dInput;
      if(OutputYaw > outMaxYaw) OutputYaw = outMaxYaw;
      else if(OutputYaw < outMinYaw) OutputYaw = outMinYaw;
      //Serial.print("\t op:");
      //Serial.print(OutputPitch);
      /*Remember some variables for next time*/
      lastInputYaw = InputYaw;
      lastTimeYaw = now;
   }
}

void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}

void SetTuningsYaw(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kpy = Kp;
   kiy = Ki * SampleTimeInSec;
   kdy = Kd / SampleTimeInSec;
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

void SetOutputLimitsYaw(double Min, double Max)
{
   if(Min > Max) return;
   outMinYaw = Min;
   outMaxYaw = Max;
    
   if(OutputYaw > outMaxYaw) {
    OutputYaw = outMaxYaw;
   }
   else if(OutputYaw < outMinYaw) {
    OutputYaw = outMinYaw;
   }
   if(ITermYaw < outMinYaw) {
    ITermYaw = outMinYaw;
   }
   else if(ITermYaw > outMaxYaw) {
    ITermYaw = outMaxYaw;
   }
}





double giveAccel(int16_t val, char axis) {
  int16_t raw = val;
  int16_t compensated;
  if(axis == 'x') {
    compensated = raw - floatax;
    if (compX == 0) {
      rawX[trackerX] = compensated;
      AX_sum += compensated;
      trackerX += 1;
      scanX += 1;
      if (trackerX == window) {
        compX = 1;
      }
    }
    else {
      AX_sum -= rawX[trackerX];
      rawX[trackerX] = compensated;
      trackerX += 1;
      trackerX = trackerX % window;
      AX_sum += compensated;
      scanX += 1;
      
    }
    if (scanX == 2*window && filter == 0) {
      floatax = AX_sum/window - (setx * asens);
      compX = 1;
      scanX += 1;
      x = setx;
    }
    if (filter == 1) {
      compensated = AX_sum / window;
    }
  }
  else if(axis == 'y') {
    compensated = raw - floatay;
    if (compY == 0) {
      rawY[trackerY] = compensated;
      AY_sum += compensated;
      trackerY += 1;
      scanY += 1;
      if (trackerY == window) {
        compY = 1;
      }
    }
    else {
      AY_sum -= rawY[trackerY];
      rawY[trackerY] = compensated;
      trackerY += 1;
      trackerY = trackerY % window;
      AY_sum += compensated;
      scanY += 1;
      
    }
    if (scanY == 2*window && filter == 0) {
      floatay = AY_sum/window - (sety * asens);
      compY = 1;
      scanY += 1;
      y = sety;
    }
    if (filter == 1) {
      compensated = AY_sum / window;
    }
  }
  else if(axis == 'z') {
    compensated = raw - floataz;
    if (compZ == 0) {
      rawZ[trackerZ] = compensated;
      AZ_sum += compensated;
      trackerZ += 1;
      scanZ += 1;
      if (trackerZ == window) {
        compZ = 1;
      }
    }
    else {
      //Serial.printf("az: %i val: %i\n", AZ_sum, rawZ[trackerZ]);
      AZ_sum -= rawZ[trackerZ];
      AZ_sum += compensated;
      rawZ[trackerZ] = compensated;
      trackerZ += 1;
      scanZ += 1;
      trackerZ = trackerZ % window;
      
      //Serial.printf("az: %i val: %i\n", AZ_sum, rawZ[trackerZ]);
    }
    if (scanZ == 2*window && filter == 0) {
     
      //Serial.printf("floataz: %i sum: %i div: %f\n", floataz, AZ_sum, AZ_sum/window);
      floataz = AZ_sum/window - (setz * asens);
      //Serial.printf("floataz: %i\n", floataz);
       //delay(10000);
      compZ = 1;
      scanZ += 1;
      filter = 1;
      z = setz;
    }
    if (filter == 1) {
      compensated = AZ_sum / window;
    }
  }
  
  /*if (filter == 1) { //potentially get rid of this to drastically speed up readings time 
    compensated = medianSort(axis);
  }*/
  
 
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
    if (compGX == 0) {
      rawGX[trackerGX] = compensated;
      GX_sum += compensated;
      trackerGX += 1;
      scanGX += 1;
      if (trackerGX == window) {
        compGX = 1;
      }
    }
    else {
      GX_sum -= rawGX[trackerGX];
      rawGX[trackerGX] = compensated;
      trackerGX += 1;
      trackerGX = trackerGX % window;
      GX_sum += compensated;
      scanGX += 1;
      
    }
    if (scanGX == 2*window && filterG == 0) {
      floatGx = GX_sum/window - (setGx * asens);
      compGX = 1;
      scanGX += 1;
      x = setGx;
    }
    if (filterG == 1) {
      compensated = GX_sum / window;
    }
  }
  else if(axis == 'y') {
    compensated = raw - floatGy;
    if (compGY == 0) {
      rawGY[trackerGY] = compensated;
      GY_sum += compensated;
      trackerGY += 1;
      scanGY += 1;
      if (trackerGY == window) {
        compGY = 1;
      }
    }
    else {
      GY_sum -= rawGY[trackerGY];
      rawGY[trackerGY] = compensated;
      trackerGY += 1;
      trackerGY = trackerGY % window;
      GY_sum += compensated;
      scanGY += 1;
      
    }
    if (scanGY == 2*window && filterG == 0) {
      floatGy = GY_sum/window - (setGy * asens);
      compGY = 1;
      scanGY += 1;
      y = setGy;
    }
    if (filterG == 1) {
      compensated = GY_sum / window;
    }
  }
  else if(axis == 'z') {
    compensated = raw - floatGz;
    if (compGZ == 0) {
      rawGZ[trackerGZ] = compensated;
      GZ_sum += compensated;
      trackerGZ += 1;
      scanGZ += 1;
      if (trackerGZ == window) {
        compGZ = 1;
      }
    }
    else {
      GZ_sum -= rawGZ[trackerGZ];
      rawGZ[trackerGZ] = compensated;
      trackerGZ += 1;
      scanGZ += 1;
      trackerGZ = trackerGZ % window;
      GZ_sum += compensated;
      
    }
    if (scanGZ == 2*window && filterG == 0) {
      floatGz = GZ_sum/window - (setGz * asens);
      compGZ = 1;
      scanGZ += 1;
      filterG = 1;
      z = setGz;
    }
    if (filterG == 1) {
      compensated = GZ_sum / window;
    }
  }
  
  /*if (filterG == 1) {
    compensated = medianSortG(axis);
  }*/
  
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



void isr_throttle() {
  if(digitalRead(10) == HIGH) {
    throt_in = micros();
    //Serial.printf("throt_in: %i\n", throt_in);
  }
  else {
    /*if((micros() - throt_in) < 1066) {
      throt_in = micros() - 1067;
    }
    else if ((micros() - throt_in) > 1906) {
      throt_in = micros() - 1905;
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
    if((micros() - pitch_in) < 1066) {
      pitch_in = micros() - 1067;
    }
    else if ((micros() - pitch_in) > 1906) {
      pitch_in = micros() - 1905;
    }
    Ppercent = abs((float)(micros() - pitch_in-1066)/836);
    /*if (Ppercent < 0.02) {
      Ppercent = 0.0;
    }
    else if (Ppercent > 1.0) {
      Ppercent = 1.0;
    }*/
  }
}

void isr_roll() {
  if(digitalRead(12) == HIGH) {
    roll_in = micros();
  }
  else {
    if((micros() - roll_in) < 1066) {
      roll_in = micros() - 1067;
    }
    else if ((micros() - roll_in) > 1906) {
      roll_in = micros() - 1905;
    }
    Rpercent = abs((float)(micros() - roll_in-1066)/836);
    /*if (Rpercent < 0.02) {
      Rpercent = 0.0;
    }
    else if (Rpercent > 1.0) {
      Rpercent = 1.0;
    }*/
    
  }
}



float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update


