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
uint8_t target = 0x68;
uint8_t buffer[14];


//filter variables gyro
double alpha = .5;
int16_t rawX[21];
int16_t rawY[20];
int16_t rawZ[20];

int16_t rawGX[20];
int16_t rawGY[21];
int16_t rawGZ[21];

short window = 21;

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

double pitch = 0;
double roll = 0;
MPU6050 accelgyro(0x68);

int16_t ax, ay, az,gx, gy, gz;

void setup()
{
    pinMode(LED_BUILTIN,OUTPUT);    // LED
    pinMode(12,INPUT_PULLUP);       // Control for Test1
    pinMode(11,INPUT_PULLUP);       // Control for Test2

    Serial.begin(250000); //115200

    // Setup for Master mode, pins 18/19, external pullups, 400kHz
    //Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
    Wire.begin();
    delay(1000);
    Serial.println("setting modes");
    /*Wire.beginTransmission(target);
    Wire.write(0x6B); //mode control
    Wire.write(0x01); // wake it up
    Wire.endTransmission();
    Wire.beginTransmission(target);
    Wire.write(0x1C); //mode control accel
    Wire.write(0x00); //16G mode
    Wire.endTransmission();
    Wire.beginTransmission(target);
    Wire.write(0x1B); //mode control gyro
    Wire.write(0x00); //250 mode
    Wire.endTransmission();
    Wire.beginTransmission(target);
    Wire.write(0x6B);
    Wire.write(0x01);
    Wire.endTransmission();
    Wire.beginTransmission(target);
    Wire.write(0x1A); //access CONFIG register
    Wire.write(0x6); //low pass filter hardcore*/
    
    accelgyro.initialize();
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
    
    delay(1);
    //elapsedMicros timer = 0;
    /*Wire.beginTransmission(target);
    Wire.write(0x3C); //read X axis
    Wire.endTransmission();
    Wire.beginTransmission(target);
    Wire.requestFrom(target, 14); //request 1 byte
    //unsigned char buffer;
    if(Wire.available()){
      int i = 0;
      while(Wire.available()) {
        buffer[i] = Wire.readByte();
        i += 1;
      }*/
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      //Serial.printf("i: %d\n", i);
      //Serial.print("X: ");
      //Serial.print(buffer[0]);
      /*if(compX)
        x = (1-alpha) * x +  alpha *(giveAccel(ax, 'x'));
      else
        x = giveAccel(ax, 'x');
      //Serial.print("\n");
      
      //Serial.print("Y: ");
      //Serial.print(buffer[2]);
      if(compY)
        y = (1-alpha) * y + alpha * giveAccel(ay, 'y');
      else
        y = giveAccel(ay, 'y');
      //Serial.print("\n");

      //Serial.print("Z: ");
      //Serial.print(buffer[4]);
      if(compZ)
        z = (1-alpha) * z + alpha * giveAccel(az, 'z');
      else
        z = giveAccel(az, 'z');
      /*Serial.print("\n");
      Serial.print("time: ");
      Serial.print(timer);*/
      
      /*
      double R = sqrt(x*x + y*y + z * z);
      double xAngle = acos(x/R) * 57.29557;
      double yAngle = acos(y/R) * 57.29557;
      double zAngle = acos(z/R) * 57.29557;
      */
      //double xAngle = (atan(x / (sqrt(y*y + z*z)))) * 57.29577;
      //double yAngle = (atan(y / (sqrt(x*x + z*z)))) * 57.29577;
      //double zAngle = (atan(sqrt(x * x  + y * y) / z)) * 57.29577;
      //Serial.printf(("%d\t%d\t%f\n"), (x, y, z));
      /*Serial.print(x);
      Serial.print("\t");
      Serial.print(y);
      Serial.print("\t");
      Serial.print(z);
      Serial.print("\n");
      Serial.print(" Xangle: ");
      Serial.print(xAngle);
      Serial.print(" Yangle: ");
      Serial.print(yAngle);
      Serial.print(" Zangle: ");
      Serial.print(zAngle);
      Serial.print("\n");*/
      double x = giveAccel(ax, 'x');
      double y = giveAccel(ay, 'y');
      double z = giveAccel(az, 'z');
      double xg = giveGyro(xg, 'x');
      double yg = giveGyro(yg, 'y');
      double zg = giveGyro(zg, 'z');
      //Serial.print("x: %f\t y: %f\t z:%f\n", x, y, z);
      Serial.print(x);
      Serial.print("\t");
      Serial.print(y);
      Serial.print("\t");
      Serial.print(z);
      Serial.print("\n");
      /*Serial.print(xg);
      Serial.print("\t");
      Serial.print(yg);
      Serial.print("\t");
      Serial.print(zg);
      Serial.print("\n");*/

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

      //Serial.print("\n");
      Serial.print(roll);
      Serial.print("\t");
      Serial.print(pitch);
      Serial.print("\n");
      /*if(compZ) {
        MadgwickAHRSupdate(xg, yg, zg, x, y, z, 0, 0, 0);
        Serial.print("q0: ");
        Serial.print(q0);
        Serial.print("\tq1: ");
        Serial.print(q1);
        Serial.print("\tq2: ");
        Serial.print(q2);
        Serial.print("\tq3: ");
        Serial.print(q3);
        Serial.print("\n");
        Serial.print("qPitch: ");
        Serial.print(-asin(2 * (q0 * q2 - q1 * q3)) * 57.29577);
        Serial.print("\tqRoll: ");
        Serial.print(atan2((2 * (q0 * q1 + q2 * q3)), 1 - (q2 * q2 + q3 * q3)) * 57.29577);
        Serial.print("\n");
      }
      */
      
    
    

   
}

//
// print scan status
//

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
  int16_t tempArray[21] = {};
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
  return tempArray[10];
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
  int16_t tempArray[21] = {};
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
  return tempArray[10];
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

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
    return;
  }

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;   

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;   

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================

