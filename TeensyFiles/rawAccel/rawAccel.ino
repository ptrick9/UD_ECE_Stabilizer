#include <i2c_t3.h>

uint8_t target = 0x68;
uint8_t buffer[14];
int16_t x,y,z = 0;
void setup() {
  pinMode(LED_BUILTIN,OUTPUT);    // LED
    pinMode(12,INPUT_PULLUP);       // Control for Test1
    pinMode(11,INPUT_PULLUP);       // Control for Test2

    Serial.begin(250000); //115200

    // Setup for Master mode, pins 18/19, external pullups, 400kHz
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
    delay(1000);
    Serial.println("setting modes");
    
    Wire.beginTransmission(target);
    Wire.send(0x6B);
    Wire.send(0x01);
    Wire.endTransmission();

    Wire.beginTransmission(target);
    Wire.send(0x1B);
    Wire.send(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(target);
    Wire.send(0x1C);
    Wire.send(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(target);
    Wire.send(0x6B);
    Wire.send(0x01);
    Wire.endTransmission();
    int i = 0;
    while (i < 100) {
      delay(10);
      Wire.beginTransmission(target);
      Wire.requestFrom(target, 14, I2C_STOP); //request 1 byte
      Wire.endTransmission();
      i += 1;
    }

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(25);
  elapsedMicros timer = 0;
  Wire.beginTransmission(target);
  Wire.send(0x3C); //read X axis
  Wire.endTransmission();
  Wire.beginTransmission(target);
  Wire.requestFrom(target, 14, I2C_STOP); //request 1 byte
  //unsigned char buffer;
  if(Wire.available()){
    int i = 0;
    while(Wire.available()) {
      buffer[i] = Wire.readByte();
      i += 1;
    }

    x = giveAccel(buffer[0], buffer[1], 'x');
    y = giveAccel(buffer[2], buffer[3], 'y');
    z = giveAccel(buffer[4], buffer[5], 'z');
    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.print(z);
    Serial.print("\n");
}
}

int16_t giveAccel(uint8_t high, uint8_t low, char axis) {
  int16_t raw = ((high << 8) | low);
  return raw;
}
