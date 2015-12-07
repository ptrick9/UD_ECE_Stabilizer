#include <avr/io.h>
#include <avr/interrupt.h>

int ledPin = 13;
int inputPin = 0;

int ledVal = 1;
int ByteReceived;

int myTimes[100];
int samples = 0;
int measure = 0;
elapsedMicros frequency;

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  pinMode(inputPin, INPUT);
  digitalWrite(ledPin,HIGH);
  Serial.begin(9600);
  frequency = 0;
  attachInterrupt(digitalPinToInterrupt(inputPin), changed, CHANGE);
  
}





void loop() {
  // put your main code here, to run repeatedly:
  if(measure == 1) {
    int i = 0;
    int sum = 0;
    while (i < 100) {
      sum = sum + myTimes[i];
      i += 1;
    }
    long int average = sum / 100;
    Serial.print(1000000/(average * 2));
    Serial.println("Hz");
    samples = 0;
    measure = 0;
    
  }
  /*if (Serial.available() > 0) {
    ByteReceived = Serial.read();
    Serial.print("Received character: ");
    Serial.print(char(ByteReceived));

    if (ByteReceived == '1') {
      if(ledVal == 1) {
        digitalWrite(ledPin,LOW);
        ledVal = 0;
      }
      else {
        digitalWrite(ledPin,HIGH);
        ledVal = 1;
      }
    }
    Serial.println();
  }*/
    

}

void changed() {
  if(samples < 100 && measure == 0) {
    myTimes[samples] = frequency;
    frequency = 0;
    samples += 1;
  }
  else {
    measure = 1;
  }
  if(digitalRead(inputPin)) {
    digitalWrite(ledPin, HIGH);
    //Serial.print("setting to high ");
    //Serial.println(samples);
  }
  else {
    digitalWrite(ledPin,LOW);
    //Serial.print("setting to low ");
    //Serial.println(samples);
  }
}



