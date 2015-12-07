#include <avr/io.h>
#include <avr/interrupt.h>

int ledPin = 13;
int inputPin = 0;

int ledVal = 1;
int ByteReceived;
void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  pinMode(inputPin, INPUT);
  digitalWrite(ledPin,HIGH);
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(inputPin), changed, CHANGE);
  
}





void loop() {
  // put your main code here, to run repeatedly:
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
    Serial.println("hi this is iteration 1");
    delay(500);

}

void changed() {
  if(digitalRead(inputPin)) {
    digitalWrite(ledPin, HIGH);
    Serial.println("setting to high");
  }
  else {
    digitalWrite(ledPin,LOW);
    Serial.println("setting to low");
  }
}



