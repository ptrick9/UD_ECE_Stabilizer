double TpwmTop = 1906; //transmitter top val
double TpwmLow = 1070; //transmitter low val
double RpwmStepsTop = 761; //steps at 400Hz 10bit resolution
double RpwmStepsLow = 434; //steps at 400Hz 10bit resolution
double RLow = 1060;
double RTop = 1860;
double Rdif = RpwmStepsTop-RpwmStepsLow; //steps at 400Hz 10bit resolution
double RpwmArm = 400; //steps at 400Hz 10bit resolution

void setup() {
  // put your setup code here, to run once:
  pinMode(3,OUTPUT);
  pinMode(2,INPUT);
  pinMode(4,INPUT);
  analogWriteFrequency(3, 400);
  analogWriteResolution(10);
  analogWrite(3, RpwmArm);
  Serial.begin(115200);
  delay(5);
}

void loop() {
  //read transmitter
  double Thigh = pulseIn(2, HIGH);
  double Tlow = pulseIn(2, LOW);
  double Ttotal = Tlow + Thigh;
  double Tpercent = abs((Thigh-TpwmLow)/(TpwmTop-TpwmLow));

  //read output
  double Rhigh = pulseIn(4, HIGH);
  double Rlow = pulseIn(4, LOW);
  double Rtotal = Rlow + Rhigh;
  double Rpercent = abs((Rhigh-RLow)/(RTop-RLow));
  
  if(Thigh > TpwmTop) {
    analogWrite(3,RpwmStepsTop);
  }
  else if(Thigh < TpwmLow) {
    analogWrite(3,RpwmStepsLow);
  }
  else {
    analogWrite(3, RpwmStepsLow+Rdif*Tpercent);
  }
  Serial.printf("high: %f\tlow: %f\ttotal: %f\tpercentThrottle: %f\n",Thigh, Tlow, Ttotal, Tpercent);
  Serial.printf("high: %f\tlow: %f\ttotal: %f\tpercentThrottle: %f\n",Rhigh, Rlow, Rtotal, Rpercent);
  delay(10);
}
