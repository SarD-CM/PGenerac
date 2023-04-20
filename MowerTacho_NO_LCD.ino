float revolutions = 0;
int rpm = 0;  // max value 32,767 16 bit
long startTime = 0;
long elapsedTime;
int v=0;

void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT_PULLUP);  // set pin to input
}

void loop() {

  revolutions = 0;
  rpm = 0;
  startTime = millis();
  attachInterrupt(digitalPinToInterrupt(2), interruptFunction, RISING);
  delay(1000);
  detachInterrupt(2);

  //now let's see how many counts we've had from the hall effect sensor and calc the RPM
  elapsedTime = millis() - startTime;  //finds the time, should be very close to 1 sec

  if (revolutions > 0) {
    rpm = (max(1, revolutions) * 60000) / (elapsedTime * 0.4);  //calculates rpm
  }

  String outRPM = String("RPM :") + rpm;
  Serial.println(outRPM);

  analogReadResolution(12);
  v = analogRead(A0);
  float voltage = v * (3.3 / 4096);
  float motorVoltage = voltage * 18.43;

  String outVOL = String("Motor voltage: ") + motorVoltage;
  Serial.println(outVOL);
}

void interruptFunction()  //interrupt service routine
{
  revolutions++;
}
