float revolutions = 0;
int rpm = 0;  // max value 32,767 16 bit
long startTime = 0;
long elapsedTime;
int v = 0;
float mrpm = 0;
float wrpm = 0;
float vel = 0;
int i = 0;
int p = 0;
bool s;
int potenciometer;
int so;

#define E1 2
#define E2 3
#define throttlePedalOut 8
#define pedalSwitch 4
#define switchOut 9
#define switchIn 6

#define throttlePedalIn A4
#define currentSensor A5
#define voltageSensor A0


void setup() {
  Serial.begin(9600);
  pinMode(E1, INPUT_PULLUP);  // set pin to input
  pinMode(E2, INPUT_PULLUP);
  pinMode(currentSensor, INPUT);
  pinMode(voltageSensor, INPUT);
  pinMode(throttlePedalIn, INPUT);
  pinMode(throttlePedalIn, INPUT);
  pinMode(pedalSwitch, INPUT_PULLUP);
  pinMode(switchOut, OUTPUT);
  pinMode(switchIn, INPUT_PULLUP);
}


void loop() {

  revolutions = 0;
  rpm = 0;
  startTime = millis();
  attachInterrupt(digitalPinToInterrupt(E1), interruptFunction, RISING);
  attachInterrupt(digitalPinToInterrupt(E2), interruptFunction, RISING);
  // attachInterrupt(digitalPinToInterrupt(pedalSwitch), pedalFunction, HIGH);

  delay(1000);
  detachInterrupt(E1);
  detachInterrupt(E2);
  detachInterrupt(pedalSwitch);

  //now let's see how many counts we've had from the hall effect sensor and calc the RPM
  elapsedTime = millis() - startTime;  //finds the time, should be very close to 1 sec

  if (revolutions > 0) {
    rpm = (max(1, revolutions) * 60000) / (48 * 2 * elapsedTime * 0.4);  //calculates rpm
    mrpm = rpm * 1.5;                                                    //angular velocity from second axle to motor
    wrpm = rpm / 1.2413793103;                                           //angular velocity from second axle to wheels

    vel = ((rpm / 60000) * 2 * 3.1416) * (135);  //linear velocity in m/s
  }

  String outRPM = String("RPM :") + rpm;
  Serial.println(outRPM);
  //analogReadResolution(12);
  v = analogRead(voltageSensor);
  delay(10);
  v = analogRead(voltageSensor);
  delay(10);

  i = analogRead(currentSensor);
  delay(10);
  i = analogRead(currentSensor);
  delay(10);

  p = analogRead(throttlePedalIn);
  delay(10);
  p = analogRead(throttlePedalIn);


  //Motor Voltage
  float voltage = map(v, 0, 512, 0, 48);
  String outVOL = String("Motor voltage: ") + voltage;
  Serial.println(outVOL);

  //Motor Current
  //float current = i * (5 / 1024);
  float current = map(i, 520, 613, 0, 100);
  String outI = String("Motor current: ") + current;
  Serial.println(outI);

  // General Switch
  s = digitalRead(switchIn);
  if (!s) {
    analogWrite(switchOut, 255);
  } else {
    analogWrite(switchOut, 0);
  }

  //Pedal
  potenciometer = map(p, 0, 1023, 0, 255);
  analogWrite(throttlePedalOut, potenciometer);
  String outP = String("Throttle ") + potenciometer;
  Serial.println(outP);
  
}
void interruptFunction()  //interrupt service routine
{
  revolutions++;
}
