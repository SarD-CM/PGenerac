#include <TimeInterrupt.h>

float revolutions = 0;
int rpm = 0;  // max value 32,767 16 bit
long startTime = 0;
long elapsedTime;
float mrpm;
float wrpm;
float vel;
bool s;
int potenciometer;
int so;
int v1;

int p = 0;
float i = 0.0;
float vm = 0.0;
float vb = 0.0;


#define E1 2
#define E2 3
#define throttlePedalOut 9
#define pedalSwitch 4
#define switchOut 9
#define switchIn 6

#define throttlePedalIn A4
#define currentSensor A5
#define voltageSensor A0
#define voltageBat A3


void setup() {


  Serial.begin(9600);
  pinMode(E1, INPUT_PULLUP);  // set pin to input
  pinMode(E2, INPUT_PULLUP);
  pinMode(switchIn, INPUT_PULLUP);
  pinMode(pedalSwitch, INPUT_PULLUP);

  pinMode(switchOut, OUTPUT);

  TimeInterrupt.begin(PRECISION);
  TimeInterrupt.addInterrupt(sensorRead, 100);
  TimeInterrupt.addInterrupt(throttle, 100);
}


void loop() {

  //analogReadResolution(12);

  revolutions = 0;
  rpm = 0;
  startTime = millis();
  attachInterrupt(digitalPinToInterrupt(E1), interruptFunction, RISING);
  attachInterrupt(digitalPinToInterrupt(E2), interruptFunction, RISING);
  // attachInterrupt(digitalPinToInterrupt(pedalSwitch), pedalFunction, HIGH);

  delay(1000);

  //RPM calculation
  //now let's see how many counts we've had from the hall effect sensor and calc the RPM
  elapsedTime = millis() - startTime;  //finds the time, should be very close to 1 sec

  if (revolutions > 0) {
    rpm = (max(1, revolutions) * 60) / (48 * 2 * elapsedTime * 0.4);  //calculates rpm
    mrpm = rpm * 1.5;                                                 //angular velocity from second axle to motor
    wrpm = rpm / 1.2413793103;                                        //angular velocity from second axle to wheels

    vel = ((rpm / 60000) * 2 * 3.1416) * (135);  //linear velocity in m/s
  }

  String outRPM = String("RPM :") + rpm;
  Serial.println(outRPM);

  //Motor Voltage

  float voltagem = map(vm, 550, 0, 0, 49.22);
  String outVOLM = String("Motor voltage: ") + voltagem;
  Serial.println(outVOLM);

  //Battery Voltage
  float voltageb = map(vb, 0, 1023, 0, 53);
  String outVOLB = String("Battery voltage: ") + voltageb;
  Serial.println(vb);


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
  String outP = String("Throttle ") + potenciometer;
  potenciometer = map(p, 0, 1023, 0, 245);
  analogWrite(throttlePedalOut, potenciometer);
    Serial.println(outP);
}
void interruptFunction()  //interrupt service routine
{
  revolutions++;
}

void sensorRead() {
  i = analogRead(currentSensor);
  vm = analogRead(voltageSensor);
  vb = analogRead(voltageBat);
}

void throttle() {
  p = analogRead(throttlePedalIn);
}
