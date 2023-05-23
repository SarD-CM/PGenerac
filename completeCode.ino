#include <Filter.h>
#include <DueTimer.h>

//el barrita no hizo nada
float revolutions = 0;
int rpm = 0;  // max value 32,767 16 bit
long startTime = 0;
long elapsedTime;
float mrpm;
float wrpm;
float vel;
bool s;
float temp;
int potenciometer;
int so;

long pi = 3.1415926536;

float kv = 8.9460;
float torque = 0.0;
float pmec = 0.0;
float pmot = 0.0;
float pbatt = 0.0;

const int timeThreshold = 5;
long timeCounter = 0;
const int maxSteps = 255;
volatile int ISRCounter = 0;
int counter = 0;
long deltaTime = 0;
bool IsCW = true;

float p = 0.0;
float im = 0.0;
float ib = 0.0;
float vm = 0.0;
float vb = 0.0;

float imF = 0.0;
float ibF = 0.0;
float vmF = 0.0;
float vbF = 0.0;

float currentm=0.0;
float currentb=0.0;


#define channelPinA 2
#define channelPinB 3
#define throttlePedalOut 9
#define pedalSwitch 4
#define switchOut 9
#define switchIn 6

#define throttlePedalIn A0
#define currentSensor A1
#define voltageSensor A2
#define voltageBat A3
#define currentBat A4

//Filter assignments
ExponentialFilter<float> filteredBcurrent(20, 0);
ExponentialFilter<float> filteredMcurrent(20, 0);
ExponentialFilter<float> filteredBvoltage(10, 0);
ExponentialFilter<float> filteredMvoltage(90, 0);

void setup() {

  analogReadResolution(12);
  Serial.begin(115200);
  pinMode(channelPinA, INPUT_PULLUP);  // set pin to input
  pinMode(channelPinB, INPUT_PULLUP);
  pinMode(switchIn, INPUT_PULLUP);
  pinMode(pedalSwitch, INPUT_PULLUP);

  pinMode(switchOut, OUTPUT);

  Timer3.attachInterrupt(batterySensorRead).setFrequency(300).start();
  Timer5.attachInterrupt(motorSensorRead).setFrequency(300).start();
  Timer4.attachInterrupt(throttle).setFrequency(300).start();

  attachInterrupt(digitalPinToInterrupt(channelPinA), doEncodeA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channelPinB), doEncodeB, CHANGE);
}

void loop() {
  //filtering

  filteredBvoltage.Filter(vb);
  vbF = filteredBvoltage.Current();

  filteredBvoltage.Filter(ib);
  ibF = filteredBcurrent.Current();

  filteredMcurrent.Filter(im);
  imF = filteredMcurrent.Current();

  filteredMvoltage.Filter(vm);
  vmF = filteredMvoltage.Current();

  //RPM calculation
  //now let's see how many counts we've had from the hall effect sensor and calc the RPM
  //finds the time, should be very close to 1 sec





  //detachInterrupt(E2);
  //detachInterrupt(E1);
  if (counter != ISRCounter) {
    deltaTime = millis() - timeCounter;
    counter = ISRCounter / (deltaTime);
  }
  delay(500);
  String outRPM = String("Motor velocity   [rpm]: ") + counter;
  Serial.println(outRPM);





  //Motor Voltage

  float voltagem = map(vmF, 3262, 3200, 0, 5.79);
  String outVOLM = String("Motor voltage      [V]: ") + vmF;
  //Serial.println(outVOLM);

  //Battery Voltage
  float voltageb = map(vbF, 0, 2905, 0, 51.34);
  String outVOLB = String("Battery voltage    [V]: ") + voltageb;
  //Serial.println(outVOLB);


  //Motor Current
  float imF2 =map(imF,0,4096,0,3.3);
  currentm = 0.0042* imF2 +2.624;
  if(im!=0){
  temp=im;
  String outIM = String("Motor current      [A]: ") + im;
  
  Serial.println(temp);
  }  

  
  //Battery current
  float ibF2 = map(ibF, 0, 4096,0,3.3);
  currentb=0.0042* ibF2 +2.624;;
  String outIB = String("Battery current    [A]: ") + ib;
  //Serial.println(outIB);

  //Motor torque
  torque = 0.0981*currentm + 0.2081;
  String outT = String("Motor torque      [Nm]: ") + torque;
  //Serial.println(outT);

  // General Switch

  s = digitalRead(switchIn);
  if (!s) {
    analogWrite(switchOut, 255);
  } else {
    analogWrite(switchOut, 0);
  }



  //Pedal
  String outP = String("Throttle           [%]: ") + potenciometer;
  potenciometer = map(p, 0, 4096, 0, 245);
  float throttle = map(p, 0, 4096, 0, 100);
  analogWrite(throttlePedalOut, potenciometer);
  //Serial.println(outP);

  //Power
  pmec = rpm * torque * pi / 30;
  pmot = voltagem * currentm;
  pbatt = voltageb * currentb;

  float list[] = { voltageb, voltagem, currentm, currentb, torque, throttle, rpm, vel, pmec, pmot, pbatt };
  //Serial.print(String(list[2]) + "," + String(list[3]) + "," + String(list[1]) + "," + String(list[4]) + "," + String(list[7]) + "," + String(list[6]) + "," + String(list[5]) + "," + String(list[8]) + "," + String(list[9]) + "," + String(list[10]) + "," String(list[11]));
}


void motorSensorRead() {
  im = analogRead(currentSensor);

  vm = analogRead(voltageSensor);
}

void batterySensorRead() {
  vb = analogRead(voltageBat);
  ib = analogRead(currentBat);
}
void throttle() {
  p = analogRead(throttlePedalIn);
}

void doEncodeA() {
  if (millis() > timeCounter + timeThreshold) {
    if (digitalRead(channelPinA) == digitalRead(channelPinB)) {
      IsCW = true;
      ISRCounter--;
    } else {
      IsCW = false;
      ISRCounter++;
    }
    timeCounter = millis();
  }
}

void doEncodeB() {
  if (millis() > timeCounter + timeThreshold) {
    if (digitalRead(channelPinA) != digitalRead(channelPinB)) {
      IsCW = true;
      ISRCounter--;
    } else {
      IsCW = false;
      ISRCounter++;
    }
    timeCounter = millis();
  }
}
