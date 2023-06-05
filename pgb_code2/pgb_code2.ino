#include <DueTimer.h>
#include <math.h>

//el santiago no hizo nada
float revolutions = 0;
float temporal = 0;
long startTime = 0;
long elapsedTime;
float mrpm;
float wrpm;
float vel;
float temp;
int potenciometer;

int capacity = 52;
float dischargeTime = 0.0;

long pi = 3.1415926536;
long wm = 0.0;
float kv = 8.9460;

double torque = 0.0;

float pmec = 0.0;
float pmot = 0.0;
float pbatt = 0.0;

const int timeThreshold = 5;
long timeCounter = 0;
const int maxSteps = 255;
float ISRCounter = 0;
unsigned long timeold;
int counter = 0;
long deltaTime = 0;
bool IsCW = true;

float throttle = 0.0;

float p = 0.0;
float im = 0.0;
float ib = 0.0;
float vm = 0.0;
float vb = 0.0;

float imF = 0.0;
float ibF = 0.0;
float vmF = 0.0;
float vbF = 0.0;
float wmF = 0.0;

double imF2 = 0.0;
double ibF2 = 0.0;
double vmF2 = 0.0;
double vbF2 = 0.0;
double wmF2 = 0.0;

int soc = 0.0;
int dischargeF = 0.0;

float currentm = 0.0;
float currentb = 0.0;
float voltagem = 0.0;
float voltageb = 0.0;


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
#define WINDOW_SIZE 10
#define WINDOW_S 25
#define WINDOW_S2 5

int INDEX1, INDEX2, INDEX3, INDEX4, INDEX5, INDEX6 = 0;
int SUM1, SUM2, SUM3, SUM4, SUM5, SUM6 = 0;
int READINGS1[WINDOW_SIZE], READINGS2[WINDOW_S], READINGS3[WINDOW_SIZE], READINGS4[WINDOW_SIZE], READINGS5[WINDOW_SIZE], READINGS6[WINDOW_S2];
float result1, result2, result3, result4, result5, result6;

//Function assignments
void motorSensorRead();
void batterySensorRead();
void doEncode();
void throttleFun();

float doFilter1();
float doFilter2();
float doFilter3();
float doFilter4();
float doFilter5();
float doFilter6();

void setup() {
  analogReadResolution(12);
  Serial.begin(115200);
  pinMode(channelPinA, INPUT_PULLUP);  // set pin to input
  pinMode(channelPinB, INPUT_PULLUP);
  pinMode(switchIn, INPUT_PULLUP);
  pinMode(pedalSwitch, INPUT_PULLUP);
  pinMode(switchOut, OUTPUT);

  pinMode(5,OUTPUT);

  Timer3.attachInterrupt(batterySensorRead).setFrequency(500).start();
  Timer4.attachInterrupt(throttleFun).setFrequency(200).start();
  Timer5.attachInterrupt(motorSensorRead).setFrequency(500).start();

  attachInterrupt(digitalPinToInterrupt(channelPinA), doEncode, RISING);
}

void loop() {
  digitalWrite(5,HIGH);
  if (millis() - timeold >= 200) {

    temporal = ISRCounter * 60000; //(millis()-timeold);
    wm = float(temporal / ((millis() - timeold) * 96.0)); // aqui esta |
    wm=wm/4.5;
    timeold = millis();
    ISRCounter = 0;
  }

  delay(500);
  //Filtering
  wmF = doFilter1(wm);
  vmF = doFilter2(vm);
  imF = doFilter3(im);
  ibF = doFilter4(ib);
  vbF = doFilter5(vb);
  //Calculation

  wrpm = wmF * pi / (30 / 1.24137);
  vel = wrpm * 0.14;
  mrpm=(wm*1.5);



  vbF2 = vbF * 0.0008056641 + 0.01;
  voltageb = vbF2 / 0.055521879;

  ibF2 = ibF * 0.0008056641 + 0.01;
  currentb = 0.0042 * ibF2 + 2.624;

  vmF2 = vmF * 0.0008056641 + 0.01;
  voltagem = voltageb - vmF2 / 0.0543259557;

  imF2 = imF * 0.0008056641 + 0.01;
  currentm = 0.0042 * imF2 + 2.624;

  torque = currentm * 0.0981 + 0.2081;

  pmec = torque * wmF;
  pmot = voltagem * currentm;
  pbatt = voltageb * currentb;

  //SoC

  soc = 0.5314 * pow(voltageb, 3) - 80.77 * pow(voltageb, 2) + 4101.7 * voltageb - 69512;
  if (soc > 100) {
    soc = 100;
  }
  else
  {
    soc = soc;
  }


  //Discharge time

  dischargeTime = capacity * soc / (currentb * 100);
  dischargeF = doFilter6(dischargeTime);

  potenciometer = map(p, 0, 4096, 0, 254);  // read pot vealue and map it for a PWM scale
  throttle = map(potenciometer, 0, 253, 0, 100); // sabe but not for a 0 to 100% scale
  analogWrite(throttlePedalOut, potenciometer);  // output PWM

  //Serial.println(dischargeTime);
  //Serial.println(dischargeF);
  //String list[] = { String(voltageb), String(voltagem), String(currentm), String(currentb), String(torque), String(throttle), String(wm), String(vel), String(pmec), String(pmot), String(pbatt), String(soc), String(dischargeF)};

  //Serial.print(list[2] + "," + list[3] + "," + list[1] + "," + list[4] + "," + list[7] + "," + list[6] + "," + list[5] + "," + list[8] + "," + list[9] + "," + list[10] + ',' + list[11] + ',' + list[12] + ',' + list[13]);

  Serial.println(String (voltagem) + ',' + String(currentm) + ',' + String(voltageb) + ',' + String(currentb) + ',' + String(mrpm) + ',' + String(throttle) + ',' + String(torque) + ',' + String(vel) + ',' + String(pmec) + ',' + String(pmot) + ',' + String(pbatt) + ',' + String(soc) + ',' + String(dischargeF));
}


void doEncode()
{
  ISRCounter++;
}

void motorSensorRead() {
  im = analogRead(currentSensor);
  vm = analogRead(voltageSensor);
}

void batterySensorRead() {
  vb = analogRead(voltageBat);
  ib = analogRead(currentBat);
}

void throttleFun() {
  p = analogRead(throttlePedalIn);
}


float doFilter1(int x) {

  SUM1 = SUM1 - READINGS1[INDEX1]; // Remove the oldest entry from the sum
  READINGS1[INDEX1] = x; // Add the newest reading to the window
  SUM1 = SUM1 + x; // Add the newest reading to the sum
  INDEX1 = (INDEX1 + 1) % WINDOW_SIZE; // Increment the index, and wrap to 0 if it exceeds the window size
  result1 = SUM1 / WINDOW_SIZE; // Divide the sum of the window by the window size for the result

  return result1;
}

float doFilter2(int x) {

  SUM2 = SUM2 - READINGS2[INDEX2]; // Remove the oldest entry from the sum
  READINGS2[INDEX2] = x; // Add the newest reading to the window
  SUM2 = SUM2 + x; // Add the newest reading to the sum
  INDEX2 = (INDEX2 + 1) % WINDOW_S; // Increment the index, and wrap to 0 if it exceeds the window size
  result2 = SUM2 / WINDOW_S; // Divide the sum of the window by the window size for the result

  return result2;
}

float doFilter3(int x) {

  SUM3 = SUM3 - READINGS3[INDEX3]; // Remove the oldest entry from the sum
  READINGS3[INDEX3] = x; // Add the newest reading to the window
  SUM3 = SUM3 + x; // Add the newest reading to the sum
  INDEX3 = (INDEX3 + 1) % WINDOW_SIZE; // Increment the index, and wrap to 0 if it exceeds the window size
  result3 = SUM3 / WINDOW_SIZE; // Divide the sum of the window by the window size for the result

  return result3;
}
float doFilter4(int x) {

  SUM4 = SUM4 - READINGS4[INDEX4]; // Remove the oldest entry from the sum
  READINGS4[INDEX4] = x; // Add the newest reading to the window
  SUM4 = SUM4 + x; // Add the newest reading to the sum
  INDEX4 = (INDEX4 + 1) % WINDOW_SIZE; // Increment the index, and wrap to 0 if it exceeds the window size
  result4 = SUM4 / WINDOW_SIZE; // Divide the sum of the window by the window size for the result

  return result4;
}

float doFilter5(int x) {

  SUM5 = SUM5 - READINGS5[INDEX5]; // Remove the oldest entry from the sum
  READINGS5[INDEX5] = x; // Add the newest reading to the window
  SUM5 = SUM5 + x; // Add the newest reading to the sum
  INDEX5 = (INDEX5 + 1) % WINDOW_SIZE; // Increment the index, and wrap to 0 if it exceeds the window size
  result5 = SUM5 / WINDOW_SIZE; // Divide the sum of the window by the window size for the result

  return result5;
}

float doFilter6(int x) {

  SUM6 = SUM6 - READINGS6[INDEX6]; // Remove the oldest entry from the sum
  READINGS6[INDEX6] = x; // Add the newest reading to the window
  SUM6 = SUM6 + x; // Add the newest reading to the sum
  INDEX6 = (INDEX6 + 1) % WINDOW_S2; // Increment the index, and wrap to 0 if it exceeds the window size
  result6 = SUM6 / WINDOW_S2; // Divide the sum of the window by the window size for the result

  return result6;
}