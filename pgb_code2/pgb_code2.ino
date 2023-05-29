
#include <DueTimer.h>

//el santiago no hizo nada
//initial vairables

float temporal = 0;
float wrpm;
float vel;
int potenciometer;

float rpm = 0;
unsigned long timeold;

long pi = 3.1415926536;
long wm = 0.0;
float kv = 8.9460;
float torque = 0.0;
float pmec = 0.0;
float pmot = 0.0;
float pbatt = 0.0;

volatile int ISRCounter = 0;
int counter = 0;
long deltaTime = 0;

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
float currentm = 0.0;
float currentb = 0.0;
float voltagem = 0.0;
float voltageb = 0.0;

//pin assigments
#define channelPinA 2
#define channelPinB 3
#define throttlePedalOut 9

#define throttlePedalIn A0
#define currentSensor A1
#define voltageSensor A2
#define voltageBat A3
#define currentBat A4

//Filter assignments

#define WINDOW_SIZE 5
int INDEX1 = 0;
int SUM1 = 0;
int READINGS1[WINDOW_SIZE];

int INDEX2 = 0;
int SUM2 = 0;
int READINGS2[WINDOW_SIZE];

int INDEX3 = 0;
int SUM3 = 0;
int READINGS3[WINDOW_SIZE];

int INDEX4 = 0;
int SUM4 = 0;
int READINGS4[WINDOW_SIZE];

int INDEX5 = 0;
int SUM5 = 0;
int READINGS5[WINDOW_SIZE];

void setup() {

  analogReadResolution(12);
  Serial.begin(115200);

  //Input for encoders
  pinMode(channelPinA, INPUT_PULLUP);  // First signal for encoder
  pinMode(channelPinB, INPUT_PULLUP);  // Second signal for encoder

  //timed analog reads
  Timer3.attachInterrupt(batterySensorRead).setFrequency(200).start();
  Timer5.attachInterrupt(motorSensorRead).setFrequency(200).start();
  Timer4.attachInterrupt(throttle).setFrequency(200).start();

  attachInterrupt(digitalPinToInterrupt(channelPinA), doEncode, RISING);

}

void loop() {
  Serial.println(" "); //This could be avoided but apparently \n scares santi

  //Second axle RPM calculation

  if (millis() - timeold >= 200) {

    temporal = ISRCounter * 60000; //(millis()-timeold);
    wm = float(temporal / ((millis() - timeold) * 48.0)); // aqui esta
    //Serial.println(wm);
    timeold = millis();
    ISRCounter = 0;
  }

  SUM1 = SUM1 - READINGS1[INDEX1]; // Remove the oldest entry from the sum
  READINGS1[INDEX1] = wm; // Add the newest reading to the window
  SUM1 = SUM1 + wm; // Add the newest reading to the sum
  INDEX1 = (INDEX1 + 1) % WINDOW_SIZE; // Increment the index, and wrap to 0 if it exceeds the window size
  wmF = SUM1 / WINDOW_SIZE; // Divide the sum of the window by the window size for the result


  delay(500);
  String outRPM1 = String("Motor velocity en[rpm]: ") + wmF;
  //Serial.println(outRPM1);

  //Motor RPM
  rpm = voltagem * kv;
  String outRPM2 = String("Motor velocity vm[rpm]: ") + rpm;
  //Serial.println(outRPM2);
  //Signal aquisition and filtering
  //Motor Voltage

  voltagem = map(vmF, 0, 4096, 0, 3.33);
  voltagem = -20.571 * voltagem + 55.76; // assigns the voltage value to the adc lecture
  String outVOLM = String("Motor voltage      [V]: ") + voltagem;
  //Serial.println(outVOLM);

  SUM2 = SUM2 - READINGS2[INDEX2]; // Remove the oldest entry from the sum
  READINGS2[INDEX2] = vm; // Add the newest reading to the window
  SUM1 = SUM1 + vm; // Add the newest reading to the sum
  INDEX2 = (INDEX2 + 1) % WINDOW_SIZE; // Increment the index, and wrap to 0 if it exceeds the window size
  vmF = SUM2 / WINDOW_SIZE; // Divide the sum of the window by the window size for the result

  //Battery Voltage
  voltageb = map(vb, 0, 3353, 0, 52.8);   //same
  //String outVOLB = String("Battery voltage    [V]: ") + voltageb;
  //Serial.println(outVOLB);

  SUM3 = SUM3 - READINGS3[INDEX3]; // Remove the oldest entry from the sum
  READINGS3[INDEX3] = vb; // Add the newest reading to the window
  SUM3 = SUM3 + vb; // Add the newest reading to the sum
  INDEX3 = (INDEX3 + 1) % WINDOW_SIZE; // Increment the index, and wrap to 0 if it exceeds the window size
  vbF = SUM3 / WINDOW_SIZE; // Divide the sum of the window by the window size for the result

  //Motor Current
  float imF2 = map(im, 0, 4096, 0, 3.3);      //same
  currentm = 0.0042 * imF2 + 2.624;           //BUT it converts the voltage to the corresponding current

  //String outIM = String("Motor current      [A]: ") + im;
  //Serial.println(outIM);

  SUM4 = SUM4 - READINGS4[INDEX4]; // Remove the oldest entry from the sum
  READINGS4[INDEX4] = im; // Add the newest reading to the window
  SUM4 = SUM4 + im; // Add the newest reading to the sum
  INDEX4 = (INDEX4 + 1) % WINDOW_SIZE; // Increment the index, and wrap to 0 if it exceeds the window size
  imF = SUM4 / WINDOW_SIZE; // Divide the sum of the window by the window size for the result


  //Battery current
  float ibF2 = map(ib, 0, 4096, 0, 3.33);    //same
  currentb = 0.0042 * ibF2 + 2.624;

  //String outIB = String("Battery current    [A]: ") + currentb;
  //Serial.println(outIB);

  SUM5 = SUM5 - READINGS5[INDEX5]; // Remove the oldest entry from the sum
  READINGS5[INDEX5] = ib; // Add the newest reading to the window
  SUM5 = SUM5 + ib; // Add the newest reading to the sum
  INDEX5 = (INDEX5 + 1) % WINDOW_SIZE; // Increment the index, and wrap to 0 if it exceeds the window size
  ibF = SUM5 / WINDOW_SIZE; // Divide the sum of the window by the window size for the result


  //Motor torque
  torque = 0.0981 * currentm + 0.2081;      // using kt*Im we use the current to calculate the motor mechanical torque
  //String outT = String("Motor torque      [Nm]: ") + torque;
  //Serial.println(outT);

  //Pedal
  //String outP = String("Throttle           [%]: ") + potenciometer;
  potenciometer = map(p, 0, 4096, 0, 254);  // read pot vealue and map it for a PWM scale
  float throttle = map(potenciometer, 0, 4096, 0, 100); // sabe but not for a 0 to 100% scale
  analogWrite(throttlePedalOut, potenciometer);  // output PWM
  //Serial.println(outP);

  //Power
  pmot = voltagem * currentm;      // P = V*I
  pbatt = voltageb * currentb;     // ... same
  wrpm = (wm / 60) * (2 * pi);     // angular velocity in rad/s
  pmec = (wrpm * pi / 30) * torque ;  //rpm turned into rad/s
  vel = wrpm * (0.14);              // wheel diameter 28cm, linear velocity wr

  // comparacion 
  Serial.println(wm);
  Serial.println(wmF);
  /*Serial.println(ib);
  Serial.println(ibF);
  Serial.println(vb);
  Serial.println(vbF);
  Serial.println(im);
  Serial.println(imF);
  Serial.println(vm);
  Serial.println(vmF);*/

  //With this list we send the data packet to the raspberry pi
  String list[] = { String(voltageb), String(voltagem), String(currentm), String(currentb), String(torque), String(throttle), String(wm), String(vel), String(pmec), String(pmot), String(pbatt)};
  // list[11] invokes the ancient ones so please do not use it
  //Serial.println("VM  ,IM  , VB   ,IB  ,RPM ,TH ,TQ  ,VEL ,PMC ,PMT ,PBT ");
  //Serial.print(list[2]+","+list[3]+","+list[1]+","+list[4]+","+list[7]+","+list[6]+","+list[5]+","+list[8]+","+list[9]+","+list[10]+','+ String(pbatt));  //Do this instead
}

//Timed reads
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

//Count rpm
void doEncode()
{

  ISRCounter++;

}
