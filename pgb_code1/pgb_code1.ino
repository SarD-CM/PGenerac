#include <filters.h>
#include <Filter.h>
#include <DueTimer.h>

//el santiago no hizo nada
//initial vairables
float revolutions = 0;
int rpm = 0;  // max value 32,767 16 bit
long startTime = 0;
long elapsedTime;
float wrpm;
float vel;
bool s;
float temp;
int potenciometer;
int so;

long pi = 3.1415926536;
long wm=0.0;
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

float currentm = 0.0;
float currentb = 0.0;

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
ExponentialFilter<float> filteredBcurrent(20, 0);
ExponentialFilter<float> filteredMcurrent(20, 0);
ExponentialFilter<float> filteredBvoltage(10, 0);
ExponentialFilter<float> filteredMvoltage(90, 0);

const float cutoff_freq   = 20.0;  //Cutoff frequency in Hz
const float sampling_time = 0.005; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD3; // Order (OD1 to OD4)

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
  Timer6.attachInterrupt (doEncode).setFrequency(400).start();

}

void loop() {
  Serial.println(" "); //This could be avoided but apparently \n scares santi

  //filtering
  filteredBvoltage.Filter(vb);        //Battery voltage filter
  vbF = filteredBvoltage.Current();   

  filteredBcurrent.Filter(ib);        //Battery current filter
  ibF = filteredBcurrent.Current();   

  filteredMcurrent.Filter(im);        // Motor current filter
  imF = filteredMcurrent.Current();

  filteredMvoltage.Filter(vm);        // Motor voltage filter
  vmF = filteredMvoltage.Current();

  //RPM calculation
  //detachInterrupt(E2);
  //detachInterrupt(E1);
  
  if (ISRCounter!=0) {
    deltaTime = millis() - timeCounter;
    wm = max(1,ISRCounter) * (1/400);
  }
  delay(500);
  //String outRPM = String("Motor velocity   [rpm]: ") + wm;
  //Serial.println(outRPM);

  //Motor Voltage

  float voltagem = map(vm, 3262, 3200, 0, 5.79);   // assigns the voltage value to the adc lecture
  //String outVOLM = String("Motor voltage      [V]: ") + vmF;
  //Serial.println(outVOLM);

  //Battery Voltage
  float voltageb = map(vb, 0, 3353, 0, 52.8);   //same
  //String outVOLB = String("Battery voltage    [V]: ") + voltageb;
  //Serial.println(outVOLB);

  //Motor Current
  float imF2 = map(im, 0, 4096, 0, 3.3);      //same
  currentm = 0.0042 * imF2 + 2.624;           //BUT it converts the voltage to the corresponding current

  //String outIM = String("Motor current      [A]: ") + im;
  //Serial.println(outIM);

  //Battery current
  float ibF2 = map(ib, 0, 4096, 0, 3.33);    //same
  currentb = 0.0042 * ibF2 + 2.624;

  //String outIB = String("Battery current    [A]: ") + currentb;
  //Serial.println(outIB);

  //Motor torque
  torque = 0.0981 * currentm + 0.2081;      // using kt*Im we use the current to calculate the motor mechanical torque
  //String outT = String("Motor torque      [Nm]: ") + torque;
  //Serial.println(outT);

  //Pedal
  //String outP = String("Throttle           [%]: ") + potenciometer;
  potenciometer = map(p, 0, 4096, 0, 245);  // read pot vealue and map it for a PWM scale
  float throttle = map(p, 0, 4096, 0, 100); // sabe but not for a 0 to 100% scale
  analogWrite(throttlePedalOut, potenciometer);  // output PWM
  //Serial.println(outP);

  //Power
  pmec = rpm * torque * pi / 30;   //rpm turned into rad/s
  pmot = voltagem * currentm;      // P = V*I
  pbatt = voltageb * currentb;     // ... same
  wrpm = (rpm/60)*(2*pi)           // angular velocity in rad/s
  vel = wrpm*(0.14)                // wheel diameter 28cm, linear velocity wr
  //With this list we send the data packet to the raspberry pi
  String list[] = { String(voltageb), String(voltagem), String(currentm), String(currentb), String(torque), String(throttle), String(rpm), String(vel), String(pmec), String(pmot), String(pbtt)};
  // list[11] invokes the ancient ones so please do not use it 
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
  if (digitalRead(channelPinA) == digitalRead(channelPinB))
  {
    ISRCounter++;
  }
  else
  {
    ISRCounter--;
  }
  timeCounter = millis();
}
