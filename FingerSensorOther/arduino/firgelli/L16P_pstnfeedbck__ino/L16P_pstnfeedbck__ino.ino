/*Firgelli Linear actuator code. L-16-140mm-positional Accuray=0.5mm*/

#include <AFMotor.h>

AF_DCMotor linearActuator(4);

#define LINEAR_POT_PIN A1 //1013<blue lead <0
#define LINEAR_POS_REF_PIN A0 //green lead
#define LINEAR_NEG_REF_PIN A2 //yellow lead

float linearActuatorTolerance = 0.5; //specified in the datasheet
float actuatorMax = 137.0;
float currentlength = 0.;
float goallength = 0;
float length = 0;
int pinValue = 0;

float mmToADC(float distance) {
  // Cubic fit to the calibration measurements
  return distance*(8.4-distance*(0.005-distance*0.000014));
}

float ADCtomm(int adc) {
  // Cubic fit to the calibration measurements
  return adc*(0.12+adc*(3.44e-6+adc*1.0e-8));
}

float getLength() {
  return ADCtomm(analogRead(LINEAR_POT_PIN));
}

void setLinearActuator(float length) {
  if (length > actuatorMax) length = actuatorMax;
  if (length < 0) length = 0;
  linearActuator.setSpeed(200);
  currentlength = getLength();
   
  Serial.print("CL: ");
  Serial.println(currentlength);
  Serial.print("GL: ");
  Serial.println(length);
   
  float oldLength = currentlength;
  int count = 0;
  while(abs(currentlength - length) > linearActuatorTolerance) {
    currentlength = getLength();
    if (currentlength == oldLength) count++;
    else count = 0;
    
    if (count > 500) 
    {
      linearActuator.run(RELEASE);
      linearActuator.setSpeed(0);
      Serial.print("D");
      return;
    }
    
    if (length > currentlength) linearActuator.run(BACKWARD);
    else linearActuator.run(FORWARD);
  }
  linearActuator.run(RELEASE);
  linearActuator.setSpeed(0);
  Serial.print("D");
}  

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  pinMode(LINEAR_POT_PIN, INPUT);
  //pinMode(LINEAR_POS_REF_PIN, INPUT);
  digitalWrite(LINEAR_POS_REF_PIN, HIGH);
  //pinMode(LINEAR_NEG_REF_PIN, INPUT);
  digitalWrite(LINEAR_NEG_REF_PIN, LOW);
}

void loop() {
  setLinearActuator(6.0);
  setLinearActuator(137.0);
}
