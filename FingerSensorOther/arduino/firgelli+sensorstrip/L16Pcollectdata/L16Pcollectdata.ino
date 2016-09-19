#include <AFMotor.h>
#include <Wire.h>

AF_DCMotor linearActuator(4);

//===== GLOBAL CONSTANTS =====//ACTUATOR
#define LINEAR_POT_PIN A1 //1013<blue lead <0
#define LINEAR_POS_REF_PIN A0 //green lead
#define LINEAR_NEG_REF_PIN A2 //yellow lead
//====== GLOBAL CONSTANTS =====//SENSOR 
#define VCNL4010_ADDRESS 0x13
#define COMMAND_0 0x80  // starts measurements, relays data ready info
#define PRODUCT_ID 0x81  // product ID/revision ID, should read 0x21
#define IR_CURRENT 0x83  // sets IR current in steps of 10mA 0-200mA
#define AMBIENT_PARAMETER 0x84  // Configures ambient light measures
#define AMBIENT_RESULT_MSB 0x85  // high byte of ambient light measure
#define AMBIENT_RESULT_LSB 0x86  // low byte of ambient light measure
#define PROXIMITY_RESULT_MSB 0x87  // High byte of proximity measure
#define PROXIMITY_RESULT_LSB 0x88  // low byte of proximity measure
#define PROXIMITY_MOD 0x8F  // proximity modulator timing

/***** GLOBAL VARIABLES *****/
int num_devices_;
unsigned int ambient_value_;
unsigned int proximity_value_;


//===== USER PARAMETERS =====//ACTUATOR
float linearActuatorTolerance = 0.13;
float actuatorMax = 137.0;
float currentlength = 0.;
float goallength = 0;
float length = 0;
int pinValue = 0;
//===== USER PARAMETERS =====//SENSOR
int i2c_ids_[] = {112};
int ir_current_ = 2; // range = [0, 20]. current = value * 10 mA
int ambient_light_measurement_rate_ = 7; // range = [0, 7]. 1, 2, 3, 4, 5, 6, 8, 10 samples per second
int ambient_light_auto_offset_ = 1; // on or off
int averaging_function_ = 7;  // range [0, 7] measurements per run are 2**value, with range [1, 2**7 = 128]
int proximity_freq_ = 2; // range = [0 , 3]. 390.625kHz, 781.250kHz, 1.5625MHz, 3.125MHz 



//===== FUNCTION DEF =====//ACTUATOR
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
  linearActuator.setSpeed(150);
  currentlength = getLength();
   
//  Serial.print("CL: ");
//  Serial.println(currentlength);
//  Serial.print("GL: ");
//  Serial.println(length);
  
  float oldLength = currentlength;
  int count = 0;
  while(abs(currentlength - length) >= linearActuatorTolerance) {
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
  //Serial.println("--");
}  


//===== FUNCTION DEF =====//SENSOR
void readSensorStripValues(int id)///////////////////////////////////////////////////////////////
{
  char buf[8];
  Serial.print(id);
  // read all 8 sensors on the strip
  for (int i = 3; i < 8; i++)
  {
    Wire.beginTransmission(id);
    Wire.write(1 << i);
    Wire.endTransmission();
    
    ambient_value_ = readAmbient();
    proximity_value_ = readProximity();

    Serial.print(", ");
    sprintf(buf, "%6d", ambient_value_);
    Serial.print(buf);
    Serial.print(", ");
    sprintf(buf, "%6d", proximity_value_);
    Serial.print(buf);
  }
  Serial.println();
}

unsigned int readProximity(){   //////////////////////////////////////////////////////////////////
  byte temp = readByte(0x80);
  writeByte(0x80, temp | 0x08);  // command the sensor to perform a proximity measure

  while(!(readByte(0x80) & 0x20));  // Wait for the proximity data ready bit to be set
  unsigned int data = readByte(0x87) << 8;
  data |= readByte(0x88);

  return data;
}

unsigned int readAmbient(){   ///////////////////////////////////////////////////////////////////
  byte temp = readByte(0x80);
  writeByte(0x80, temp | 0x10);  // command the sensor to perform ambient measure

  while(!(readByte(0x80) & 0x40));  // wait for the proximity data ready bit to be set
  unsigned int data = readByte(0x85) << 8;
  data |= readByte(0x86);

  return data;
}

void initSensorStrip(int id)//////////////////////////////////////////////////////////////////////
{
  Wire.beginTransmission(id);
  Wire.write(0);
  Wire.endTransmission();

  // initialize each IR sensor on the strip (normally 8 of them)
  for (int i = 3; i < 8; i++)
  {
    // specify IR sensor
    Wire.beginTransmission(id);
    Wire.write(1 << i);
    Wire.endTransmission();
 
    delay(50); 

    byte temp = readByte(PRODUCT_ID);
    //byte proximityregister = readByte(IR_CURRENT);
    //Serial.println(proximityregister);
    if (temp != 0x21){  // Product ID Should be 0x21 for the 4010 sensor
      Serial.print("IR sensor failed to initialize: id = ");
      Serial.print(i);
      Serial.print(". ");
      Serial.println(temp, HEX);
    }
    else
    {
      Serial.print("IR sensor online: id = ");
      Serial.println(i);
    }
    //Feel free to play with any of these values, but check the datasheet first!
    writeByte(AMBIENT_PARAMETER, 0x7F);
    writeByte(IR_CURRENT, ir_current_);
    writeByte(PROXIMITY_MOD, 1); // 1 recommended by Vishay
  }
  Wire.beginTransmission(id);
  Wire.write(0);
  Wire.endTransmission();

}

void writeByte(byte address, byte data)///////////////////////////////////////////////////////////
{
  Wire.beginTransmission(VCNL4010_ADDRESS);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

byte readByte(byte address){/////////////////////////////////////////////////////////////////////
  Wire.beginTransmission(VCNL4010_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(VCNL4010_ADDRESS, 1);
  while(!Wire.available());
  byte data = Wire.read();
  return data;
}





void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  pinMode(LINEAR_POT_PIN, INPUT);
  //pinMode(LINEAR_POS_REF_PIN, INPUT);
  //digitalWrite(LINEAR_POS_REF_PIN, HIGH);
  //pinMode(LINEAR_NEG_REF_PIN, INPUT);
  //digitalWrite(LINEAR_NEG_REF_PIN, LOW);
  Wire.begin();

  // clear serial
  Serial.println();
  Serial.println();
  delay(2000);

  // get number of i2c devices specified by user
  num_devices_ = sizeof(i2c_ids_) / sizeof(int);
  Serial.print("Attached i2c devices: ");
  Serial.println(num_devices_);

  // initialize attached devices
  for (int i = 0; i < num_devices_; i++)
  {
    Serial.print("Initializing IR Sensor Strip: ");
    Serial.println(i2c_ids_[i]);
    initSensorStrip(i2c_ids_[i]);
  }
  Serial.println("Starting main loop...");
  Serial.println("strip id, ambient value[0], proximity value[0], ambient value[1], ... ");
  delay(100);
}



void loop() {
 
    // Read sensor values and increment the actuator' length 
//  for (int i = 0; i < num_devices_; i++)
//  {
//    setLinearActuator(6);
//    for (int j=72; j<=137; j+=2 )
//    {
//      setLinearActuator(j);
//      delay(1000);
//      readSensorStripValues(i2c_ids_[i]);
//      //Serial.println("--");
//      delay(500);
//    }
//  }
      for (int i = 0; i < num_devices_; i++)
   {
      setLinearActuator(72);
    for (int j=1; j<=50; j+=1 )
    {
     
      linearActuator.run(FORWARD);
      delay(500);
      readSensorStripValues(i2c_ids_[i]);
      delay(500);
    }
  }
  
}
