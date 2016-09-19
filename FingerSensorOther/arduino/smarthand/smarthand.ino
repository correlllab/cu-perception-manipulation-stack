/* VCNL4000 Proximity and Ambient Light Sensor
   Example Code
   by: Jim Lindblom
   SparkFun Electronics
   date: August 30, 2011 (updated November 3, 2011)
   license: Beerware - Use this code however you want. If you find
   it useful you can by me a beer someday.

   There are a lot of parameters that you can play with on this sensor:
   * IR LED current: this can range from 0 (0mA, off) to 20 (200mA). Increase IR current,
   and you increase the sensitivity. Write to the IR_CURRENT register to adjust this
   value.
   * Ambient Light Parameter: You can adjust the number of measurements that are
   averaged into one output. By default the code averages the maximum, 128. Write to
   the AMBIENT_PARAMETER to change this value.
   * Proximity Signal Frequency: The frequency of the IR test signal can be either
   3.125 MHZ, 1.5625 MHz, 781.25 kHz, or 390.625kHz. Writing a 0, 1, 2, or 3 to the
   PROXIMITY_FREQ register will set this. The code sets it to 781.25 kHz by default.
   * Modulation delay and dead time: Not a lot of info in the datasheet on how
   to adjust these values. Write to the PROXIMITY_MOD register to change them. By
   default this register is set to 0x81 (129) as recommended by Vishay.

   Try to optimize the sensor for your application. Play around with those parameters.

   The sensor gets more and more sensitive as an object gets closer to it. The proximity
   output does not share a linear relationship with distance. The max distance is about
   20cm. Minimum distance is a few mm.

   Hardware:
   This code was developed using the SparkFun breakout board:
   http://www.sparkfun.com/products/10901
   Connections are:
   VCNL4000 Breakout ---------------- Arduino
        3.3V  -----------------------  3.3V
        GND  ------------------------  GND
        SCL  ------------------------  A5
        SDA  ------------------------  A4
        IR+  ------------------------  5V (3.3V is fine too)
 */
#include <Wire.h>

#define VCNL4000_ADDRESS 0x13  // 0x26 write, 0x27 read

// VCNL4000 Register Map
#define COMMAND_0 0x80  // starts measurments, relays data ready info
#define PRODUCT_ID 0x81  // product ID/revision ID, should read 0x11
#define IR_CURRENT 0x83  // sets IR current in steps of 10mA 0-200mA
#define AMBIENT_PARAMETER 0x84  // Configures ambient light measures
#define AMBIENT_RESULT_MSB 0x85  // high byte of ambient light measure
#define AMBIENT_RESULT_LSB 0x86  // low byte of ambient light measure
#define PROXIMITY_RESULT_MSB 0x87  // High byte of proximity measure
#define PROXIMITY_RESULT_LSB 0x88  // low byte of proximity measure
#define PROXIMITY_FREQ 0x89  // Proximity IR test signal freq, 0-3
#define PROXIMITY_MOD 0x8A  // proximity modulator timing

unsigned int ambientValue, proximityValue;

void setup()
{
  Serial.begin(9600);  // Serial's used to debug and print data
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  Wire.begin();  // initialize I2C stuff

  Wire.beginTransmission(112);
  Wire.write(0);
  Wire.endTransmission();


  // I2C mux at address 112

 for(int i=0;i<3;i++){
    Wire.beginTransmission(112);
    Wire.write(1 << i);
    Wire.endTransmission();

    /* Now some VNCL400 initialization stuff
       Feel free to play with any of these values, but check the datasheet first!*/
    writeByte(AMBIENT_PARAMETER, 0x0F);  // Single conversion mode, 128 averages
    writeByte(IR_CURRENT, 7);  // Set IR current to 200mA
    writeByte(PROXIMITY_FREQ, 6);  // 781.25 kHz
    writeByte(PROXIMITY_MOD, 0x81);  // 129, recommended by Vishay
  }
  
   for(int i=4;i<7;i++){
    Wire.beginTransmission(112);
    Wire.write(1 << i);
    Wire.endTransmission();

    /* Now some VNCL400 initialization stuff
       Feel free to play with any of these values, but check the datasheet first!*/
    writeByte(AMBIENT_PARAMETER, 0x0F);  // Single conversion mode, 128 averages
    writeByte(IR_CURRENT, 7);  // Set IR current to 200mA
    writeByte(PROXIMITY_FREQ, 6);  // 781.25 kHz
    writeByte(PROXIMITY_MOD, 0x81);  // 129, recommended by Vishay
  }
  
  Wire.beginTransmission(112);
  Wire.write(0);
  Wire.endTransmission();

  delay(100);

}

void loop()
{
  /* The loop just continuously reads the ambient and proximity values
  and spits them out over serial. */
  //  ambientValue = readAmbient();
  //if(Serial.read()=='A'){
   for(int i=0;i<3;i++){
      Wire.beginTransmission(112);
      Wire.write(1 << i);
      Wire.endTransmission();
      proximityValue = readProximity();
    //Serial.println(ambientValue, DEC);
      Serial.print(proximityValue, DEC);
      Serial.print("\t");
    }
    for(int i=4;i<5;i++){
      Wire.beginTransmission(112);
      Wire.write(1 << i);
      Wire.endTransmission();
      proximityValue = readProximity();
    //Serial.println(ambientValue, DEC);
      Serial.print(proximityValue, DEC);
      Serial.print("\t");
    }
    Serial.println();
    Wire.beginTransmission(112);
    Wire.write(0);
    Wire.endTransmission();
   

  // delay(100);  // You may want to uncomment this for visibility
}

// readProximity() returns a 16-bit value from the VCNL4000's proximity data registers
unsigned int readProximity()
{
  unsigned int data;
  byte temp;

  temp = readByte(COMMAND_0);
  writeByte(COMMAND_0, temp | 0x08);  // command the sensor to perform a proximity measure

  while (!(readByte(COMMAND_0) & 0x20))
    ;  // Wait for the proximity data ready bit to be set
  data = readByte(PROXIMITY_RESULT_MSB) << 8;
  data |= readByte(PROXIMITY_RESULT_LSB);

  return data;
}

// readAmbient() returns a 16-bit value from the VCNL4000's ambient light data registers
unsigned int readAmbient()
{
  unsigned int data;
  byte temp;

  temp = readByte(COMMAND_0);
  writeByte(COMMAND_0, temp | 0x10);  // command the sensor to perform ambient measure

  while (!(readByte(COMMAND_0) & 0x40))
    ;  // wait for the proximity data ready bit to be set
  data = readByte(AMBIENT_RESULT_MSB) << 8;
  data |= readByte(AMBIENT_RESULT_LSB);

  return data;
}

// writeByte(address, data) writes a single byte of data to address
void writeByte(byte address, byte data)
{
  Wire.beginTransmission(VCNL4000_ADDRESS);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

// readByte(address) reads a single byte of data from address
byte readByte(byte address)
{
  byte data;

  Wire.beginTransmission(VCNL4000_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(VCNL4000_ADDRESS, 1);
  while (!Wire.available())
    ;
  data = Wire.read();

  return data;
}

