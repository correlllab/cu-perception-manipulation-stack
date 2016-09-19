/*
 * Author(s) : Andy McEvoy ( mcevoy.andy@gmail.com )
 * Desc      : Simple controller for Firgelli 12-100-50-12-p using an Adafruit MotorShield v1.2
 * Copyright 2015 Andy McEvoy
 */

#include <Wire.h>
#include <AFMotor.h>

/*
 * Feedback potentiometer negative reference rail = GND
 * Feedback potentiometer positive reference rail = 5V
 */

// GLOBAL VARIABLES:
int pot_pin_ = 8; // feedback potentiometer wiper (position signal)
int pot_value_ = 0;
int motor_port_ = 4; // port (M1 - M4) that the motor is plugged into 

int padding = 10;
int upper_limit_ = 1012 - padding;
int lower_limit_ = 10 + padding;
int direction_ = 0;

AF_DCMotor linear_actuator_(motor_port_,MOTOR12_64KHZ);


void setup()
{
  Serial.begin(9600);
  printWelcomeMsg();
 
  linear_actuator_.setSpeed(200);
}

void loop()
{
  int i;

  gotoPosition(upper_limit_);
  delay(1000);

  gotoPosition(lower_limit_);
  delay(1000);

  gotoPosition( (upper_limit_ - lower_limit_) / 2 );
  
  linear_actuator_.run(RELEASE);
  delay(1000);
}

void printWelcomeMsg()
{
  Serial.println("Simple Firgelli controller");

  Serial.print("Motor port = ");
  Serial.print(motor_port_);
  Serial.println();

  delay(1000);
}

void gotoPosition(int position)
{
  if (position > upper_limit_ || position < lower_limit_)
  {
    Serial.println("WARNING: tried to set position outside of limits");
  }

  if (pot_value_ >= position)
  {
    linear_actuator_.run(BACKWARD);
    while (pot_value_ > position)
    {
      pot_value_ = analogRead(pot_pin_);
    }
    linear_actuator_.run(RELEASE);
  }
  else
  {
    linear_actuator_.run(FORWARD);
    while (pot_value_ < position)
    {
      pot_value_ = analogRead(pot_pin_);
    }
    linear_actuator_.run(RELEASE);
  }

  pot_value_ = analogRead(pot_pin_);
  Serial.print("position (set / actual) = ");
  Serial.print(position);
  Serial.print(" / ");
  Serial.print(pot_value_);
  Serial.println();
}
