#include <VarSpeedServo.h>

VarSpeedServo myServo;
#define PIN_SERVO (8)
int ii;
 
void SetStrokePerc(float strokePercentage)
{
  if ( strokePercentage >= 1.0 && strokePercentage <= 99.0 )
  {
    int usec = 1000 + strokePercentage * ( 2000 - 1000 ) / 100.0 ;
    myServo.writeMicroseconds( usec );
  }
}
void SetStrokeMM(int strokeReq,int strokeMax)
{
  SetStrokePerc( ((float)strokeReq) / strokeMax );
}
 
 
void setup() 
{ 
  myServo.attach(PIN_SERVO);
} 
  
 
void loop() 
{ 
 for (ii=1000; ii<=2000; ii+=10)
   { 
     myServo.writeMicroseconds( ii );
     delay(1500);   
     while (ii==2000){
     myServo.writeMicroseconds( ii );
   }
   
   }
  myServo.writeMicroseconds(1050);
}
