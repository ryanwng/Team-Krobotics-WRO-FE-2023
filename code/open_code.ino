#include <Servo.h>

Servo BLDCMotor;
Servo servoMotor;
int incomingByte = 0;
char buff[10];


void setup(){
  Serial.begin(9600);
  servoMotor.attach(9);
  servoMotor.write(90);
  BLDCMotor.attach(10, 1000, 2000);
  BLDCMotor.writeMicroseconds(1500);
  delay(6000);//mandatory delay waiting for motor to be ready
}


void loop(){
  if(Serial.available() > 0)
  {
    Serial.readBytes(buff,5);
    buff[4] = '\0';
    int val = atoi(buff);
    Serial.println(val);

    if(val <= 2000 && val >= 1000)x
    {
      BLDCMotor.write(val); // < 1500 counter-clockwise; > 1500 clockwise
    }
    else if(val > 2000 && val <= 2180)
    {
      servoMotor.write(val - 2000);
    }
  }
}
