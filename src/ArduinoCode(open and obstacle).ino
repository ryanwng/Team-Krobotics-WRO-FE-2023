/* CODE EXPLANATION

The Arduino code works in tandem with the Raspberry Pi in a master-slave relationship. 
The Pi calculates everything and sends the value of the speed and angles to the Arduino by giving it 4 digit numbers
The Arduino then translates whether the value given is a speed or an angle, and then communicates that information to the motor.

*/

#include <Servo.h>
Servo BLDCMotor;
Servo servoMotor;
char buff[5]; //Var which will help us differentiate the start and end of information


void setup(){
  Serial.begin(115200); //Baud rate == 115200
  servoMotor.attach(11); // White wire for servo motor
  servoMotor.write(90); //Sets servo to be straight (90 degrees)
  BLDCMotor.attach(13, 1000, 2000); // Yellow wire
  BLDCMotor.writeMicroseconds(1500); //Delay to wait for motor to be ready
}


void loop(){
  if(Serial.available() > 0) //If Arduino recieves a signal from the Pi
  {
    Serial.readBytes(buff,5);
    buff[4] = '\0'; //end of byte, so the Arduino know when the start and end of a command is
    int val = atoi(buff); //converts string to int

    if(val <= 1900 && val >= 1400) // Determines if the number is speed
    {
      BLDCMotor.write(val); // > 1500 forward, < 1500 backwards
    }
    else if(val > 2000 && val <= 2180) // Determines if the number is an angle
    {                                                                                                       
      servoMotor.write(val - 2000); // 2045 left, 2090 straight, 2135 right
    }
  }
}

