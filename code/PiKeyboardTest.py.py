#!/usr/bin/env python3
import serial
from readchar import readkey, key
from time import sleep

if __name__ == '__main__':
    '''
    timeout: this is a timeout for read operations. Here we set it to 1 second. 
    It means that when we read from Serial, the program won’t be stuck forever if the data is not coming. 
    After 1 second or reading, if not all bytes are received, the function will return the already received bytes.
    '''
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1) #approximately 960 characters per second
    ser.flush() #block the program until all the outgoing data has been sent
    speed = 1500
    angle = 2090
    while True:
        k = readkey()
        
        if k == "w":
            if(speed > 1400):
                speed -= 1  #speed up
            ser.write((str(speed) + "\n").encode('utf-8'))
            print("speed: ", speed)
            
        if k == "x":
            if (speed < 1500):
                speed += 1  #speed down
            ser.write((str(speed) + "\n").encode('utf-8'))
            print("speed: ", speed)
           
        if k == key.SPACE:
            speed = 1500
            ser.write((str(speed) + "\n").encode('utf-8'))
            print("speed: ", speed)
        
        if k == "a":
            if(angle < 2135): #2090 + 45
                angle += 1  #turn left
            ser.write(((str(angle) + "\n").encode('utf-8')))
            print("angle: ", angle)
            
        if k == "d":
            if(angle > 2045): #2090 - 45
                angle -= 1  #turn right
            ser.write(((str(angle) + "\n").encode('utf-8')))
            print("angle: ", angle)
        
        if k == "s":
            angle = 2090
            ser.write(((str(angle) + "\n").encode('utf-8')))
            print("angle: ", angle)
    
        sleep(0.01)
        
        
        
        
 
        
        
        

