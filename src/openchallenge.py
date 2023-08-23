import cv2
from time import sleep
import numpy as np
from picamera2 import Picamera2
import serial
#import RPi.GPIO as GPIO

if __name__ == '__main__':

#    GPIO.setwarnings(False)
#    GPIO.setmode(GPIO.BOARD)
#    GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    ''' #Test later in order to see how the button works
        while(True):
        if GPIO.input(5) == GPIO.LOW: #wait for button to be pressed
            break
    '''

    #Intializing the camera
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640,480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()
    width = 640
    height = 480
    kd = 0
    kp = 0.004
    angle = 2090
    count = 0
    turns = 0
    leftn = False
    rightn = False
    pasterror = 0
    #points = [(115,100), (525,100), (640,470), (0,470)]

    points = [(0,0),(640,0),(640,480),(0,480)]

    ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 1) #approximately 57600 characters per second
    ser.flush()
    sleep(8)	    
    
    speed = 1660 #faster speed to accelerate
    ser.write((str(speed) + "\n").encode('utf-8'))
    count = 0
    counted = False
    while True:
        im= picam2.capture_array()


        input = np.float32(points)
        output = np.float32([(0,0), (width-1, 0), (width-1,height-1), (0, height-1)])

        matrix = cv2.getPerspectiveTransform(input,output)
        imgPerspective = cv2.warpPerspective(im, matrix, (width, height),
                                            cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))



        # Find contours in the edge image
        imgGray = cv2.cvtColor(imgPerspective, cv2.COLOR_BGR2GRAY)
        ret, imgThresh = cv2.threshold(imgGray, 30, 255, cv2.THRESH_BINARY_INV)
        # First coord is top left, second coord is bottom right.
        
        lft = [[0,310],[220,480]]

        right = [[420,305],[640,480]]

        contoursLeft, _ = cv2.findContours(imgThresh[lft[0][1]:lft[1][1],lft[0][0]:lft[1][0]], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contoursRight, _ = cv2.findContours(imgThresh[right[0][1]:right[1][1],right[0][0]:right[1][0]], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Offset based off topleft corner of each left/right rectrangle
        contoursLeft = [ c + lft[0] for c in contoursLeft]
        contoursRight = [ c + right[0] for c in contoursRight]
       

        # Draw contours on the original image
        cv2.drawContours(im, contoursLeft, -1, (0, 255, 0), 2)
        cv2.drawContours(im, contoursRight, -1, (0, 255, 0), 2)
     
        lftRectColor = (255,0,0) # Blue
        rtRectColor = (255,0,0) #Still blue
       
        lftTot = 0
        for c in contoursLeft:
            lftTot += cv2.contourArea(c)

        rtTot = 0
        for c in contoursRight:
            rtTot += cv2.contourArea(c)
        

        error = lftTot-rtTot
        d = error - pasterror 
        steering = kp * error + kd * d 
        steering = int(steering)
        pasterror = error
        print("Steering:",steering)
        if angle > 2180 or angle < 2000:
            angle = 2090
        
        
        if (lftTot <= 2000 and rtTot >= 2000 and error < -1700) or (lftTot <= 3000 and rtTot <= 9000 and error < -5500) or (lftTot <= 7000 and rtTot >= 13000 and error < -6000):
            angle = 2050 + steering
            print(lftTot <= 2000 and rtTot >= 2000 and error < -1700)
            print(lftTot <= 3000 and rtTot <= 9000 and error < -5500)
            print(lftTot <= 7000 and rtTot >= 13000 and error < -6000)
            if not counted:
                count += 1
            counted = True
            rightn = True
        elif (rtTot <= 2000 and lftTot >= 2000 and error > 1700) or (rtTot <= 3000 and lftTot <= 9000 and error > 5500) or (rtTot <= 7000 and lftTot >= 13000 and error > 6000):
            angle = 2130 + steering
            print(rtTot <= 2000 and lftTot >= 2000 and error > 1700)
            print(rtTot <= 3000 and lftTot <= 9000 and error > 5500)
            print(rtTot <= 7000 and lftTot >= 13000 and error > 6000)
            if not counted:
                count+=1
            counted = True
            leftn = True
        elif error > 450 or error < -450 and (rightn== True or leftn == True):
            counted = False
            angle = 2090
            if steering > 45:
                steering = 45
            elif steering < -45:
                steering = -45 
            angle += steering
            leftn = False
            rightn = False
        if count >= 1200: 
            print("stopping")
            ser.flush()
            speed = 1500
            angle = 2090
            ser.write((str(speed) + "\n").encode('utf-8'))
            ser.write((str(angle) + "\n").encode('utf-8'))
            break
            cv2.destroyAllWindows()
        
        
        angle = int(angle)    
        if angle > 2160:
            angle = 2160
        elif angle < 2020:
            angle = 2020
        
        sleep(0.1)
        print("Angle:",angle)
        print("Error:",error)
        print("Turns:", count)
    
        ser.write((str(angle) + "\n").encode('utf-8'))

        cv2.rectangle(im,tuple(lft[0]),tuple(lft[1]),lftRectColor,2)
        cv2.rectangle(im,tuple(right[0]),tuple(right[1]),rtRectColor,2)
        cv2.putText(im,str(lftTot),(lft[0][0],lft[0][1]-5),cv2.FONT_HERSHEY_SIMPLEX,1,lftRectColor,2,cv2.LINE_AA)
        cv2.putText(im,str(rtTot),(right[0][0],right[0][1]-5),cv2.FONT_HERSHEY_SIMPLEX,1,rtRectColor,2,cv2.LINE_AA)
        cv2.putText(im,str(angle),(lft[0][0]-20,right[0][1]-50),cv2.FONT_HERSHEY_SIMPLEX,1,lftRectColor,2,cv2.LINE_AA)
        cv2.putText(im,str(count),(lft[0][0]-100,right[0][1]-250),cv2.FONT_HERSHEY_SIMPLEX,1,lftRectColor,2,cv2.LINE_AA)
        #cv2.imshow("Thresh", imgThresh)

        cv2.imshow("Contours", im)

        if cv2.waitKey(1)==ord('q') or turns == 3: #wait until key ‘q’ pressed
            ser.flush()
            speed = 1500
            angle = 2090
            ser.write((str(speed) + "\n").encode('utf-8'))
            ser.write((str(angle) + "\n").encode('utf-8'))
            break
    cv2.destroyAllWindows()
