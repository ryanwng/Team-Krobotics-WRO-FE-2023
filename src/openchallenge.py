import cv2
import time
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
    kp = 0.008
    angle = 2090
    turns = 0
    pasterror = 0
    
    orangeturns = 0
    # Initialize the last detection time
    last_detection_time = time.time()
    blast_detection_time = time.time()

    lft = [[0,150],[150,480]]
    right = [[490,150],[640,480]]
    
    whole = [[0,0],[640,480]]

    lftRectColor = (255,0,0) # Blue
    rtRectColor = (255,0,0) #Still blue

    # Define range for dark blue color in HSV
    lower_blue = np.array([100,40,40])
    upper_blue = np.array([180,255,255])

    lower_orange = np.array([0,40,20])
    upper_orange = np.array([11,255,255])

    points = [(0,0),(640,0),(640,480),(0,480)]
    
    lap = False

    ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 1) #approximately 57600 characters per second
    ser.flush()
    time.sleep(8)	    
    
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
                                            
                                                          
		# Define range for orange color in HSV
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only blue and orange colors
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)

        # Bitwise-OR mask and original image
        res_blue = cv2.bitwise_and(im,im, mask= mask_blue)
        res_orange = cv2.bitwise_and(im,im, mask= mask_orange)

        # Find contours in the result images
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_orange, _ = cv2.findContours(mask_orange, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # If contours are found and 5 seconds have passed since the last detection,
        # increment the counter and update the last detection 
        for i in range(len(contours_blue)):
            cnt = contours_blue[i]
            areablue = cv2.contourArea(cnt)
            
        for i in range(len(contours_orange)):
            cnt = contours_orange[i]
            areaorang = cv2.contourArea(cnt)
    


        # Find contours in the edge image
        imgGray = cv2.cvtColor(imgPerspective, cv2.COLOR_BGR2GRAY)
        ret, imgThresh = cv2.threshold(imgGray, 40, 255, cv2.THRESH_BINARY_INV) #This number may be subject to change, in order to detect only black
        # First coord is top left, second coord is bottom right.

        contoursLeft, _ = cv2.findContours(imgThresh[lft[0][1]:lft[1][1],lft[0][0]:lft[1][0]], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contoursRight, _ = cv2.findContours(imgThresh[right[0][1]:right[1][1],right[0][0]:right[1][0]], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        
        # Offset based off topleft corner of each left/right rectrangle
        contoursLeft = [ c + lft[0] for c in contoursLeft]
        contoursRight = [ c + right[0] for c in contoursRight]
        
       

        # Draw contours on the original image
        cv2.drawContours(im, contoursLeft, -1, (0, 255, 0), 2)
        cv2.drawContours(im, contoursRight, -1, (0, 255, 0), 2)

       
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
        if angle > 2180 or angle < 2000:
            angle = 2090
    
        if ((lftTot < 2000 or rtTot < 2000) and time.time() - blast_detection_time >= 8.5):
            turns += 1
            blast_detection_time = time.time()     
        '''
        if (lftTot <= 2000 and rtTot >= 2000 and error < -1900) or (lftTot <= 3000 and rtTot <= 9000 and error < -5500) or (lftTot <= 7000 and rtTot >= 13000 and error < -6000):
            angle = 2055 + steering
            print(lftTot <= 2000 and rtTot >= 2000 and error < -1900)
            print(lftTot <= 3000 and rtTot <= 9000 and error < -4500)
            print(lftTot <= 7000 and rtTot >= 13000 and error < -6000)

        elif (rtTot <= 2000 and lftTot >= 2000 and error > 1900) or (rtTot <= 3000 and lftTot <= 9000 and error > 5500) or (rtTot <= 7000 and lftTot >= 13000 and error > 6000):
            angle = 2125 + steering
            print(rtTot <= 2000 and lftTot >= 2000 and error > 1900)
            print(rtTot <= 3000 and lftTot <= 9000 and error > 4500)
            print(rtTot <= 7000 and lftTot >= 13000 and error > 6000)
        '''

        if error > 450 or error < -450:
            angle = 2090 + steering
            
        if turns >=12: 
            print("3 LAPS BABY YAY")
            if lap == False:
                last_detection_time = time.time()
            lap = True
            if (time.time() - last_detection_time >= 6):
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
       
        #print("Angle:",angle)
        #print("Steering:",steering)
        #print("Error:",error)
        print("Turns:", turns)
        #print("Blue Area:", areablue)
        #print("Orange Area:", areaorang)
    
        ser.write((str(angle) + "\n").encode('utf-8'))
        

        cv2.rectangle(im,tuple(lft[0]),tuple(lft[1]),lftRectColor,2)
        cv2.rectangle(im,tuple(right[0]),tuple(right[1]),rtRectColor,2)
        cv2.putText(im,str(lftTot),(lft[0][0],lft[0][1]-5),cv2.FONT_HERSHEY_SIMPLEX,1,lftRectColor,2,cv2.LINE_AA)
        cv2.putText(im,str(rtTot),(right[0][0],right[0][1]-5),cv2.FONT_HERSHEY_SIMPLEX,1,rtRectColor,2,cv2.LINE_AA)
        cv2.putText(im,str(angle),(lft[0][0]-20,right[0][1]-50),cv2.FONT_HERSHEY_SIMPLEX,1,lftRectColor,2,cv2.LINE_AA)
        cv2.putText(im,str(turns),(lft[0][0]-50,right[0][1]-50),cv2.FONT_HERSHEY_SIMPLEX,1,lftRectColor,2,cv2.LINE_AA)
    
        #cv2.imshow("Thresh", imgThresh)
        cv2.imshow("Final", im)

        if cv2.waitKey(1)==ord('q'): #End program, press 'q'
            ser.flush()
            speed = 1500
            angle = 2090
            ser.write((str(speed) + "\n").encode('utf-8'))
            ser.write((str(angle) + "\n").encode('utf-8'))
            break
    cv2.destroyAllWindows()

#Improved Open (More consistent)
