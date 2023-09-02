'''
TO DO:
Reduce ROI slightly (make it slightly lower)
Clean up code a little bit


'''
import cv2
import time
import numpy as np
from picamera2 import Picamera2
import serial
import RPi.GPIO as GPIO

if __name__ == '__main__':
    

    #Initializing the camera
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640,480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()

    #Initalizing variables
    width = 640
    height = 480
    kp = 0.008
    angle = 2090
    turns = 0

    # Initializing last detection time
    last_detection_time = time.time()
    delay = time.time()

    #Region of interests for walls 
    lft = [[0,150],[150,480]]
    right = [[490,150],[640,480]]

    blueColour = (255,0,0) # Blue

    # Define range for blue and orange in HSV
    lower_blue = np.array([100,40,40])
    upper_blue = np.array([180,255,255])
	
    lower_orange = np.array([0,40,20])
    upper_orange = np.array([11,255,255])

	
    # Whole screen
    points = [(0,0),(640,0),(640,480),(0,480)]
    
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 1) #approximately 115200 characters per second
    ser.flush()
    time.sleep(8)	    
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	
    while(True):
        if GPIO.input(5) == GPIO.LOW: #Waits for a button to be pressed before running the code
            break
            
    speed = 1660
    ser.write((str(speed) + "\n").encode('utf-8'))
    
    while True:
        im= picam2.capture_array()


        input = np.float32(points)
        output = np.float32([(0,0), (width-1, 0), (width-1,height-1), (0, height-1)])

        matrix = cv2.getPerspectiveTransform(input,output)
        imgPerspective = cv2.warpPerspective(im, matrix, (width, height),
                                            cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))
                                            
                                                          
	# Range for hsv
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only blue and orange colors
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)

        # Bitwise-OR mask and original image
        res_blue = cv2.bitwise_and(im,im, mask= mask_blue)
        res_orange = cv2.bitwise_and(im,im, mask= mask_orange)

        # Find contours in the images
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_orange, _ = cv2.findContours(mask_orange, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	# Find area of blue and orange lines respectively.
        for i in range(len(contours_blue)):
            cnt = contours_blue[i]
            areablue = cv2.contourArea(cnt)
            
        for i in range(len(contours_orange)):
            cnt = contours_orange[i]
            areaorang = cv2.contourArea(cnt)
    

        # Find contours in the edge image
        imgGray = cv2.cvtColor(imgPerspective, cv2.COLOR_BGR2GRAY)
        ret, imgThresh = cv2.threshold(imgGray, 40, 255, cv2.THRESH_BINARY_INV) #1st number (40) may be subject to change, in order to detect only black depending on lighting

	    
        # First coord is top left, second coord is bottom right.

        contoursLeft, _ = cv2.findContours(imgThresh[lft[0][1]:lft[1][1],lft[0][0]:lft[1][0]], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contoursRight, _ = cv2.findContours(imgThresh[right[0][1]:right[1][1],right[0][0]:right[1][0]], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        
        # Offset based off topleft corner of each left/right rectrangle
        contoursLeft = [ c + lft[0] for c in contoursLeft]
        contoursRight = [ c + right[0] for c in contoursRight]
        

	# Finding area of black in left and right Region of Interests (ROI)
        lftTot = 0
        for c in contoursLeft:
            lftTot += cv2.contourArea(c)

        rtTot = 0
        for c in contoursRight:
            rtTot += cv2.contourArea(c)
                   

	# Steering logic
        error = lftTot-rtTot
        steering = kp * error
        steering = int(steering)

	# If angle is too wide, car goes back to 2090    
        if angle > 2180 or angle < 2000:
            angle = 2090

	# If there's enough disparity between the areas of the two ROIS, enact steering logic
	if error > 450 or error < -450:
            angle = 2090 + steering
		
    	# Counts the number of turns done, with a delay on counting the number of laps
        if ((lftTot < 2000 or rtTot < 2000) and time.time() - delay >= 8.5):
            turns += 1
            delay = time.time()     

	# If 12 turns have elapsed (3 laps), then delay for a little bit, before stopping
        if turns >=12: 
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
				
        #Makes sure the angle is not too extreme
        angle = int(angle)    
        if angle > 2160:
            angle = 2160
        elif angle < 2020:
            angle = 2020
    
        ser.write((str(angle) + "\n").encode('utf-8'))
