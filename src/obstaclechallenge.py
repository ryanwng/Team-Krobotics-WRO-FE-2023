'''
This code can successfully do the obstacle challenge.

'''

import cv2
import time
import numpy as np
from picamera2 import Picamera2
import serial
#import RPi.GPIO as GPIO

if __name__ == '__main__':

    
    This part will allow the code to run at the start of a button instead of running it manually.
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    
        while(True):
        if GPIO.input(5) == GPIO.LOW: #wait for button to be pressed
            break
    

    #Intializing the camera
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640,480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()

    #Initalizing variables
    width = 640
    height = 480
    kp = 0.006
    angle = 2090
    turns = 0
    pid = True
    red = False
    green = False
    

    # Initialize last detection time
    last_detection_time = time.time()
    delay = time.time()

    #Region of interests
    lft = [[0,150],[150,480]]
    right = [[490,150],[640,480]]
    mid = [[150, 150], [490, 480]]
    lowmid = [[0,300], [640, 480]]
    points = [(0,0),(640,0),(640,480),(0,480)]
    
    steering = 0
    redarea = 0
    greenarea = 0
    change = False
    lap = False

    blueColor = (255,0,0) # Blue
    greenColor = (0,255,0) #green
    midRectColor = (0,0,255) #Red!
    lftRectColor = (255,0,0) # Blue
    rtRectColor = (255,0,0)
    midRectColor = (0,0,255) #Red!
    greenColor = (0,255,0) #hmm, I wonder what colour this is

    lower_red = np.array([120, 140, 90]) #([120, 30, 90])
    upper_red = np.array([180, 255, 255]) #([180, 255, 255]) 
    lower_green = np.array([30, 80, 30]) #([60, 100, 30])
    upper_green = np.array([100, 255, 255]) #([90, 255, 255])

    ser = serial.Serial('/dev/ttyACM0', 115200, timeout = 1) #approximately 115200 characters per second
    ser.flush()
    time.sleep(8)	    
    
    speed = 1680 #faster speed to accelerate
    ser.write((str(speed) + "\n").encode('utf-8'))

    while True:
        steering = 1
        im = picam2.capture_array()


        input = np.float32(points)
        output = np.float32([(0,0), (width-1, 0), (width-1,height-1), (0, height-1)])

        matrix = cv2.getPerspectiveTransform(input,output)
        imgPerspective = cv2.warpPerspective(im, matrix, (width, height),
                                            cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))
                                            
                                                          
	# Define range for HSV
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
            
        #Red Detection Begins ~
        max_area = 0
        max_cnt = None
        
        rmask = cv2.inRange(hsv, lower_red, upper_red) #test to see if it can be taken out of while loop

        contours_red, _ = cv2.findContours(rmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for i in range(len(contours_red)):
            cnt = contours_red[i]
            redarea = cv2.contourArea(cnt)
            if redarea > max_area:
                max_area = redarea
                max_cnt = cnt
            if(redarea > 1500 and max_cnt is not None):
                cv2.drawContours(im, contours_red, i, (0, 255, 0), 2)
                M = cv2.moments(max_cnt)

                # Calculate x,y coordinate of center
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                print("Red Centroid coordinates: ", cX,",",cY)

                # Draw contour and center of shape on image
                cv2.drawContours(im, [max_cnt], -1, (0, 255, 0), 3)
                cv2.circle(im,(cX,cY),5,(255,255,255),-1)
                
                if cX > 170:
                #constants may be changed
                    steering = 0.05*(cX-200) + 0.1*(cY)
                    if steering > 45:
                        steering = 45
                
                elif cX < 170 and cX > 50: #doesn't need to readjust 
                    steering = 0
               
                elif cX < 50: #Helps with edge cases
                    steering = -35
                
                
                angle = 2090 + steering
                red = True
                green = False
                pid = False
            else:
                pid = True

        
        #Green detection begins ~
        
        if red == False:
            max_area = 0
            max_cnt = None
       
            gmask = cv2.inRange(hsv, lower_green, upper_green)
            contours_green, _ = cv2.findContours(gmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			
            for i in range(len(contours_green)):
                cnt = contours_green[i]
                greenarea = cv2.contourArea(cnt)
                if greenarea > max_area:
	                max_area = greenarea
	                max_cnt = cnt
                if(greenarea > 1500 and max_cnt is not None):
                    cv2.drawContours(im, contours_green, i, (0, 255, 0), 2)
                    M = cv2.moments(cnt)

		    # Calculate x,y coordinate of center
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    print("Green Centroid coordinates: ", cX,",",cY)

		# Draw contour and center of shape on image
                    cv2.drawContours(im, [cnt], -1, (0, 255, 0), 3)
                    cv2.circle(im,(cX,cY),5,(255,255,255),-1)
					
                    if cX < 480:
			#constants may be changed
                        steering = -0.05*(cX-200) - 0.1*(cY)
                        if steering < -45:
                            steering = -45
					
                    elif cX > 480 and cX < 550: #doesn't need to readjust 
                        steering = 0
                    
                    elif cX > 550:
                        steering = 35
					
                    angle = 2090 + steering
                    green = True
                    red = False
                    pid = False
                else:
                    pid = True       
            

	
      

        # Find contours in the edge image
        imgGray = cv2.cvtColor(imgPerspective, cv2.COLOR_BGR2GRAY)
        ret, imgThresh = cv2.threshold(imgGray, 35, 255, cv2.THRESH_BINARY_INV) #This number may be subject to change, in order to detect only black
        # First coord is top left, second coord is bottom right.

        contoursLeft, _ = cv2.findContours(imgThresh[lft[0][1]:lft[1][1],lft[0][0]:lft[1][0]], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contoursRight, _ = cv2.findContours(imgThresh[right[0][1]:right[1][1],right[0][0]:right[1][0]], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contoursLowMid, _ = cv2.findContours(imgThresh[lowmid[0][1]:lowmid[1][1],lowmid[0][0]:lowmid[1][0]], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Offset based off topleft corner of each left/right rectrangle
        contoursLeft = [ c + lft[0] for c in contoursLeft]
        contoursRight = [ c + right[0] for c in contoursRight]
        contoursLowMid = [ c + lowmid[0] for c in contoursLowMid]

        # Draw contours on the original image
        cv2.drawContours(im, contoursLeft, -1, (0, 255, 0), 2)
        cv2.drawContours(im, contoursRight, -1, (0, 255, 0), 2)
        #cv2.drawContours(im, contoursLowMid, -1 (0, 255, 0), 2)

       
        lftTot = 0
        for c in contoursLeft:
            lftTot += cv2.contourArea(c)

        rtTot = 0
        for c in contoursRight:
            rtTot += cv2.contourArea(c)    

        lowMidTot = 0
        for c in contoursLowMid:
            lowMidTot += cv2.contourArea(c)   

        if angle > 2180 or angle < 2000:
                angle = 2090 

        if((lftTot < 2000 or rtTot < 2000) and time.time() - delay >= 8.5):
	        turns += 1
	        delay = time.time()
	        
        if pid == True:
            red = False
            green = False
            error = lftTot-rtTot
            if steering == 1:
                steering = kp * error
                #print("Wall PID")
                if error > 450 or error < -450:
                    angle = 2090 + steering
                if lowMidTot > 25000:
                    if lftTot < rtTot:
                        steering = 45
                    elif rtTot > lftTot:
                        steering = -45
            steering = int(steering)
            
        '''
        if turns == 8:
            if change == False:
                delayer = time.time()
                change = True
                check = False
            if (red == True and time.time() -  delayer >= 1) and check == False: #implies needs to finish last lap counterclockwise
                print("Start")
                check = True
                ser.flush()
                speed = 1675
                if lftTot > rtTot:
                    angle = 2120
                elif rtTot < lftTot:
                    angle = 2060
                ser.write((str(speed) + "\n").encode('utf-8'))
                ser.write((str(angle) + "\n").encode('utf-8'))
                time.sleep(3)
                ser.flush()
                
                
                if lftTot > rtTot:
                    angle = 2030
                    ser.write((str(angle) + "\n").encode('utf-8'))
                    time.sleep(5)
                    print("End")
                elif rtTot > lftTot:
                    angle = 2150
                    ser.write((str(angle) + "\n").encode('utf-8'))
                    time.sleep(5)
                    print("End")
                    '''
                
        if turns >=12: 
            if lap == False:
                last_detection_time = time.time() 
                lap = True
            if (time.time() - last_detection_time >= 7.5):
                ser.flush()
                speed = 1500
                angle = 2090
                ser.write((str(speed) + "\n").encode('utf-8'))
                ser.write((str(angle) + "\n").encode('utf-8'))
                break
                cv2.destroyAllWindows()
        
        angle = int(angle)    
        if angle > 2140:
            angle = 2140
        elif angle < 2040:
            angle = 2040
       
        #print("Angle:",angle)
        #print("Steering:",steering)
        #print("Red Area:", redarea)
        #print("Green Area:", greenarea)
        #print("Error:",error)
        #print("Turns:", turns)
        #print("Orange turns:",orangeturns)
        #print("Lowmid total:", lowMidTot)
        #print(red)
    
        ser.write((str(angle) + "\n").encode('utf-8'))
        
	'''
        cv2.rectangle(im,tuple(lft[0]),tuple(lft[1]),lftRectColor,2)
        cv2.rectangle(im,tuple(right[0]),tuple(right[1]),rtRectColor,2)
        cv2.rectangle(im,tuple(mid[0]),tuple(mid[1]), midRectColor,2)
        cv2.rectangle(im,tuple(lowmid[0]),tuple(lowmid[1]), greenColor,2)
        cv2.putText(im,str(lftTot),(lft[0][0],lft[0][1]-5),cv2.FONT_HERSHEY_SIMPLEX,1,lftRectColor,2,cv2.LINE_AA)
        cv2.putText(im,str(rtTot),(right[0][0],right[0][1]-5),cv2.FONT_HERSHEY_SIMPLEX,1,rtRectColor,2,cv2.LINE_AA)
        cv2.putText(im,str(angle),(lft[0][0]-20,right[0][1]-50),cv2.FONT_HERSHEY_SIMPLEX,1,lftRectColor,2,cv2.LINE_AA)
        cv2.putText(im,str(turns),(lft[0][0]-50,right[0][1]-50),cv2.FONT_HERSHEY_SIMPLEX,1,lftRectColor,2,cv2.LINE_AA)
        cv2.putText(im,str(greenarea),(mid[0][0]-20,mid[0][1]-50),cv2.FONT_HERSHEY_SIMPLEX,1,greenColor,2,cv2.LINE_AA) #Area of green contours
        cv2.putText(im,str(redarea),(mid[0][0]-20,mid[0][1]-100),cv2.FONT_HERSHEY_SIMPLEX,1,midRectColor,2,cv2.LINE_AA) #Area of red contours
        cv2.imshow("Final", im)

        if cv2.waitKey(1)==ord('q'): #End program, press 'q'
            ser.flush()
            speed = 1500
            angle = 2090
            ser.write((str(speed) + "\n").encode('utf-8'))
            ser.write((str(angle) + "\n").encode('utf-8'))
            break
    cv2.destroyAllWindows()
    '''
