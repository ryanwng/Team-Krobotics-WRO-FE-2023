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
    red = False
    green = False
    flip = False
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

        mid = [[210, 305], [440, 480]]

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

        if count == 8 and flip == False:
            sleep(0.05)
            if red == True and turns <= 5:
                speed = 1630
                angle = 2020
                turns+=1
            


    #Red Detection Begins ~
        contoursMid, _ = cv2.findContours(imgThresh[mid[0][1]:mid[1][1],mid[0][0]:mid[1][0]], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contoursMid = [ c + mid[0] for c in contoursMid]
        cv2.drawContours(im, contoursMid, -1, (0, 255, 0), 2)

        img_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    # Lower mask (0-10)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask0 = cv2.inRange(img_hsv, lower_red, upper_red)
    # Upper mask (170-180)
        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
    # Join the masks
        raw_mask = mask0 | mask1
        contours = cv2.findContours(raw_mask, cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]

        for i in range(len(contoursMid)):
            cnt = contoursMid[i]
            area = cv2.contourArea(cnt)
            print("Area of red", area)
            if(area > 2000):
                cv2.drawContours(im, contours, i, (0, 255, 0), 2)
                M = cv2.moments(cnt)

                # Calculate x,y coordinate of center
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                print("Centroid coordinates: ", cX,",",cY)

                # Draw contour and center of shape on image
                cv2.drawContours(im, [cnt], -1, (0, 255, 0), 3)
                cv2.circle(im,(cX,cY),5,(255,255,255),-1)
                
                if cX > 320:
                    steering = 0.01*(cX-160)* 0.05*(cY)
                
                elif cX < 320:
                    steering = 0.01*(cX-480) * 0.05*(cY)
                    
                
                angle = 2090 + steering
                red = True
                green = False


        #Green detection begins ~
        
            
        lower_green = np.array([50,120,50])
        upper_green = np.array([255, 255, 15])
        gmask = cv2.inrange(img_hsv, lower_green, upper_green)
        contours = cv2.findContours(gmask, cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
        
        for i in range(len(contoursMid)):
            cnt = contoursMid[i]
            area = cv2.contourArea(cnt)
            print("Area of green", area)
            if(area > 2000):
                cv2.drawContours(im, contours, i, (0, 255, 0), 2)
                M = cv2.moments(cnt)

                # Calculate x,y coordinate of center
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                print("Centroid coordinates: ", cX,",",cY)

                # Draw contour and center of shape on image
                cv2.drawContours(im, [cnt], -1, (0, 255, 0), 3)
                cv2.circle(im,(cX,cY),5,(255,255,255),-1)
                
                if cX > 320:
                    steering = 0.01*(cX-480) * 0.05*(cY)
                
                elif cX < 320:
                    steering = 0.01*(cX-160)* 0.05*(cY)
                
                angle = 2090 + steering
                green = True
                red = False

        

        else:
            error = lftTot-rtTot
            d = error - pasterror 
            steering = kp * error + kd * d 
            steering = int(steering)
            pasterror = error
            red = False 
            Green = False
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
        print("Steering:",steering)
        print("Error:",error)
        print("Turns:", count)
        print("# of turns for cancer", turns)
    
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

#Major Procrastination 1