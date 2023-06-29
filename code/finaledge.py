import cv2
from time import sleep
import numpy as np
from picamera2 import Picamera2
import serial
import RPi.GPIO as GPIO

if __name__ == '__main__':

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)

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
    pg = 0.3 #proportional gain
    angle = 2090
    #speed = 1380
    points = [(0,200), (640,200), (640,370), (0,370)] #ROI points
    color = (0, 255, 255) #yellow
    thickness = 4 #line thickness
    ser = serial.Serial('/dev/ttyACM0', 57600, timeout = 1) #approximately 57600 characters per second
    ser.flush()
    sleep(8) 

    speed = 1330 #faster speed to accelerate
    ser.write((str(speed) + "\n").encode('utf-8'))

    while True:
        im= picam2.capture_array()
        #cv2.imshow("Camera", im)

        '''
        line1 = cv2.line(im, points[0], points[1], color, thickness)
        line2 = cv2.line(im, points[1], points[2], color, thickness)
        line3 = cv2.line(im, points[2], points[3], color, thickness)
        line4 = cv2.line(im, points[3], points[0], color, thickness)
        #cv2.imshow("ROI", im)
        
        #for showing ROI on screen

        '''

        input = np.float32(points)
        output = np.float32([(0,0), (width-1, 0), (width-1,height-1), (0, height-1)])

        matrix = cv2.getPerspectiveTransform(input,output)
        imgPerspective = cv2.warpPerspective(im, matrix, (width, height),
                                            cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0)) #sets up perspective
    #cv2.imshow("Perspective", imgPerspective)

        imgGray = cv2.cvtColor(imgPerspective, cv2.COLOR_BGR2GRAY) #grayscale
        #cv2.imshow("gray", imgGray)

        ret, imgThresh = cv2.threshold(imgGray, 110, 255, cv2.THRESH_BINARY_INV) #change this to like 50, or maybe even 10 as it is detecting other colours
        #cv2.imshow("threshold", imgThresh)

        imgBlur = cv2.GaussianBlur(imgGray, (3,3), 0)
        v = np.median(imgGray)
        lowThresh = int(max(0, (1.0 - 0.33) * v))
        highThresh = int(min(255, (1.0 + 0.33) * v))
        imgCanny = cv2.Canny(imgBlur, lowThresh, highThresh)
        imgFinal = cv2.add(imgThresh, imgCanny)
        #cv2.imshow("final", imgFinal)

        column = []
        for i in range(640):
            column.append(sum(imgFinal[:, i]))


        max1 = max(column[:319])
        max2 = max(column[320:])
        laneLeft = column.index(max1)
        laneRight = column.index(max2, 320)
        laneCenter = int((laneLeft + laneRight) / 2)
        print("laneCenter: ", laneCenter)

        imgFinalColor = cv2.cvtColor(imgFinal, cv2.COLOR_GRAY2BGR)
        cv2.line(imgFinalColor, (laneLeft, 0), (laneLeft, 479), (0, 255, 0), thickness=1, lineType=8) #left edge
        cv2.line(imgFinalColor, (laneRight, 0), (laneRight, 479), (0, 255, 0), thickness=1, lineType=8) #right edge
        target = cv2.line(imgFinalColor, (laneCenter, 0), (laneCenter, 479), (0, 255, 0), thickness=1, lineType=8) #lane center
        frame_center = cv2.line(imgFinalColor, (319, 0), (319, 479), (0, 0, 255), thickness=1, lineType=8) #red

        diff = (320 - laneCenter) * pg 

        ser.flush()


        angle -= diff
        print("Diff: ", diff)
        if angle <= 2000 or angle >= 2180 or laneCenter == 321:
            angle = 2090 #safety
        print("angle: ", angle)
        ser.write((str(angle) + "\n").encode('utf-8'))

        speed = 1380
        if speed >= 1600 or speed <= 1400:
            speed = 1380 #safety
        print("Speed: ", speed)
        ser.write((str(speed) + "\n").encode('utf-8'))


        cv2.imshow("finalColor", imgFinalColor)

        if cv2.waitKey(1)==ord('q'):#wait until key ‘q’ pressed
            ser.flush()
            speed = 0
            angle = 2090
            ser.write((str(speed) + "\n").encode('utf-8'))
            ser.write((str(angle) + "\n").encode('utf-8'))
            break
    cv2.destroyAllWindows()
