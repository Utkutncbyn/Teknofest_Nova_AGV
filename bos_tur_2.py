import serial
import cv2
import numpy as np
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
import time
import multiprocessing



cap = cv2.VideoCapture(0)
cap.set(3, 160)
cap.set(4, 120)



#### Pin Connections  ####
in1 = 22
in2 = 27
in3 = 12
in4 = 21
en1 = 5
en2 = 19
en3 = 17
en4 = 6
GPIO.setmode(GPIO.BCM)
GPIO.setup(en1, GPIO.OUT)
GPIO.setup(en2, GPIO.OUT)
GPIO.setup(en3, GPIO.OUT)
GPIO.setup(en4, GPIO.OUT)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
p1 = GPIO.PWM(en1, 100)
p2 = GPIO.PWM(en2, 100)
p3 = GPIO.PWM(en3, 100)
p4 = GPIO.PWM(en4, 100)
p1.start(80)
p2.start(80)
p3.start(80)
p4.start(80)
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
GPIO.output(in3, GPIO.LOW)
GPIO.output(in4, GPIO.LOW)
ser  = serial.Serial("/dev/ttyUSB0",9600)
ser.baudrate = 9600
global mesafe

def engel_kacis():
    sag()
    time.sleep(1)
    duz()
    time.sleep(2)
    sol()
    time.sleep(1)
    duz()
    time.sleep(2)
    sol()
    time.sleep(1)
    duz()
    time.sleep(2)
    sag()
    time.sleep(1)

def sol():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def sag():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)

def duz():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)

def dur():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)
#  Functions
def mesafe_engel ():
    
        read_ser = str(ser.readline())
        read_ser = read_ser.strip("  x  f c a . e f b ' \ n ")
        read_ser = read_ser.split(";")
        mesafe = read_ser[0]
        mesafe = float(mesafe)
        print("mesafe :",mesafe)
        
    




p1 = multiprocessing.Process(target =mesafe_engel)



while True:
    p1.run()
    ret, frame = cap.read()
    low_boundry = np.uint8([50,50,50])
    high_boundry = np.uint8([0,0,0])
    mask_filter = cv2.inRange(frame, high_boundry, low_boundry)
    contours, hierarchy = cv2.findContours(mask_filter, 1, cv2.CHAIN_APPROX_NONE)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        
    
        if M["m00"] !=0 :    
            curve_x = int(M['m10']/M['m00'])
            curve_y = int(M['m01']/M['m00'])
            print("Curve X : "+str(curve_x)+"  Curve Y : "+str(curve_y))
            

            #x_coor = read_ser[2]
            #y_coor = read_ser[3]

            # Bu kısım _main_'de de olabilir.
        

#             cv2.drawContours(frame, c, -1, (0,0,255), 1)
#             cv2.imshow("Mask",mask_filter)
#             cv2.imshow("Frame",frame)
            
        ## Dönüş Case ve Komutları ##

            
            if curve_x >= 120:
                sag()
                print("sag")
                
        
            elif curve_x < 120 and curve_x > 40:
                duz()
                print("duz")
        
            elif curve_x <=40:
                sol()
                print("sol")
                
            if mesafe <=3 :
                print("engelden kacılıyor")
                engel_kacis()
                counter +=1
            
            
                
    else:
        print("sola donuluyor")
        sol()

#             cv2.circle(frame, (curve_x,curve_y), 5, (255,255,255), -
        







