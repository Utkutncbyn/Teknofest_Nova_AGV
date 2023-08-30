from asyncore import read
import serial
import cv2
import numpy as np
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
import time
import pymssql


ser  = serial.Serial("/dev/ttyUSB0",9600)
ser.baudrate = 9600

counter_5 = 0
counter_7 = 0
counter_9 = 0
counter_18 = 0
counter_22 = 0
counter_26 = 0
counter_30 = 0
counter_31 = 0
counter_33 = 0
counter_38 = 0
counter_43 = 0
counter_48 = 0

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
in5 = 23
in6 = 26
in7 = 25
in8 = 24
buzzer = 16
GPIO.setmode(GPIO.BCM)
GPIO.setup(en1, GPIO.OUT)
GPIO.setup(en2, GPIO.OUT)
GPIO.setup(en3, GPIO.OUT)
GPIO.setup(en4, GPIO.OUT)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(in5, GPIO.OUT)
GPIO.setup(in6, GPIO.OUT)
GPIO.setup(in7, GPIO.OUT)
GPIO.setup(in8, GPIO.OUT)
GPIO.setup(buzzer,GPIO.OUT)
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
GPIO.output(in5, GPIO.LOW)
GPIO.output(in6, GPIO.LOW)
GPIO.output(in7, GPIO.LOW)
GPIO.output(in8, GPIO.LOW)




#  Functions

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

def YukAl():
    global is_loaded
    global akım
    GPIO.output(buzzer,GPIO.HIGH)
    GPIO.output(in5, GPIO.HIGH)
    GPIO.output(in6, GPIO.LOW)
    GPIO.output(in7, GPIO.HIGH)
    GPIO.output(in8, GPIO.LOW)
    is_loaded = True
    akım = 2

def YukIndir():
    global is_loaded
    global akım
    GPIO.output(buzzer,GPIO.HIGH)
    GPIO.output(in5, GPIO.LOW)
    GPIO.output(in6, GPIO.HIGH)
    GPIO.output(in7, GPIO.LOW)
    GPIO.output(in8, GPIO.HIGH)
    is_loaded = False
    akım = 0.40
 
counter = 1
while True:
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
            
            read_ser = str(ser.readline())
            read_ser = read_ser.strip("  x   f c a . e f b ' \ n ")
            read_ser = read_ser.split(";")
            qr = read_ser[1]
            mesafe = read_ser[0]
            mesafe = float(mesafe)
            #x_coor = read_ser[2]
            #y_coor = read_ser[3]

            # Bu kısım _main_'de de olabilir.
            conn = pymssql.connect(server="192.168.208.31", port="1433", user="sa", password="Faruk6161.", database="Nova")
            cursor = conn.cursor()

            cursor.execute("UPDATE [Diagnostics] set [IsLoaded] = %s WHERE [Id] = %s ",(is_loaded,1))
            cursor.execute("UPDATE [Diagnostics] set [Velocity] = %s WHERE [Id] = %s ",(45,1))
            cursor.execute("UPDATE [Diagnostics] set [BatteryLevel] = %s WHERE [Id] = %s ",(92,1))
            cursor.execute("UPDATE [Diagnostics] set [BatteryHeat] = %s WHERE [Id] = %s ",(23,1))
            cursor.execute("UPDATE [Diagnostics] set [CurrentThroughput] = %s WHERE [Id] = %s ",(akım,1))
            cursor.execute("UPDATE [Diagnostics] set [LastPosition] = %s WHERE [Id] = %s ",(qr,1))
            conn.commit()

            cursor.close()  # Duruma göre cursor kapatılmayabilir.

            cv2.drawContours(frame, c, -1, (0,0,255), 1)
            cv2.imshow("Mask",mask_filter)
            cv2.imshow("Frame",frame)
            
        ## Dönüş Case ve Komutları ##

            if curve_x >= 120:
                print("Saga Dönülüyor - Şerit!")
                sag()
        
            elif curve_x < 120 and curve_x > 40:
                print("Şerit Takp Ediliyor")
                duz()
        
            elif curve_x <=40:
                print("Sola Dönülüyor - Şerit!")
                sol()
            
            if mesafe  <= 60 and counter ==1:
                print("duruyorum")
                time.sleep(5)
                counter +=1
            elif mesafe <=50 and counter ==2 :
                engel_kacis()
                counter = counter +1
                print(mesafe)

            elif mesafe  <= 40 and counter ==3:
                print("duruyorum")
                time.sleep(5)
                counter +=1
            elif mesafe <=30 and counter ==4 :
                engel_kacis()
                counter = counter +1
                print(mesafe)
            
            elif mesafe  <= 20 and counter ==5:
                print("duruyorum")
                time.sleep(5)
                counter +=1
            elif mesafe <=12 and counter ==6 :
                engel_kacis()
                counter = counter +1
                print(mesafe)

            cv2.circle(frame, (curve_x,curve_y), 5, (255,255,255), -1)
            

            if qr == "Q22" and counter_22 == 0:
                counter_22 += 1
                sag()
                time.sleep(1) 
            elif qr == "Q26" and counter_26 == 0:
                counter_26 += 1
                time.sleep(0.5)
                sag()
                time.sleep(1)
            elif qr == "Q33" and counter_33 == 0:
                counter_33 += 1
                dur()
                time.sleep(2)
                YukAl()
                time.sleep(4)
                GPIO.output(buzzer,GPIO.LOW)
            elif qr == "Q31" and counter_31 == 0:
                counter_31 += 1
                time.sleep(0.5)
                sag()
                time.sleep(1)
            elif qr == "Q9" and counter_9 == 0:
                counter_9 += 1
                time.sleep(0.5)
                sag()
                time.sleep(1)
            elif qr == "Q43" and counter_43 == 0:
                counter_43 += 1
                time.sleep(0.5)
                sol()
                time.sleep(1)
            elif qr == "Q18" and counter_18 == 0:
                counter_18 += 1
                time.sleep(0.5)
                sol()
                time.sleep(1)
            elif qr == "Q48" and counter_48 == 0:
                counter_48 += 1
                time.sleep(0.5)
                sol()
                time.sleep(1)
            elif qr == "Q5" and counter_5 == 0:
                counter_5 += 1
                time.sleep(0.5)
                sol()
                time.sleep(1)
            elif qr == "Q38" and counter_38 == 0:
                counter_38 += 1
                dur()
                time.sleep(2)
                YukIndir()
                time.sleep(4)
                GPIO.output(buzzer,GPIO.LOW)
            elif qr == "Q30" and counter_30 == 0:
                counter_30 += 1
                time.sleep(0.5)
                sol()
                time.sleep(1)
            elif qr == "Q22" and counter_22== 1:
                counter_22 += 1
                sol()
                time.sleep(1)
            elif qr == "Q7" and counter_7 == 0:
                counter_7 += 1
            elif qr == "Q7" and counter_7 == 1:
                counter_7 += 1
                dur()
                break
    else:
        print("Şerit Algılanmadı !!!")
        sol()
            
cap.release()
cv2.destroyAllWindows()
