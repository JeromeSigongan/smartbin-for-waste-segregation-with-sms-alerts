# ------------------------------------------------------------------------
# Smart Bin ML Project
# Bachelor of Science in Information Technology batch 2022-2023 Capstone Project
# 
# (c) 2022 by Jerome Sigongan, Shahida Xerxy Cuizon, Hamer Sinodlay, Joanna Redondo, Maricel Macapulay
# Northern Bukidnon State College
# --------------------------------------------------------------------------
from __future__ import division
import Adafruit_PCA9685
import RPi.GPIO as GPIO
import time
import serial

from picamera import PiCamera
from time import sleep
from lobe import ImageModel
from GSMmodule import SMS

TRIG=int(21)
ECHO=int(20)
TRIG1=int(8)
ECHO1=int(25)
TRIG2=int(24)
ECHO2=int(23)
TRIG3=int(26)
ECHO3=int(19)

Relay2=4
Relay3=17
Relay5=27
Relay6=5
Relay8=6
Relay9=13

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(TRIG1, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)
GPIO.setup(TRIG2, GPIO.OUT)
GPIO.setup(ECHO2, GPIO.IN)
GPIO.setup(TRIG3, GPIO.OUT)
GPIO.setup(ECHO3, GPIO.IN)

GPIO.setup(Relay2, GPIO.OUT)
GPIO.setup(Relay3, GPIO.OUT)
GPIO.setup(Relay5, GPIO.OUT)
GPIO.setup(Relay6, GPIO.OUT)
GPIO.setup(Relay8, GPIO.OUT)
GPIO.setup(Relay9, GPIO.OUT)


#Arduino - Raspberry Pi Serial Communication
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0',9600, timeout=1)
    ser.flush()
    
# Initialise the PCA9685 servo motor driver using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Configure min and max servo pulse lengths
servo_min = 200  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096

camera = PiCamera()

# Load Lobe TF model
# --> Change model file path as needed
model = ImageModel.load('/home/pi/Lobe/model')

# Take Photo
def take_photo():
    print("Detected")
    ser.write(b"normal\n")
    # Start the camera preview
    camera.start_preview(alpha=200)
    # wait 2s or more for light adjustment
    sleep(3) 
    # Optional image rotation for camera
    # --> Change or comment out as needed
    camera.rotation = 270
    #Input image file path here
    # --> Change image path as needed
    camera.capture('/home/pi/Pictures/image.jpg')
    #Stop camera
    camera.stop_preview()

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

# Identify prediction and turn on appropriate stepper rotation
def bin_select(label):
    print(label)
    if label == "Biodegradable":
        print ("Bin-1 Level Measurement")
        time.sleep(0.000002)
        GPIO.output(TRIG,True)
        time.sleep(0.000010)
        GPIO.output(TRIG,False)
            
        StartTime = time.time()
        StopTime = time.time()
            
        while GPIO.input(ECHO)==0:
            StartTime=time.time()
        while GPIO.input(ECHO)==1:
            StopTime=time.time()
                
        TimeElapsed = StopTime - StartTime
        distance = (TimeElapsed * 34300)/2
        distance = int(distance)
        print("Distance: %.2f cm" % (distance))

        if(distance>33):
            print('Moving stepper')
            ser.write(b"CCW\n")
            time.sleep(10)
            print('Moving servo on channel 0')
            # Move servo on channel O between extremes.
            pwm.set_pwm(0, 0, servo_min)
            time.sleep(1)
            pwm.set_pwm(0, 0, servo_max)
            time.sleep(1)
            #Stepper motor back on default position
            ser.write(b"default\n")
                
            GPIO.output(Relay2,True)
            GPIO.output(Relay3,True)

        elif(distance>=7 and distance<=33):
            print('Moving stepper')
            ser.write(b"CCW\n")
            time.sleep(10)
            print('Moving servo on channel 0')
            # Move servo on channel O between extremes.
            pwm.set_pwm(0, 0, servo_min)
            time.sleep(1)
            pwm.set_pwm(0, 0, servo_max)
            time.sleep(1)
            #Stepper motor back on default position
            ser.write(b"default\n")
            
            GPIO.output(Relay2,True)
            GPIO.output(Relay3,False)
                
        elif(distance>=0 and distance<=3):        
            GPIO.output(Relay2,False)
            GPIO.output(Relay3,False)
            ser.write(b"frawn\n")
            print("Bin 1 is already full send SMS to utility")
            sendSMS1()        
                    
    if label == "Recyclable":
        print ("Bin-2 Level Measurement")
        time.sleep(0.000002)
        GPIO.output(TRIG1,True)
        time.sleep(0.000010)
        GPIO.output(TRIG1,False)
            
        StartTime = time.time()
        StopTime = time.time()
            
        while GPIO.input(ECHO1)==0:
            StartTime=time.time()
        while GPIO.input(ECHO1)==1:
            StopTime=time.time()
                
        TimeElapsed = StopTime - StartTime
        distance = (TimeElapsed * 34300)/2
        distance = int(distance)
        print("Distance: %.2f cm" % (distance))

        if(distance>33):    
            print('Moving servo on channel 0')
            # Move servo on channel O between extremes.
            pwm.set_pwm(0, 0, servo_min)
            time.sleep(1)
            pwm.set_pwm(0, 0, servo_max)
            time.sleep(1)
            
            GPIO.output(Relay5,True)
            GPIO.output(Relay6,True)

        elif(distance>=7 and distance<=33):
            print('Moving servo on channel 0')
            # Move servo on channel O between extremes.
            pwm.set_pwm(0, 0, servo_min)
            time.sleep(1)
            pwm.set_pwm(0, 0, servo_max)
            time.sleep(1)
            
            GPIO.output(Relay5,True)
            GPIO.output(Relay6,False)
            
        elif(distance>=0 and distance<=6):        
            GPIO.output(Relay5,False)
            GPIO.output(Relay6,False)
            ser.write(b"frawn\n")
            print("Bin 2 is already full send SMS to utility")
            sendSMS2()

                    
    if label == "Non-Biodegradable":
        print ("Bin-3 Level Measurement")
        time.sleep(0.000002)
        GPIO.output(TRIG2,True)
        time.sleep(0.000010)
        GPIO.output(TRIG2,False)
            
        StartTime = time.time()
        StopTime = time.time()
            
        while GPIO.input(ECHO2)==0:
            StartTime=time.time()
        while GPIO.input(ECHO2)==1:
            StopTime=time.time()
                
        TimeElapsed = StopTime - StartTime
        distance = (TimeElapsed * 34300)/2
        distance = int(distance)
        print("Distance: %.2f cm" % (distance))

        if(distance>33):
            print('Moving stepper')
            ser.write(b"CW\n")
            time.sleep(10)
            print('Moving servo on channel 0')
            # Move servo on channel O between extremes.
            pwm.set_pwm(0, 0, servo_min)
            time.sleep(1)
            pwm.set_pwm(0, 0, servo_max)
            time.sleep(1)
            #Stepper motor back on default position
            ser.write(b"default\n")    
            
            GPIO.output(Relay8,True)
            GPIO.output(Relay9,True)  

        elif(distance>=7 and distance<=33):
            print('Moving stepper')
            ser.write(b"CW\n")
            time.sleep(10)
            print('Moving servo on channel 0')
            # Move servo on channel O between extremes.
            pwm.set_pwm(0, 0, servo_min)
            time.sleep(1)
            pwm.set_pwm(0, 0, servo_max)
            time.sleep(1)
            #Stepper motor back on default position
            ser.write(b"default\n")
            
            GPIO.output(Relay8,True)
            GPIO.output(Relay9,False)
            
        if(distance>=0 and distance<=6):        
            GPIO.output(Relay8,False)
            GPIO.output(Relay9,False)
            ser.write(b"frawn\n")
            print("Bin 3 is already full send SMS to utility")
            sendSMS3()
                    
    if label == "No-Garbage":
        ser.write(b"arrow\n")

# Main Function
while True:
    GPIO.output(TRIG3,True)
    time.sleep(0.000010)
    GPIO.output(TRIG3,False)
    
    StartTime = time.time()
    StopTime = time.time()
    
    while GPIO.input(ECHO3)==0:
        StartTime=time.time()
    while GPIO.input(ECHO3)==1:
        StopTime=time.time()
        
    TimeElapsed = StopTime - StartTime
    distance = (TimeElapsed * 34300)/2
    distance = int(distance)
    if(distance>=0 and distance<=5):
        # Move servo on channel O between extremes.
        pwm.set_pwm(1,2, 0, servo_min)
        time.sleep(5)
        pwm.set_pwm(1,2, 0, servo_max)
        time.sleep(1)
        take_photo()
        # Run photo through Lobe TF model
        result = model.predict_from_file('/home/pi/Pictures/image.jpg')
        # --> Change image path
        bin_select(result.prediction)
        time.sleep(0.1)
        ser.write(b"smile\n")                 
    else:
        ser.write(b"arrow\n")
        print("No garbage detected")
        
GPIO.cleanup()