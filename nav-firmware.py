#from nav_pub import * 
from gps_pub import * # publish GPS status
from compass_pub import * # publish compass status
from vector_pub import * # publish vectors
import time
import RPi.GPIO as GPIO
from threading import Thread

vector_thread = Thread(target=publish_vector)
vector_thread.start()

try:
        while(True):
                publish_compas_status()
                publish_gps_status()
                calibrate_external_compass()
                time.sleep(.1)
except KeyboardInterrupt:
        GPIO.setup(13,GPIO.OUT)
        GPIO.setup(19,GPIO.OUT)
        GPIO.setup(26,GPIO.OUT)

        GPIO.output(13,GPIO.LOW)
        GPIO.output(19,GPIO.LOW)
        GPIO.output(26,GPIO.LOW)
        print("exit")
