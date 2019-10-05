from gps3.agps3threaded import AGPS3mechanism
import RPi.GPIO as GPIO
import numpy
import board
import busio
import adafruit_lsm9ds0
import time
import json
from mqtt_client.publisher import Publisher
from mqtt_client.subscriber import Subscriber
from threading import Thread
import IMU
import datetime
import math
from haversine import haversine,Unit
# GPS Setup
agps_thread = AGPS3mechanism()  # Instantiate AGPS3 Mechanisms
agps_thread.stream_data()  # From localhost (), or other hosts, by example, (host='gps.ddns.net')
agps_thread.run_thread()  # Throttle time to sleep after an empty lookup, default '()' 0.2 two tenths of a second
#initalizing previous GPS location for haversine
prev_pos = 0
current_pos = 0	
total_distance = 0
# I2C connection:
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds0.LSM9DS0_I2C(i2c)

#internal IMU setup

# If the IMU is upside down (Skull logo facing up), change this value to 1
IMU_UPSIDE_DOWN = 0

RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40      # Complementary filter constant


################# Compass Calibration values ############
# Use calibrateBerryIMU.py to get calibration values 
# Calibrating the compass isnt mandatory, however a calibrated 
# compass will result in a more accurate heading value.

magXmin =  0
magYmin =  0
magZmin =  0
magXmax =  0
magYmax =  0
magZmax =  0


IMU.detectIMU()     #Detect if BerryIMUv1 or BerryIMUv2 is connected.
IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass

gyroXangle = 0.0
gyroYangle = 0.0
gyroZangle = 0.0
CFangleX = 0.0
CFangleY = 0.0
kalmanX = 0.0
kalmanY = 0.0

a = datetime.datetime.now()

# Setup Pubber
pubber = Publisher(client_id="nav-pubber")

def publish_gps_status():
	global prev_pos
	global current_pos
	global total_distance
	
	current_pos = (agps_thread.data_stream.lat,agps_thread.data_stream.lon)
	distance_traveled = haversine(current_pos,prev_pos, unit=Unit.NAUTICAL_MILES)
	total_distance = total_distance + distance_traveled
	print(str(current_pos))
	print(str(prev_pos))
			
	if (agps_thread.data_stream.speed != 'n/a'):
		speed_kn = agps_thread.data_stream.speed * 1.94384449
	else:
		speed_kn = 0
		
	message = {
		'time' :  agps_thread.data_stream.time,
		'latitude' : agps_thread.data_stream.lat,
		'longitude' : agps_thread.data_stream.lon,
		'speed': speed_kn,
		'course': agps_thread.data_stream.track,
		'distance': total_distance

	}

	print(message)

	led_on_message = {
		'led_id' : 19,
		'command' : 1
	}

	led_off_message = {
		'led_id' : 19,
		'command' : 0
	}

	# check to see if the gps has a fix

	if(agps_thread.data_stream.time == 'n/a'):
		app_json = json.dumps(led_off_message)
		pubber.publish("/command/led",app_json)

	else:
		app_json = json.dumps(led_on_message)
		pubber.publish("/command/led",app_json)
  
	app_json = json.dumps(message)
	pubber.publish("/status/gps",app_json)
	prev_pos = current_pos

def publish_compas_status():
	mag_x, mag_y, mag_z = sensor.magnetic
	temp = sensor.temperature
	compass = round(-(24 + numpy.degrees(numpy.arctan2(mag_x, mag_y))))
	if compass < 0:
		compass = 360 + compass
	screencompass = compass
	message = {
		'temp' : temp,
		'compass': compass,
	}
	app_json = json.dumps(message)
	pubber.publish("/status/compass",app_json)

def publish_internal_compass_status():

	#Read the accelerometer,gyroscope and magnetometer values
	ACCx = IMU.readACCx()
	ACCy = IMU.readACCy()
	ACCz = IMU.readACCz()
	GYRx = IMU.readGYRx()
	GYRy = IMU.readGYRy()
	GYRz = IMU.readGYRz()
	MAGx = IMU.readMAGx()
	MAGy = IMU.readMAGy()
	MAGz = IMU.readMAGz()
	

	#Apply compass calibration    
	MAGx -= (magXmin + magXmax) /2 
	MAGy -= (magYmin + magYmax) /2 
	MAGz -= (magZmin + magZmax) /2 
 
	#Calculate heading
	heading = 180 * math.atan2(MAGy,MAGx)/M_PI

	#Only have our heading between 0 and 360
	if heading < 0:
		heading += 360

	# # #Calculate the new tilt compensated values
	#  magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)

	#  magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)   #LSM9DS0

	# #Calculate tilt compensated heading
	# tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI

	# if tiltCompensatedHeading < 0:
	# 			tiltCompensatedHeading += 360

	message = {
				'heading': heading
	}
	app_json = json.dumps(message)
	pubber.publish("/status/internal_compass",app_json)
	
def on_led_command(client, userdata, message):
	obj = json.loads(message.payload.decode('utf-8'))
	led_selector = obj['led_id']
	led_opp = obj['command']
	try:
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(19,GPIO.OUT) # BLUE
		GPIO.setup(26,GPIO.OUT) # WHITE
		GPIO.setup(13,GPIO.OUT) # GREEN
		
		if(led_opp == 1):
			GPIO.output(led_selector,GPIO.HIGH)
		else:
			GPIO.output(led_selector,GPIO.LOW)

	except Exception:
		 pass
		# print("LED FAILTURE")




# Setup Subscriber to change LED
default_subscriptions = {
	"/command/led": on_led_command
}
subber = Subscriber(client_id="led_actuator", broker_ip="192.168.1.170", default_subscriptions=default_subscriptions)

thread = Thread(target=subber.listen)
thread.start()

#subber.listen()

try: 
	while True:
		publish_gps_status()
		publish_compas_status()
		publish_internal_compass_status()
		time.sleep(.1)
except KeyboardInterrupt:
	# turn off all leds when program exits
	GPIO.output(13,GPIO.LOW)
	GPIO.output(19,GPIO.LOW)
	GPIO.output(26,GPIO.LOW)


