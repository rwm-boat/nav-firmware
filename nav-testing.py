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
prev_pos = (0,0)
current_pos = (0,0)
total_distance = 0
distance_traveled = 0
# I2C connection if possible
try:
	i2c = busio.I2C(board.SCL, board.SDA)
	sensor = adafruit_lsm9ds0.LSM9DS0_I2C(i2c)
except Exception:
	print("no 9dof imu")

#internal IMU setup
# If the IMU is upside down (Skull logo facing up), change this value to 1
IMU_UPSIDE_DOWN = 0

RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40      # Complementary filter constant

#internal compass calibration values

magXmin =  0
magYmin =  0
magZmin =  0
magXmax =  0
magYmax =  0
magZmax =  0

#external compass hard iron distortion calibration values

e_magXmin = 0
e_magYmin = 0
e_magZmin = 0
e_magXmax = 0
e_magYmax = 0
e_magZmax = 0

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

# Setup publisher
pubber = Publisher(client_id="nav-pubber")

def publish_gps_status():
	global prev_pos
	global current_pos
	global total_distance
	global distance_traveled

	# determine if real values are being produced, convert to knots, then calculate distance traveled
	if (agps_thread.data_stream.speed != 'n/a' and agps_thread.data_stream.speed != 0):
				speed_kn = agps_thread.data_stream.speed * 1.94384449
				current_pos = (agps_thread.data_stream.lat,agps_thread.data_stream.lon)
				if(prev_pos == (0,0)):
					prev_pos = current_pos
				distance_traveled = haversine(current_pos,prev_pos, unit=Unit.NAUTICAL_MILES)	
	else:
		speed_kn = 0

	#if gps is returning real values, publish values
	if(distance_traveled < 0.1):
		total_distance += distance_traveled
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

		# check to see if the gps has a fix, if it does, turn on LED's
		if(agps_thread.data_stream.time == 'n/a'):
			app_json = json.dumps(led_off_message)
			pubber.publish("/command/led",app_json)

		else:
			app_json = json.dumps(led_on_message)
			pubber.publish("/command/led",app_json)

		app_json = json.dumps(message)
		pubber.publish("/status/gps",app_json)
		prev_pos = current_pos

def calibrate_external_compass():
	mag_x, mag_y, mag_z = sensor.magnetic
	if(mag_x > e_magXmax): e_magXmax = mag_x
	if(mag_x < e_magXmin): e_magXmin = mag_x
	if(mag_y > e_magYmax): e_magYmax = mag_y
	if(mag_y < e_magYmin): e_magYmin = mag_y
	print("X Max: " + str(e_magXmax))
	print("Y Max: " + str(e_magYmax))
	print("X Min: " + str(e_magXmin))
	print("Y Min: " + str(e_magXmin))

def publish_compas_status():

	try:
		mag_x, mag_y, mag_z = sensor.magnetic
		temp = sensor.temperature

		e_MAGX = mag_x
		e_MAGY = mag_y
		# Apply compass calibration    
		e_MAGX -= (e_magXmin + e_magXmax) /2 
		e_MAGY -= (e_magYmin + e_magYmax) /2 
		#Calculate heading
		heading = 180 * math.atan2(e_MAGY,e_MAGX)/M_PI
		#Only have our heading between 0 and 360
		if heading < 0:
			heading += 360
		message = {
			'temp' : temp,
			'compass': heading,
		}
		print (message)
		app_json = json.dumps(message)
		pubber.publish("/status/compass",app_json)

	except Exception:
		print("no external imu")
def publish_internal_compass_status():

	#Read the accelerometer,gyroscope and magnetometer values

	MAGx = IMU.readMAGx()
	MAGy = IMU.readMAGy()
	
	#Apply compass calibration    
	MAGx -= (magXmin + magXmax) /2 
	MAGy -= (magYmin + magYmax) /2 
 
	#Calculate heading
	heading = 180 * math.atan2(MAGy,MAGx)/M_PI

	#Only have our heading between 0 and 360
	if heading < 0:
		heading += 360

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
			GPIO.output(26,GPIO.HIGH)
			GPIO.output(13,GPIO.HIGH)
		else:
			GPIO.output(led_selector,GPIO.LOW)

	except Exception:
		print("Status LED's not plugged in")

# Setup Subscriber to change LED
default_subscriptions = {
	"/command/led": on_led_command
}
subber = Subscriber(client_id="led_actuator", broker_ip="192.168.1.170", default_subscriptions=default_subscriptions)

# new thread for the subscriber
thread = Thread(target=subber.listen)
thread.start()

try: 
	while True:
		# #publish all boat values at 10hz interval
		# publish_gps_status()
		# publish_compas_status()
		# publish_internal_compass_status()
		calibrate_external_compass()
		time.sleep(.1)

# turn off all leds when program exits		
except KeyboardInterrupt:
	GPIO.output(13,GPIO.LOW)
	GPIO.output(19,GPIO.LOW)
	GPIO.output(26,GPIO.LOW)


