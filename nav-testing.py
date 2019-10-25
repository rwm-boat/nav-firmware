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
import numpy as np
import nvector as nv

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

M_PI = 3.14159265358979323846

#external compass hard iron distortion calibration values
e_magXmin = -0.4564
e_magYmin = -0.4388
e_magZmin = 0
e_magXmax = 0.44608
e_magYmax = 0.46336
e_magZmax = 0.66368

#calibration function values
ext_magXmin = 0
ext_magYmin = 0
ext_magXmax = 0
ext_magYmax = 0
ext_magZmax = 0
ext_magZmin = 0


IMU.detectIMU()     #Detect if BerryIMUv1 or BerryIMUv2 is connected.
IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass

a = datetime.datetime.now()

# Setup publisher
pubber = Publisher(client_id="nav-pubber")

speed = 0
current_lat = 0
current_lon = 0

#vector math
wgs84 = nv.FrameE(name='WGS84')
distance = 0

def publish_gps_status():
	global prev_pos
	global current_pos
	global total_distance
	global distance_traveled
	global speed
	global current_lat
	global current_lon
	try:
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
			current_pos = (agps_thread.data_stream.lat,agps_thread.data_stream.lon)
			current_lon = agps_thread.data_stream.lon
			current_lat = agps_thread.data_stream.lat
			message = {
				'time' :  agps_thread.data_stream.time,
				'latitude' : agps_thread.data_stream.lat,
				'longitude' : agps_thread.data_stream.lon,
				'speed': speed_kn,
				'course': agps_thread.data_stream.track,
				'distance': total_distance
			}
		
			app_json = json.dumps(message)
			pubber.publish("/status/gps",app_json)

	except Exception:
		# print("invalid gps values")
		pass

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
		speed = speed_kn
		prev_pos = current_pos

def calibrate_external_compass():
	mag_x, mag_y, mag_z = sensor.magnetic
	print(str(mag_x) + "," + str(mag_y))
	
	global ext_magXmax
	global ext_magXmin
	global ext_magYmax 
	global ext_magYmin 
	global ext_magZmin
	global ext_magZmax

	if(mag_x > ext_magXmax): ext_magXmax = mag_x
	if(mag_x < ext_magXmin): ext_magXmin = mag_x
	if(mag_y > ext_magYmax): ext_magYmax = mag_y
	if(mag_y < ext_magYmin): ext_magYmin = mag_y
	if(mag_z > ext_magZmax): ext_magZmax = mag_z
	if(mag_z < ext_magZmin): ext_magZmin = mag_z

	print("X Max: " + str(ext_magXmax))
	print("Y Max: " + str(ext_magYmax))
	print("X Min: " + str(ext_magXmin))
	print("Y Min: " + str(ext_magYmin))
	print("Z Min: " + str(ext_magZmin))
	print("Z Max: " + str(ext_magZmax))
	

def publish_compas_status():

	try:
		mag_x, mag_y, mag_z = sensor.magnetic
		temp = sensor.temperature
		
		# Apply hard iron distortion calibration 
		offset_x = (e_magXmax + e_magXmin) / 2
		offset_y = (e_magYmax + e_magYmin) / 2
		offset_z = (e_magZmax + e_magZmin) / 2

		corrected_x = mag_x - offset_x
		corrected_y = mag_y - offset_y
		corrected_z = mag_z - offset_z

		# Apply soft iron compass calibration
		avg_delta_x = (e_magXmax - e_magXmin) / 2
		avg_delta_y = (e_magYmax - e_magYmin) / 2
		avg_delta_z = (e_magZmax - e_magZmin) / 2
		avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3

		scale_x = avg_delta / avg_delta_x
		scale_y = avg_delta / avg_delta_y
		scale_z = avg_delta / avg_delta_z

		corrected_x = (mag_x - offset_x) * scale_x
		corrected_y = (mag_y - offset_y) * scale_y
		corrected_z = (mag_y - offset_z) * scale_z

		out_file = open("uncalibrated_compass.txt", "a")
		out_file.write(str(mag_x) + "," + str(mag_y) + "," + str(mag_z))
		out_file.write("\n")

		out_file = open("calibrated_compass.txt", "a")
		out_file.write(str(corrected_x) + "," + str(corrected_y) + "," + str(corrected_z))
		out_file.write("\n")

		#Calculate heading
		heading = 180 * math.atan2(corrected_x,corrected_y)/M_PI
		uncal_heading = 180 * math.atan2(mag_x,mag_y)/M_PI

		#Only have our heading between 0 and 360
		if heading < 0:
			heading += 360
		if uncal_heading < 0:
			uncal_heading += 360

		out_file = open("course_comparison.txt", "a")
		out_file.write(str(heading) + "," + str(uncal_heading) + "," + str(internal_compass) + "," + str(agps_thread.data_stream.track))
		out_file.write("\n")

		message = {
			'temp' : temp,
			'compass': heading,
		}
		app_json = json.dumps(message)
		pubber.publish("/status/compass",app_json)

	except Exception:
		#print("no external imu")
		pass

def publish_vector():

	global distance

	TARGET_RADIUS = 0.002

	# JSON for lat and lon locations
	gps_targets = [
		{
			"latitude" : 42.274982,
			"longitude" : -71.816890
		},
		{
			"latitude" : 42.275465,
			"longitude" : -71.816851
		}
	]

	for x in gps_targets:

		distance = 5 # initialize non-zero
		print("WAYPOINT HIT")

		while(distance > TARGET_RADIUS):

			try:
				target_lat = float(x['latitude'])
				target_lon = float(x['longitude'])
				target_pos = (target_lat,target_lon)
				distance = haversine(current_pos,target_pos,unit=Unit.NAUTICAL_MILES)
			except Exception:
				pass


				#print("non-valid gps values")
			
			# -------- MAGNITUDE CONSTANTS ---------
			# 5 - full chat
			# 4 - comfortable planning
			# 3 - min-plane-speed
			# 2 - max efficency displacement
			# 1 - low speed trolling
			# 0 - stop

			max_speed = 0.5 # ~ 100 meters
			plane = 0.5 
			min_plane = 0.5
			max_efficency = 0.15 # ~ 25m
			troll = 0.0025 # ~ 5m
			# ---------------------------------------

			# calculate vector between two points 
			target_point = wgs84.GeoPoint(latitude=target_lat, longitude=target_lon, z=0, degrees = True)
			current_point = wgs84.GeoPoint(latitude=current_lat, longitude=current_lon, z=0, degrees = True)
			p_AB_N = current_point.delta_to(target_point)
			azimuth = p_AB_N.azimuth_deg[0]
			angle = azimuth

			# calculate magnitude from distance
			try:
				distance = haversine(current_pos,target_pos, unit=Unit.NAUTICAL_MILES)
				
			except Exception:
				pass
				#("non-valid gps")
			if(distance > plane): magnitude = 5
			elif(distance > min_plane): magnitude = 4
			elif(distance > max_efficency): magnitude = 3
			else: magnitude = 1
			print("distance: " + str(distance))
			if(angle < 0):
				angle += 360
			# post message with new data
			message = {
				'heading' : angle,
				'magnitude' : magnitude,
			}
			print(message)
			app_json = json.dumps(message)
			pubber.publish("/status/vector",app_json)
			time.sleep(1)
	
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

vector_thread = Thread(target=publish_vector)
vector_thread.start()

counter = 0

try: 
	while True:
		#publish all boat values at 10hz interval
		publish_gps_status()
		publish_compas_status()
		#calibrate_external_compass()

		# if(counter > 9):
		# 	publish_vector()
		# 	counter = 0
		# counter += 1

		time.sleep(.1)

# turn off all leds when program exits		
except KeyboardInterrupt:
	GPIO.output(13,GPIO.LOW)
	GPIO.output(19,GPIO.LOW)
	GPIO.output(26,GPIO.LOW)


