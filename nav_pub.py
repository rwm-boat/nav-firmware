
import RPi.GPIO as GPIO
import numpy

import time
import json
from mqtt_client.publisher import Publisher
from mqtt_client.subscriber import Subscriber
from threading import Thread
import datetime
import math
from haversine import haversine,Unit
import numpy as np
import nvector as nv


a = datetime.datetime.now()

# Setup publisher
pubber = Publisher(client_id="nav-pubber")



#vector math
wgs84 = nv.FrameE(name='WGS84')
distance = 0


def publish_vector():

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
	#itterate through list of gps targets
	for x in gps_targets:

		distance = 5 # initialize non-zero

	# while you have not yet hit the target create vectors
		while(distance > TARGET_RADIUS):

			try:
				target_lat = float(x['latitude'])
				target_lon = float(x['longitude'])
				target_pos = (target_lat,target_lon)
				cur_lat = float(current_lat)
				cur_lon = float(current_lon)
				if(cur_lat is not 0 and cur_lon is not 0):
					current_position = (cur_lat,cur_lon)
				distance = haversine(current_position,target_pos,unit=Unit.NAUTICAL_MILES)
			except Exception:
				pass

			# calculate vector between two points 
			target_point = wgs84.GeoPoint(latitude=target_lat, longitude=target_lon, z=0, degrees = True)
			current_point = wgs84.GeoPoint(latitude=cur_lat, longitude=cur_lon, z=0, degrees = True)
			p_AB_N = current_point.delta_to(target_point)
			azimuth = p_AB_N.azimuth_deg[0]
			angle = azimuth
			if(angle < 0):
				angle += 360
			
			# calculate magnitude from distance
			if(distance > 10): magnitude = 1 # remove outliers
			elif(distance > plane): magnitude = 5
			elif(distance > min_plane): magnitude = 4
			elif(distance > max_efficency): magnitude = 3
			else: magnitude = 1
			print("distance: " + str(distance))
			print(current_pos)
			print(target_pos)
			

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

# new thread for publishing the vector at 1hz
vector_thread = Thread(target=publish_vector)
vector_thread.start()


# ------------- main program loop ----------------

#publish all boat values at 10hz interval
try: 
	while True:
		publish_gps_status()
		publish_compas_status()
		time.sleep(.1)

except KeyboardInterrupt:
	# turn off all leds when program exits
	GPIO.setup(13,GPIO.OUT)
	GPIO.setup(19,GPIO.OUT)
	GPIO.setup(26,GPIO.OUT)

	GPIO.output(13,GPIO.LOW)
	GPIO.output(19,GPIO.LOW)
	GPIO.output(26,GPIO.LOW)