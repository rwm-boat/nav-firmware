import math
import gps_pub
from haversine import haversine,Unit
import numpy as np
import nvector as nv
from mqtt_client.publisher import Publisher
import time
import json

# Setup publisher
pubber = Publisher(client_id="vector-pubber")

#vector math
wgs84 = nv.FrameE(name='WGS84')
distance = 0

def publish_vector():

	global distance
	# --------- CONSTANTS ---------
	# 5 - full chat
	# 4 - comfortable planning
	# 3 - min-plane-speed
	# 2 - max efficency displacement
	# 1 - low speed trolling
	# 0 - stop
	FULL_CHAT = 0.5 # ~ 100 meters
	PLANE = 0.5 
	MIN_PLANE = 0.5
	MAX_EFFICENCY = 0.00674946 # ~ 25m
	TROLL = 0.008 # ~ 15m radius

	TARGET_RADIUS = 0.00269978 # 5m radius
	TARGET_PUB_FREQ = 4 # seconds between transmissions

	GETTINGCLOSE_RADIUS = 0.00431965 # 8m radius
	GETTINGCLOSE_PUB_FREQ = 1 # seconds between transmissions
	# -------------------------------

	
	with open('gps_waypoints.txt', "r") as json_file:
		#itterate through list of gps targets from text file
		for line in json_file.readlines():
			x = json.loads(line)
			distance = 5 # initialize non-zero
			
			# while you have not yet hit the target create vectors
			while(distance > TARGET_RADIUS):
				try:
					target_lat = float(x['latitude'])
					target_lon = float(x['longitude'])
					target_pos = (target_lat,target_lon)
					cur_lat = float(gps_pub.current_lat)
					cur_lon = float(gps_pub.current_lon)
					current_position = (cur_lat,cur_lon)
					distance = haversine(current_position,target_pos,unit=Unit.NAUTICAL_MILES)
				except Exception:
					print("vector pub not receiving valid gps")
				
				if(distance < 10): # get rid of outliers
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
					# elif(distance > PLANE): magnitude = 5
					# elif(distance > MIN_PLANE): magnitude = 4
					# elif(distance > MAX_EFFICENCY): magnitude = 3
					elif(distance > TROLL): magnitude = 2
					elif(distance < TROLL): magnitude = 1
					elif(distance < TARGET_RADIUS): magnitude = 0
					print("distance: " + str(distance))
			
					# post message with new data
					message = {
						'heading' : angle,
						'magnitude' : magnitude,
					}
					print(message)
					app_json = json.dumps(message)
					pubber.publish("/status/vector",app_json)

					# once the boat hits the getting close distance, it speeds up transmissions
					if(distance < GETTINGCLOSE_RADIUS):
						time.sleep(GETTINGCLOSE_PUB_FREQ)
					else: time.sleep(TARGET_PUB_FREQ)

					

