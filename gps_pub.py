from gps3.agps3threaded import AGPS3mechanism
import time
import json
from mqtt_client.publisher import Publisher
from haversine import haversine,Unit

# GPS Setup
agps_thread = AGPS3mechanism()  # Instantiate AGPS3 Mechanisms
agps_thread.stream_data()  # From localhost (), or other hosts, by example, (host='gps.ddns.net')
agps_thread.run_thread()  # Throttle time to sleep after an empty lookup, default '()' 0.2 two tenths of a second

#global variables
speed = 0
current_lat = 0
current_lon = 0
distance_traveled = 0
total_distance = 0
current_pos = (0,0)
prev_pos = (0,0)

# Setup publisher
pubber = Publisher(client_id="gps-pubber")

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
		else:
			speed_kn = 0

	except Exception:
		print("GPS Failure")

		
		# check to see if the gps has a fix, if it does, turn on LED's
		led_on_message = {
			'led_id' : 19,
			'command' : 1
		}

		led_off_message = {
			'led_id' : 19,
			'command' : 0
		}
		if(agps_thread.data_stream.time == 'n/a'):
			app_json = json.dumps(led_off_message)
			pubber.publish("/command/led",app_json)
		else:
			app_json = json.dumps(led_on_message)
			pubber.publish("/command/led",app_json)
		
		print(message)
		app_json = json.dumps(message)
		pubber.publish("/status/gps",app_json)
		
		speed = speed_kn
		prev_pos = current_pos