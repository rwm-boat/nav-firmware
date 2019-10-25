

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
