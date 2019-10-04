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

# GPS Setup
agps_thread = AGPS3mechanism()  # Instantiate AGPS3 Mechanisms
agps_thread.stream_data()  # From localhost (), or other hosts, by example, (host='gps.ddns.net')
agps_thread.run_thread()  # Throttle time to sleep after an empty lookup, default '()' 0.2 two tenths of a second

# I2C connection:
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds0.LSM9DS0_I2C(i2c)

# Setup Pubber
pubber = Publisher(client_id="nav-pubber")

def publish_gps_status():
    message = {
        'time' :  agps_thread.data_stream.time,
        'latitude' : agps_thread.data_stream.lat,
        'longitude' : agps_thread.data_stream.lon,
        'speed': agps_thread.data_stream.speed,
        'course': agps_thread.data_stream.track
    }
    app_json = json.dumps(message)
    pubber.publish("/status/gps",app_json)

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
    
def on_led_command(client, userdata, message):
    obj = json.loads(message.payload.decode('utf-8'))
    led_selector = obj['led_id']
    led_opp = obj['command']
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(26,GPIO.OUT)
        GPIO.output(led_selector,GPIO.HIGH)
    except Exception:
        print("LED FAILTURE")


# Setup Subscriber to change LED
default_subscriptions = {
    "/command/led": on_led_command,
}
subber = Subscriber(client_id="led_actuator", broker_ip="192.168.1.170", default_subscriptions=default_subscriptions)

thread = Thread(target=subber.listen)
thread.start()

#subber.listen()

while True:
    publish_gps_status()
    publish_compas_status()
    time.sleep(1)


