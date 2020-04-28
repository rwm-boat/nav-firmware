import board
import busio
#import adafruit_lsm9ds0
import RPi.GPIO as GPIO
from mqtt_client.publisher import Publisher
import math
import json
import numpy
import datetime
from scipy.signal import butter, lfilter, freqz
from scipy import signal
import filterpy.kalman as kf
from filterpy.stats import gaussian

i2c = busio.I2C(board.SCL,board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# #external compass hard iron distortion calibration values
# e_magXmin = -0.55064
# e_magYmin = -0.47416
# e_magZmin = 0
# e_magXmax = 0.49216
# e_magYmax = 0.3976
# e_magZmax = 0.6792

# #calibration function values
# ext_magXmin = 0
# ext_magYmin = 0
# ext_magXmax = 0
# ext_magYmax = 0
# ext_magZmax = 0
# ext_magZmin = 0


# gyroXangle = 0.0
# gyroYangle = 0.0
# gyroZangle = 0.0
# CFangleX = 0.0
# CFangleY = 0.0


# i2c = busio.I2C(board.SCL, board.SDA)
# sensor = adafruit_lsm9ds0.LSM9DS0_I2C(i2c)

# M_PI = 3.14159265358979323846
# G_GAIN = 0.070
# RAD_TO_DEG = 57.29578
# AA =  0.40      # Complementary filter constant

pubber = Publisher(client_id="compass-pubber")

a = datetime.datetime.now()

# current =  0
# previous = 0

# ### sqrt of the variance is the error distance (aka. meters or degrees) ###
# # how much error there is in the process model (how much do we trust the model)
# process_var = 4.
# # how much error there is in each sensor measurement (how much do we trust the sensor)
# sensor_var = .3

# ## Initial State ##
# # [position, variance, sensor_var] sensor state
# # [(meters, degrees), (meters, degrees), ] ## remember sqrt of variance is distance error
# x = gaussian(0., 0., sensor_var) 


# #[position, velocity, process_var] of model initial state
# # how we think the system works
# process_model = gaussian(0., 0., process_var)

# def calibrate_external_compass():
# 	global ext_magXmax
# 	global ext_magXmin
# 	global ext_magYmax 
# 	global ext_magYmin 
# 	global ext_magZmin
# 	global ext_magZmax

# 	mag_x, mag_y, mag_z = sensor.magnetic
	
# 	print(str(mag_x) + "," + str(mag_y))

# 	if(mag_x > ext_magXmax): ext_magXmax = mag_x
# 	if(mag_x < ext_magXmin): ext_magXmin = mag_x
# 	if(mag_y > ext_magYmax): ext_magYmax = mag_y
# 	if(mag_y < ext_magYmin): ext_magYmin = mag_y
# 	if(mag_z > ext_magZmax): ext_magZmax = mag_z
# 	if(mag_z < ext_magZmin): ext_magZmin = mag_z

# 	print("X Max: " + str(ext_magXmax))
# 	print("Y Max: " + str(ext_magYmax))
# 	print("X Min: " + str(ext_magXmin))
# 	print("Y Min: " + str(ext_magYmin))
# 	print("Z Min: " + str(ext_magZmin))
# 	print("Z Max: " + str(ext_magZmax))

def publish_compas_status():

	print(sensor.magnetic)

		
	message = {
		'temp' : temp,
		'compass': heading,
		'gyro_z' : rate_gyr_z,
		'kalman_lp': low_pass_filter(kalman_filter(heading), .2),
		"kalman" : kalman_filter(heading)
	}
	print(message)
	app_json = json.dumps(message)
	pubber.publish("/status/compass",app_json)

	