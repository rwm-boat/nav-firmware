import board
import busio
import adafruit_bno055
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

#external compass hard iron distortion calibration values
e_magXmin = -26.25
e_magYmin = -43.0
e_magZmin = -52.875
e_magXmax = 50.6875
e_magYmax = 37.875
e_magZmax = 0

#calibration function values
ext_magXmin = 0
ext_magYmin = 0
ext_magXmax = 0
ext_magYmax = 0
ext_magZmax = 0
ext_magZmin = 0


gyroXangle = 0.0
gyroYangle = 0.0
gyroZangle = 0.0
CFangleX = 0.0
CFangleY = 0.0

M_PI = 3.14159265358979323846
G_GAIN = 0.070
RAD_TO_DEG = 57.29578
AA =  0.40      # Complementary filter constant

pubber = Publisher(client_id="compass-pubber")

a = datetime.datetime.now()

current =  0
previous = 0

### sqrt of the variance is the error distance (aka. meters or degrees) ###
# how much error there is in the process model (how much do we trust the model)
process_var = 4.
# how much error there is in each sensor measurement (how much do we trust the sensor)
sensor_var = .3

## Initial State ##
# [position, variance, sensor_var] sensor state
# [(meters, degrees), (meters, degrees), ] ## remember sqrt of variance is distance error
x = gaussian(0., 0., sensor_var) 


#[position, velocity, process_var] of model initial state
# how we think the system works
process_model = gaussian(0., 0., process_var)

def calibrate_external_compass():
	global ext_magXmax
	global ext_magXmin
	global ext_magYmax 
	global ext_magYmin 
	global ext_magZmin
	global ext_magZmax

	mag_x, mag_y, mag_z = sensor.magnetic
	
	print(str(mag_x) + "," + str(mag_y))

	if(mag_x > ext_magXmax): ext_magXmax = mag_x
	if(mag_x < ext_magXmin): ext_magXmin = mag_x
	if(mag_y > ext_magYmax): ext_magYmax = mag_y
	if(mag_y < ext_magYmin): ext_magYmin = mag_y
	if(mag_z > ext_magZmax): ext_magZmax = mag_z
	if(mag_z < ext_magZmin): ext_magZmin = mag_z

	print("X Max: " + str(ext_magXmax))
	print("Y Max: " + str(ext_magYmax))
	print("Z Max: " + str(ext_magZmax))
	print("X Min: " + str(ext_magXmin))
	print("Y Min: " + str(ext_magYmin))
	print("Z Min: " + str(ext_magZmin))

def publish_compas_status():
		global a 
		global gyroXangle
		global gyroYangle
		global gyroZangle
		global CFangleX
		global CFangleY


		mag_x, mag_y, mag_z = sensor.magnetic
		temp = sensor.temperature
		accel_x, accel_y, accel_z = sensor.acceleration
		gyro_x, gyro_y, gyro_z = sensor.gyro
		
		# Apply hard iron distortion calibration 
		offset_x = (e_magXmax + e_magXmin) / 2
		offset_y = (e_magYmax + e_magYmin) / 2
		offset_z = (e_magZmax + e_magZmin) / 2

		corrected_x = mag_x - offset_x
		corrected_y = mag_y - offset_y
		corrected_z = mag_z - offset_z

		mag_x = corrected_x
		mag_y = corrected_y
		mag_z = corrected_z

		# # Apply soft iron compass calibration
		# avg_delta_x = (e_magXmax - e_magXmin) / 2
		# avg_delta_y = (e_magYmax - e_magYmin) / 2
		# avg_delta_z = (e_magZmax - e_magZmin) / 2
		# avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3

		# scale_x = avg_delta / avg_delta_x
		# scale_y = avg_delta / avg_delta_y
		# scale_z = avg_delta / avg_delta_z

		# corrected_x = (mag_x - offset_x) * scale_x
		# corrected_y = (mag_y - offset_y) * scale_y
		# corrected_z = (mag_y - offset_z) * scale_z

		##Calculate loop Period(LP). How long between Gyro Reads
		b = datetime.datetime.now() - a
		a = datetime.datetime.now()
		LP = b.microseconds/(1000000*1.0)
		#print "Loop Time | %5.2f|" % ( LP ),

		# Convert Gyro raw to degrees per second
		rate_gyr_x =  gyro_x * G_GAIN
		rate_gyr_y =  gyro_y * G_GAIN
		rate_gyr_z =  gyro_z * G_GAIN

		#Calculate the angles from the gyro. 
		gyroXangle+=rate_gyr_x*LP
		gyroYangle+=rate_gyr_y*LP
		gyroZangle+=rate_gyr_z*LP

		#Convert Accelerometer values to degrees
		AccXangle =  (math.atan2(accel_x,accel_z)*RAD_TO_DEG)
		AccYangle =  (math.atan2(accel_z,accel_x)+M_PI)*RAD_TO_DEG

		#convert the values to -180 and +180
		if AccYangle > 90:
			AccYangle -= 270.0
		else:
			AccYangle += 90.0

		#Complementary filter used to combine the accelerometer and gyro values.
		CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
		CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle

		#Normalize accelerometer raw values.
		accXnorm = accel_x/math.sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z)
		accYnorm = accel_y/math.sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z)

		#Calculate pitch and roll
		pitch = math.asin(accXnorm)
		roll = -math.asin(accYnorm/math.cos(pitch))
		
    	#Calculate the new tilt compensated values
		magXcomp = mag_x*math.cos(pitch)+mag_z*math.sin(pitch)
		magYcomp = mag_x*math.sin(roll)*math.sin(pitch)+mag_y*math.cos(roll)-mag_z*math.sin(roll)*math.cos(pitch)   #LSM9DS0

		#Calculate heading
		heading = round(numpy.degrees(math.atan2(magYcomp,magXcomp)))+ 90

		#Only have our heading between 0 and 360
		if heading < 0:
		 	heading += 360

		def low_pass_filter(data, a):

			global previous
					
			output = (a*data) + (1-a) * previous
			previous = output

			return output
    			    			
		def kalman_filter(z):
    			
			global x
			global process_model
			
  			# -------- PREDICT --------- #
			# X is the state of the system
			# P is the variance of the system
			# u is the movement of the system due to the process
			# Q is the noise of the process
			x, P = kf.predict(x=x, P=process_model)
			# sensor says z with a standard deviation of sensorvar**2
			# probability of the measurement given the current state 
			#likelihood = gaussian(z, mean(z), sensor_var) 

			# -------- UPDATE --------- #
			# X is the state of the system
			# P is the variance of the system
			# z is the measurement
			# R is the measurement variance
			x, P = kf.update(x=x, P=P, z=z, R=sensor_var)
			# print( "x: ",'%.3f' % x, "var: " '%.3f' % P, "z: ", '%.3f' % z)
			#print(x)
			return x

		
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

	
