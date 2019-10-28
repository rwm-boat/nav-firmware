import board
import busio
import adafruit_lsm9ds0
import RPi.GPIO as GPIO
from mqtt_client.publisher import Publisher

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

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds0.LSM9DS0_I2C(i2c)

M_PI = 3.14159265358979323846

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

		# out_file = open("uncalibrated_compass.txt", "a")
		# out_file.write(str(mag_x) + "," + str(mag_y) + "," + str(mag_z))
		# out_file.write("\n")

		#Calculate heading
		heading = 180 * math.atan2(corrected_x,corrected_y)/M_PI


		#Only have our heading between 0 and 360
		if heading < 0:
			heading += 360
		message = {
			'temp' : temp,
			'compass': heading,
		}
		print(message)
		app_json = json.dumps(message)
		pubber.publish("/status/compass",app_json)

	