from IMU import IMU
import math
import numpy as np
import time
imu = IMU()
while True:
	m = imu.get_measurements()
	print '%f \t %f \t %f \t %f' %(m[0],m[1],m[2],np.arctan2(m[4],m[3])*180/3.14)
	time.sleep(0.1)	
