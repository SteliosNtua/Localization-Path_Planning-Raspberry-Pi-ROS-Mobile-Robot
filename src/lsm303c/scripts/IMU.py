#!/usr/bin/env python
# Distributed with a free-will license.
# Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
# LSM303DLHC
# This code is designed to work with the LSM303DLHC_I2CS I2C Mini Module available from ControlEverything.com.
# https://www.controleverything.com/products
import rospy
import RPi.GPIO as GPIO
import time
import signal
import math
import smbus
import numpy as np
import sys
from std_msgs.msg import String
class IMU:
	def __init__(self):
		# Get I2C bus
		self.bus = smbus.SMBus(1)
		self.xm = 0.0
		self.ym = 0.0
		# LSM303DLHC Accl address, 0x1D(25)
		# Select control register1, 0x20(32)
		#		0x27(39)	Acceleration data rate = 10Hz, Power ON, X, Y, Z axis enabled
		self.bus.write_byte_data(0x1D, 0x20, 0x27)
		# LSM303DLHC Accl address, 0x1D(25)
		# Select control register4, 0x23(35)
		#		0x00(00)	Continuos update, Full scale selection = +/-2g,
		self.bus.write_byte_data(0x1D, 0x23, 0x00)

		time.sleep(0.5)

	def get_measurements(self):
		# LSM303DLHC Accl address, 0x1D(25)
		# Read data back from 0x28(40), 2 bytes
		# X-Axis Accl LSB, X-Axis Accl MSB
		data0 = self.bus.read_byte_data(0x1D, 0x28)
		data1 = self.bus.read_byte_data(0x1D, 0x29)

		# Convert the data
		xAccl = data1 * 256 + data0
		if xAccl > 32767 :
			xAccl -= 65536
		
		# LSM303DLHC Accl address, 0x1D(25)
		# Read data back from 0x2A(42), 2 bytes
		# Y-Axis Accl LSB, Y-Axis Accl MSB
		data0 = self.bus.read_byte_data(0x1D, 0x2A)
		data1 = self.bus.read_byte_data(0x1D, 0x2B)
		
		# Convert the data
		yAccl = data1 * 256 + data0
		if yAccl > 32767 :
			yAccl -= 65536
		
		# LSM303DLHC Accl address, 0x1D(25)
		# Read data back from 0x2C(44), 2 bytes
		# Z-Axis Accl LSB, Z-Axis Accl MSB
		data0 = self.bus.read_byte_data(0x1D, 0x2C)
		data1 = self.bus.read_byte_data(0x1D, 0x2D)
		
		# Convert the data
		zAccl = data1 * 256 + data0
		if zAccl > 32767 :
			zAccl -= 65536
		
		# LSM303DLHC Mag address, 0x1E(30)
		# Select MR register, 0x02(02)
		#		0x00(00)	Continous conversion mode
		self.bus.write_byte_data(0x1E, 0x22, 0x00)
		# LSM303DLHC Mag address, 0x1E(30)
		# Select CRA register, 0x00(00)
		#		0x10(16)	Temperatuer disabled, Data output rate = 15Hz
		self.bus.write_byte_data(0x1E, 0x20, 0x10)
		# LSM303DLHC Mag address, 0x1E(30)
		# Select CRB register, 0x01(01)
		#		0x20(32)	Gain setting = +/- 1.3g
		self.bus.write_byte_data(0x1E, 0x21, 0x20)
		
		#time.sleep(0.5)
		
		# LSM303DLHC Mag address, 0x1E(30)
		# Read data back from 0x03(03), 2 bytes
		# X-Axis Mag MSB, X-Axis Mag LSB
		data0 = self.bus.read_byte_data(0x1E, 0x29)
		data1 = self.bus.read_byte_data(0x1E, 0x28)
		
		# Convert the data
		xMag = data0 * 256 + data1
		if xMag > 32767 :
			xMag -= 65536
		
		# LSM303DLHC Mag address, 0x1E(30)
		# Read data back from 0x05(05), 2 bytes
		# Y-Axis Mag MSB, Y-Axis Mag LSB
		data0 = self.bus.read_byte_data(0x1E, 0x2B)
		data1 = self.bus.read_byte_data(0x1E, 0x2A)
		
		# Convert the data
		yMag = data0 * 256 + data1
		if yMag > 32767 :
			yMag -= 65536
		
		# LSM303DLHC Mag address, 0x1E(30)
		# Read data back from 0x07(07), 2 bytes
		# Z-Axis Mag MSB, Z-Axis Mag LSB
		data0 = self.bus.read_byte_data(0x1E, 0x2C)
		data1 = self.bus.read_byte_data(0x1E, 0x2D)
		
		# Convert the data
		zMag = data0 * 256 + data1
		if zMag > 32767 :
			zMag -= 65536
		return xAccl,yAccl,zAccl,xMag-self.xm,yMag-self.ym,zMag		

	def calibrate(self,N):
		#take N measurements while the robot is spining
		#it must make at least one revolution
		#we want the robot spinning
		data = np.zeros((N,2)) 
		print 'Rotate the magnetometer 360 degrees'
		for i in range(N):
			sys.stdout.write('Finished '  + str(float(i)/N*100) +'%\r')
			sys.stdout.flush()
			m = self.get_measurements()
			data[i,0] = m[3]
			data[i,1] = m[4]
			time.sleep(0.1)
		self.xm = (data[:,0].min() + data[:,0].max())/2
		self.ym = (data[:,1].min() + data[:,1].max())/2
		return

def publish_imu():
   		pub = rospy.Publisher('imu_topic',String,queue_size=150)
		rospy.init_node('imu_topic', anonymous=True)
		rate=rospy.Rate(10)
		imu = IMU()
		imu.calibrate(100)
		while not rospy.is_shutdown():
			m = imu.get_measurements()
			#print '%f \t %f \t %f \t %f \t %f \t %f' %m # %(m[0],m[1],m[2],np.arctan2(m[4],m[3])*180/3.14)
			#print '%f' %(np.arctan2(m[4],m[3])*180/3.14 + 180)
			hello_str= "%f %f %f %f" %(m[0],m[1],m[2],np.arctan2(m[4],m[3]))
			rospy.loginfo(hello_str)
			pub.publish(hello_str)
			rate.sleep()

if __name__ == '__main__':
    try:
        publish_imu()
    except rospy.ROSInterruptException:
    	# disable GPIO
	GPIO.cleanup()
        pass
