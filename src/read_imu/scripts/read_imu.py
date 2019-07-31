#!/usr/bin/env python  
import rospy
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time
import signal
import sys
import math
from sensor_msgs.msg import Imu
import numpy as np
from IMU import IMU

def publish_imu():
	rospy.init_node('imu_publisher', anonymous=True)
	#imu_topic = rospy.get_param('~imu_topic')
	pub = rospy.Publisher('imu_data', Imu, queue_size=1)

	#initialize Imu
        imu = IMU()
        imu.calibrate(100)

	#rospy.init_node('sonar_publisher', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	current_meas = Imu()
	while not rospy.is_shutdown():
                m = imu.get_measurements()
                current_meas.header.stamp = rospy.get_rostime()
                current_meas.linear_acceleration.x = m[0]
                current_meas.linear_acceleration.y = m[1]
                current_meas.linear_acceleration.z = m[2]
                current_meas.orientation.x = m[3]
                current_meas.orientation.y = m[4]
                current_meas.orientation.z = m[5]
                pub.publish(current_meas)
                rate.sleep()

if __name__ == '__main__':
    try:
        publish_imu()
    except rospy.ROSInterruptException:
        pass
