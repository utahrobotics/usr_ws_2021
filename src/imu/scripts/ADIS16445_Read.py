#!/usr/bin/env python
"""
This node is the communication layer betweeen the USR Ros subsystem and the stepper motor controllers.
"""
# TODO: add recieving info from the stepper controller

#import rclpy
#from rclpy.node import Node

import rospy
import serial
import time
import threading
import math

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Header



class ADIS16445Reader():
	# This class is responsible for driving all of the Maxon motor controllers using published information from the
	# Mobility node
	def __init__(self):
		rospy.init_node('ADIS16445_reader')
		self.pub = rospy.Publisher('sensors/imu/imu', Imu, queue_size=10)
		self._mc = serial.Serial("/dev/imuteensy", 115200, timeout=.1)
		time.sleep(1)  # give the connection a second to settle

		read_thread = threading.Thread(target=self.readSerial)
		read_thread.setDaemon(True) 
		read_thread.start()
        
	def readSerial(self):
		while True:
			try:
				data = self._mc.readline();
				if data:
					print(data)
					ADIS16445Reader.parseIMU(data)

			except serial.SerialException:
				rospy.logwarn("Tried to read from uninitialized micro-controller")

#"12345|25000|25000|25000|20000|20000|20000|1002"
	def parseImu(data):
		vals=data.split("|")
		DIAG_STAT = "{0:016b}".format(int(vals[0]))
		XGYRO_OUT = (int(vals[1])/100.0) * (math.pi/180.0) #rad/sec
		YGYRO_OUT = (int(vals[2])/100.0) * (math.pi/180.0) #rad/sec
		ZGYRO_OUT = (int(vals[3])/100.0) * (math.pi/180.0) #rad/sec
		XACCL_OUT = (int(vals[4])/4000.0) * 9.8066 #m/s^2
		YACCL_OUT = (int(vals[5])/4000.0) * 9.8066 #m/s^2
		ZACCL_OUT = (int(vals[6])/4000.0) * 9.8066 #m/s^2
		TEMP_OUT = ADIS16445Reader.translate(int(vals[7]), -962, 1002, -40, 105)
		print("DIAG_STAT: " + DIAG_STAT)
		print("XGYRO_OUT: " + str(XGYRO_OUT))
		print("YGYRO_OUT: " + str(YGYRO_OUT))
		print("ZGYRO_OUT: " + str(ZGYRO_OUT))
		print("XACCL_OUT: " + str(XACCL_OUT))
		print("YACCL_OUT: " + str(YACCL_OUT))
		print("ZACCL_OUT: " + str(ZACCL_OUT))
		print("TEMP_OUT: " + str(TEMP_OUT))
		imuMsg = Imu()

		header = Header()
		header.stamp = rospy.Time.now()

		angular_velocity = Vector3()
		angular_velocity.x = XGYRO_OUT
		angular_velocity.y = YGYRO_OUT
		angular_velocity.z = ZGYRO_OUT
		angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

		linear_acceleration = Vector3()
		linear_acceleration.x = XACCL_OUT
		linear_acceleration.y = YACCL_OUT
		linear_acceleration.z = ZACCL_OUT
		linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

		orientation_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]

		imuMsg.header = header
		imuMsg.angular_velocity = angular_velocity
		imuMsg.angular_velocity_covariance = angular_velocity_covariance
		imuMsg.linear_acceleration = linear_acceleration
		imuMsg.linear_acceleration_covariance = linear_acceleration_covariance
		imuMsg.orientation_covariance = orientation_covariance

		self.pub.publish(imuMsg)

	
	def translate(value, leftMin, leftMax, rightMin, rightMax):
		# Figure out how 'wide' each range is
		leftSpan = leftMax - leftMin
		rightSpan = rightMax - rightMin

		# Convert the left range into a 0-1 range (float)
		valueScaled = float(value - leftMin) / float(leftSpan)

		# Convert the 0-1 range into a value in the right range.
		return rightMin + (valueScaled * rightSpan)


def main(args=None):
	reader = ADIS16445Reader()
	reader.parseImu("")
	rospy.spin()


if __name__ == '__main__':
	main()

