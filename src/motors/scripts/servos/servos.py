#!/usr/bin/env python
from __future__ import division
import time
import rospy
from std_msgs.msg import Int32
import Adafruit_PCA9685


class Servos:
    #This class is responsible for driving all of the Maxon motor controllers using published information from the 
    # Mobility node
    def __init__(self):
        self.cam_subscriber_ = rospy.Subscriber('cam_angle', Int32, self.cam_callback,queue_size=1)
        self.sensor_arm_subscriber_ = rospy.Subscriber('sensor_arm_angle', Int32, self.sensor_arm_callback,queue_size=1)
	self.pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)

	# Set frequency to 60hz, good for servos.
	self.pwm.set_pwm_freq(60)

    def cam_callback(self, msg):
    	camAng = msg.data
	self.pwm.set_pwm(7, 0, camAng)
    	
    def sensor_arm_callback(self, msg):
    	armAng = msg.data
	self.pwm.set_pwm(6, 0, armAng)



def main(args=None):
	rospy.init_node('servo_motors')
	Servos()
	rospy.spin()
	# Destroy the node explicitly
	rospy.shutdown()

if __name__ == '__main__':
    main()
