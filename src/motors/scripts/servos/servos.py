#!/usr/bin/env python
from __future__ import division
import time
import rospy
import tf
from std_msgs.msg import Int32
import Adafruit_PCA9685

servo_min = 100  # Min pulse length out of 4096
servo_max = 650  # Max pulse length out of 4096

angle_min = -25
angle_max = 270


class Servos:
    #This class is responsible for driving all of the Maxon motor controllers using published information from the 
    # Mobility node
    def __init__(self):
        self.cam_subscriber_ = rospy.Subscriber('cam_angle', Int32, self.cam_callback,queue_size=1)
        self.sensor_arm_subscriber_ = rospy.Subscriber('sensor_arm_angle', Int32, self.sensor_arm_callback,queue_size=1)

	self.br = tf.TransformBroadcaster()

	self.pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)

	# Set frequency to 60hz, good for servos.
	self.pwm.set_pwm_freq(60)

    def cam_callback(self, msg):
    	camAng = msg.data
        angle = ((camAng - servo_max) / (servo_max - servo_min) * (angle_max-angle_min)) + angle_min
	#TODO: get accurate translation measurements
	self.br.sendTransform((0.1, 0.1, 0.4), tf.transformations.quaternion_from_euler(0,0,angle),
				rospy.Time.now(), "servo_link", "base_link")
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
