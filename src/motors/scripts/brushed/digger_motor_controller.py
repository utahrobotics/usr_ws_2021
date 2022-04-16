#!/usr/bin/env python3
import rospy

from std_msgs.msg import Float32
import os
print(os.getcwd())

from pyvesc import VESC

#from angle_sense import AngleSensor

# serial port that VESC is connected to. Something like "COM3" for windows and as below for linux/mac
serial_port1 = '/dev/FSESC_drum'
serial_port2 = '/dev/FSESC_arm'

class DrivingSubscriber:
    #This class is responsible for driving all of the Maxon motor controllers using published information from the 
    # Mobility node
    def __init__(self):
        self.drum_subscriber_ = rospy.Subscriber('drum_vel', Float32, self.drum_drive_callback,queue_size=1)

        self.arm_subscriber_ = rospy.Subscriber('arm_vel', Float32, self.arm_callback,queue_size=1)

        self.drum_motor = VESC(serial_port=serial_port1)
        self.arm_motor = VESC(serial_port=serial_port2)

        #self.sensor = AngleSensor()
        #self.sensor.start_adc(0)

    def __del__(self):
         self.drum_motor.set_duty_cycle(0)
         self.arm_motor.set_duty_cycle(0)

    def drum_drive_callback(self, msg):
    	drum_vel = msg.data
    	rospy.logwarn(drum_vel)
    	self.drum_motor.set_duty_cycle(drum_vel)
    	
    def arm_callback(self, msg):
    	arm_vel = msg.data
    	#rospy.logwarn(arm_vel)
    	self.arm_motor.set_duty_cycle(arm_vel)

#    def getAngle():
#        return self.sensor.computeDegrees(5, self.sensor.computeVolts(self.sensor.get_last_result()))


def main(args=None):
	rospy.init_node('minimal_subscriber')
	sub_node = DrivingSubscriber()
	rospy.spin()
	# Destroy the node explicitly
	rospy.shutdown()

if __name__ == '__main__':
    main()
