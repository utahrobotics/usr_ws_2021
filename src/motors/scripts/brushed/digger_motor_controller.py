#!/usr/bin/env python3
import rospy

from std_msgs.msg import Float32
import os
print(os.getcwd())

from pyvesc import VESC

from angle_sense import AngleSensor

# serial port that VESC is connected to. Something like "COM3" for windows and as below for linux/mac
#serial_port1 = '/dev/FSESC_drum'
serial_port1 = '/dev/ttyACM0'
serial_port2 = '/dev/ttyACM1'

class DrivingSubscriber:
    #This class is responsible for driving all of the Maxon motor controllers using published information from the 
    # Mobility node
    def __init__(self):
        self.drum_subscriber_ = rospy.Subscriber('drum_vel', Float32, self.drum_drive_callback,queue_size=1)

        self.arm_subscriber_ = rospy.Subscriber('arm_vel', Float32, self.arm_callback,queue_size=1)
        self.arm_angle_pub = rospy.Publisher('/sensors/angleSensor/angle', Float32 , queue_size=10)

        #self.drum_motor = VESC(serial_port=serial_port1)
        self.arm_motor = VESC(serial_port=serial_port2)

        self.sensor = AngleSensor()
        #self.sensor.start_adc(0)

    def __del__(self):
         self.drum_motor.set_duty_cycle(0)
         self.arm_motor.set_duty_cycle(0)

    def drum_drive_callback(self, msg):
        return
        drum_vel = msg.data
        #rospy.logwarn(drum_vel)
        self.drum_motor.set_duty_cycle(drum_vel)
        
    def arm_callback(self, msg):
        return
        arm_vel = msg.data
        #rospy.logwarn(arm_vel)
        self.arm_motor.set_duty_cycle(arm_vel)

    def poll(self):
        self.arm_motor.set_duty_cycle(1.0)
        pass
        #self.arm_angle_pub.publish(self.sensor.computeDegrees(5, self.sensor.computeVolts(self.sensor.get_last_result())))

#    def getAngle():
#        return self.sensor.computeDegrees(5, self.sensor.computeVolts(self.sensor.get_last_result()))


def main(args=None):
    rospy.init_node('digger_controller')
    sub_node = DrivingSubscriber()
    r = rospy.Rate(20)          # 20hz
    while not rospy.is_shutdown():
        r.sleep()
        sub_node.poll()
    #rospy.shutdown()       # does not exist


if __name__ == '__main__':
    main()
