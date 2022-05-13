#!/usr/bin/env python
# defines a LocomotionController object which
# !/usr/bin/env python
# interfaces with user input to control the robot.

from tokenize import Double
import vector
import math
import numpy as np
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy

servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096


class CamCtlr:
    # take range from -1 to 1 and traslate that to the angle we need to move the wheels.
    def __init__(self):
        self.CamPulse = int(servo_min + ((servo_max - servo_min) / 2))  # 150-600
        self.ArmPulse = servo_min  # 150-600
        self.LastJoy = None
        self.camAnglePub = rospy.Publisher('cam_angle', Int32, queue_size=1)
        self.sensorArmAnglePub = rospy.Publisher('sensor_arm_angle', Int32, queue_size=1)
        rospy.Subscriber("telemetry_joy", Joy, self.joyCallback, queue_size=1)
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            if not rospy.get_param("/isAutonomous") and not self.LastJoy == None:
                self.CamControl(self.LastJoy.axes[6], self.LastJoy.axes[7])
            r.sleep()
    
    def CamControl(self, _dPadX, _dPadY):
        if _dPadX < 0 and self.CamPulse > servo_min:
            self.CamPulse -= 3
        elif _dPadX > 0 and self.CamPulse < servo_max:
            self.CamPulse += 3
        
        if _dPadY < 0 and self.ArmPulse > servo_min:
            self.ArmPulse -= 3
        elif _dPadY > 0 and self.ArmPulse < servo_max:
            self.ArmPulse += 3
        
        self.camAnglePub.publish(Int32(self.CamPulse))
        self.sensorArmAnglePub.publish(Int32(self.ArmPulse))
        
        return (self.CamPulse, self.ArmPulse)
    
    def joyCallback(self, joy):
        # rospy.logwarn(joy)
        self.LastJoy = joy


#    def autonomyCallback(self,twist):
#        global locController
#        if rospy.get_param("/isAutonomous"):
#            print("autonomy twist recieved")
#
#            linear_x = twist.linear.x
#            angular_z = twist.angular.z
#
#            left_speed = linear_x + angular_z
#            right_speed = linear_x - angular_z
#
#            locController.tankSteer(left_speed, right_speed)

if __name__ == "__main__":
    rospy.init_node('cameras')
    camController = CamCtlr()
