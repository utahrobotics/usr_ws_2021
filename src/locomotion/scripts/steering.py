#!/usr/bin/env python
# defines a LocomotionController object which
# interfaces with user input to control the robot.

from tokenize import Double
import vector
import math
import numpy as np
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from locomotion.msg import SteerAndThrottle, BypassVelocity


class LocCtlr:
# take range from -1 to 1 and traslate that to the angle we need to move the wheels.
    scale = 0
    throttle = 0.0
    frontAngle = 0.0
    backAngle = 0.0
    rightMotion = 0.0
    leftMotion = 0.0
    wheelDist = 1.0
    currentStartButtonState = False
    previousStartButtonState = False
    steeringType =0

    def __init__(self, _pub, Scale = 1):
        self.pub = _pub
        self.angle=0.0
        self.rightInput=0.0

    def tankSteer(self,left_joystick, right_joystick):
        self.leftMotion = left_joystick
        self.rightMotion = right_joystick
        angles = [90, 90, 90, 90]
        velocities = [self.leftMotion, self.leftMotion, self.rightMotion, self.rightMotion]
        H = Header()
        H.stamp = rospy.Time.now()
        msg = SteerAndThrottle()
        msg.header = H
        msg.angles = angles
        msg.throttles = velocities
        self.pub.publish(msg)
        return (angles, velocities)

    def translationControl(self, left_joystickY, right_joystickX):
        self.wheelCenter = [0, 0.5]
        angle = (right_joystickX * 90)
        angles = [90 - angle, 90 - angle, 90 - angle, 90 - angle]

        vel = left_joystickY
        velocities = [vel, vel, vel, vel]
        H = Header()
        H.stamp = rospy.Time.now()
        msg = SteerAndThrottle()
        msg.header = H
        msg.angles = angles
        msg.throttles = velocities
        self.pub.publish(msg)
        return (angles, velocities)

    def radialSteer(self, left_joystickY, right_joystickX):
        angle = (right_joystickX * 90)
        angles = [90 - angle, 90 + angle, 90 - angle, 90 + angle]

        vel = left_joystickY
        velocities = [vel, vel, vel, vel]
        H = Header()
        H.stamp = rospy.Time.now()
        msg = SteerAndThrottle()
        msg.header = H
        msg.angles = angles
        msg.throttles = velocities
        self.pub.publish(msg)
        return (angles, velocities)
        
    def joyCallback(self, joy):
        if not rospy.get_param("/isAutonomous"):
            #rospy.logwarn("mode " + str(self.steeringType))
            self.currentStartButtonState = joy.buttons[9]
            if self.currentStartButtonState and not self.previousStartButtonState :
                self.steeringType=(self.steeringType+1)%3
                rospy.logwarn("switched to drive mode " + str(self.steeringType))
            if self.steeringType==0:
            # error source
                self.tankSteer(joy.axes[1],joy.axes[5])
            if self.steeringType==1:
                self.radialSteer(joy.axes[1],joy.axes[2])
            if self.steeringType==2:
                self.translationControl(joy.axes[1],joy.axes[2])
            self.previousStartButtonState = self.currentStartButtonState

    def autonomyCallback(self, twist):
        if rospy.get_param("/isAutonomous"):
            #print("autonomy twist recieved")

            linear_x = twist.linear.x
            angular_z = twist.angular.z

            left_speed = linear_x + angular_z
            right_speed = linear_x - angular_z

            self.tankSteer(left_speed, right_speed)
    
    def bypassCallback(self, bypass):
        if rospy.get_param("/isAutonomous"):
            self.tankSteer(bypass.left_joy, bypass.right_joy)

if __name__ == "__main__":
    pub = rospy.Publisher('locomotion', SteerAndThrottle, queue_size=1)
    locController = LocCtlr(pub)
    rospy.init_node('locomotion')
    rospy.Subscriber("telemetry_joy", Joy, locController.joyCallback, queue_size=1)
    rospy.Subscriber("cmd_vel", Twist, locController.autonomyCallback, queue_size=1)
    rospy.Subscriber("bypass_vel", Twist, locController.bypassCallback, queue_size=1)
    rospy.spin()
	#i = 0
	#while True:
		#i += 0.5
		#print(controller.tankSteer(i,5))
