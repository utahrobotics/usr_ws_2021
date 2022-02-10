#!/usr/bin/env python
# defines a LocomotionController object which
# interfaces with user input to control the robot.

from tokenize import Double
import vector
import math
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from locomotion.msg import SteerAndThrottle

class LocCtlr:
# take range from -1 to 1 and traslate that to the angle we need to move the wheels.
    scale=0
    throttle=0.0
    frontAngle=0.0
    backAngle = 0.0
    rightMotion =0.0
    leftMotion= 0.0
    wheetDist = 1.0
    def __init__(self,Scale):
	self.pub = rospy.Publisher('locomotion', SteerAndThrottle, queue_size=10)
	rospy.init_node('locomotion')
        self.scale=Scale
        self.angle=0.0
        self.rightInput=0.0

    def tankSteer(self,_leftJoystick, _rightJoystick):
        self.leftMotion = _leftJoystick * self.scale
        self.rightMotion = _rightJoystick * self.scale
        angles = [0, 0, 0, 0]
        velocities = [self.leftMotion, self.leftMotion, self.rightMotion, self.rightMotion]
        H = Header()
	H.stamp = rospy.Time.now()
	msg = SteerAndThrottle()
	msg.header = H
	msg.angles = angles
	msg.throttles = velocities
	self.pub.publish(msg)
        return (angles, velocities)

    def translationControl(self, _leftJoystick, _rightTrigger):
        self.wheelCenter = [0, 0.5]
        self.steerIntensity = [(_leftJoystick * 2)**3, 0]
        angleVector = np.subtract(self.wheelCenter, self.steerIntensity)
        angleVectorUnit = angleVector / np.linalg.norm(angleVector)
        axisX = [1, 0]
        dot = np.dot(angleVectorUnit, axisX)
        angle = np.arccos(dot) * 180 / math.pi
        angles = [angle, angle, angle, angle]
        velocities = [_rightTrigger, _rightTrigger, _rightTrigger, _rightTrigger]
	H = Header()
	H.stamp = rospy.Time.now()
	msg = SteerAndThrottle()
	msg.header = H
	msg.angles = angles
	msg.throttles = velocities
	self.pub.publish(msg)
        return (angles, velocities)

    def radialSteer(self, _leftJoystick, _rightTrigger):
        self.wheelCenter = [0, 0.5]
        self.steerIntensity = [(_leftJoystick * 2)**3, 0]
        angleVector = np.subtract(self.wheelCenter, self.steerIntensity)
        angleVectorUnit = angleVector / np.linalg.norm(angleVector)
        axisX = [1, 0]
        dot = np.dot(angleVectorUnit, axisX)
        angle = np.arccos(dot) * 180 / math.pi
        angles = [angle, -angle, angle, -angle]
        velocities = [_rightTrigger, _rightTrigger, _rightTrigger, _rightTrigger]
	H = Header()
	H.stamp = rospy.Time.now()
	msg = SteerAndThrottle()
	msg.header = H
	msg.angles = angles
	msg.throttles = velocities
	self.pub.publish(msg)
        return (angles, velocities)

if __name__ == "__main__":
   	controller = LocCtlr(1)
	i = 0
	while True:
		i += 0.5
		print(controller.tankSteer(i,5))
