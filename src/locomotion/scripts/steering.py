#!/usr/bin/env python
# defines a LocomotionController object which
# interfaces with user input to control the robot.

from tokenize import Double
import vector
import math
import numpy as np
import rospy
from std_msgs.msg import Header
from std_msgs.msg import Twist
from sensor_msgs.msg import Joy
from locomotion.msg import SteerAndThrottle

class LocCtlr:
    wheelDist = 1.0
    
    def __init__(self, _pub):
        self.pub = _pub
        
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
        self.wheelCenter = [0, self.wheelDist/2]
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
        
    def joyCallback(joy):
        if not rospy.get_param("/isAutonomous"):
            print("telemetry recieved")

    def autonomyCallback(twist):
        global locController
        if rospy.get_param("/isAutonomous"):
            print("autonomy twist recieved")

            linear_x = twist.linear.x
            angular_z = twist.angular.z

            left_speed = linear_x + angular_z
            right_speed = linear_x - angular_z

            locController.tankSteer(left_speed, right_speed)

if __name__ == "__main__":
    pub = rospy.Publisher('locomotion', SteerAndThrottle, queue_size=10)
    locController = LocCtlr(pub)
    rospy.init_node('locomotion')
    rospy.Subscriber("telemetry_joy", Joy, locController.joyCallback)
    rospy.Subscriber("autonomy/twist", Twist, locController.autonomyCallback)
    rospy.spin()
	#i = 0
	#while True:
		#i += 0.5
		#print(controller.tankSteer(i,5))