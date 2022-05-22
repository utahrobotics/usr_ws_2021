#!/usr/bin/env python
# defines a LocomotionController object which
# !/usr/bin/env python
# interfaces with user input to control the robot.

from tokenize import Double
import vector
import math
import numpy as np
import rospy
import actionlib
from locomotion.msg import SetAngleAction, SetAngleFeedback, SetAngleResult
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy

servo_min = 100  # Min pulse length out of 4096
servo_max = 650  # Max pulse length out of 4096

angle_min = -25
angle_max = 270


class CamCtlr:
# take range from -1 to 1 and traslate that to the angle we need to move the wheels.
	def __init__(self):
		self.CamPulse = 595  # 150-600
		self.ArmPulse = servo_min  # 150-600
		self.LastJoy = None

		self.camAnglePub = rospy.Publisher('cam_angle', Int32, queue_size=1)
		self.sensorArmAnglePub = rospy.Publisher('sensor_arm_angle', Int32, queue_size=1)

		rospy.Subscriber("telemetry_joy", Joy, self.joyCallback, queue_size=1)

		self.setCamAngleServer = actionlib.SimpleActionServer(
            "set_cam_angle_as", SetAngleAction, execute_cb=self.setCamAngle_cb, auto_start=False)
		self.setCamAngleServer.start()
		self.setArmAngleServer = actionlib.SimpleActionServer(
            "set_arm_angle_as", SetAngleAction, execute_cb=self.setArmAngle_cb, auto_start=False)
		self.setArmAngleServer.start()

		r = rospy.Rate(100)
		while not rospy.is_shutdown():
			if not rospy.get_param("/isAutonomous") and not self.LastJoy == None:
				self.CamControl(self.LastJoy.axes[6], self.LastJoy.axes[7])
			r.sleep()

	def setCamAngle_cb(self, goal):
		ppm = servo_max - ((goal.angle-angle_min)/(angle_max-angle_min))*(servo_max-servo_min)
		self.CamPulse = ppm
		self.camAnglePub.publish(Int32(self.CamPulse))

	def setArmAngle_cb(self, goal):
		ppm = servo_max - ((goal.angle-angle_min)/(angle_max-angle_min))*(servo_max-servo_min)
		self.ArmPulse = ppm
		self.sensorArmAnglePub.publish(Int32(self.ArmPulse))
			
	def CamControl(self, _dPadX, _dPadY):
		if _dPadX < 0 and self.CamPulse > servo_min:
			self.CamPulse -= 1
		elif _dPadX>0 and self.CamPulse < servo_max:
			self.CamPulse += 1
	
		if _dPadY<0 and self.ArmPulse > servo_min:
			self.ArmPulse -= 1
		elif _dPadY>0  and self.ArmPulse < servo_max:
			self.ArmPulse += 1

		self.camAnglePub.publish(Int32(self.CamPulse))
		self.sensorArmAnglePub.publish(Int32(self.ArmPulse))

		return (self.CamPulse, self.ArmPulse)
			
	def joyCallback(self, joy):
			# rospy.logwarn(joy)
			self.LastJoy = joy;

if __name__ == "__main__":
    rospy.init_node('cameras')
    camController = CamCtlr()
