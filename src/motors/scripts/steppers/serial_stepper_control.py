#!/usr/bin/env python
"""
This node is the communication layer betweeen the USR Ros subsystem and the stepper motor controllers.
"""
# TODO: add recieving info from the stepper controller

#import rclpy
#from rclpy.node import Node
import rospy
import yaml
import serial
import time
from enum import Enum
import os
import struct
import rospkg
import threading

import actionlib
from motors.msg import HomeMotorManualAction, HomeMotorManualFeedback, HomeMotorManualResult

from locomotion.msg import SteerAndThrottle
from sensor_msgs.msg import Joy


class Command(Enum):
	# command for the stepper controller
	init_all = 1
	align_all = 2
	align_one = 3
	stop_all = 4
	stop_one = 5
	blink_led = 6
	home_port = 7
	cancel = 8
	start_manual_home = 9
	stop_manual_home = 10

class SteeringSubscriber():
	# This class is responsible for driving all of the Maxon motor controllers using published information from the
	# Mobility node
	def __init__(self):
		rospy.init_node('stepper_control_node')
		
		# get an instance of RosPack with the default search paths
		rospack = rospkg.RosPack()

		# get the file path for rospy_tutorials
		rospack.get_path('motors')

		tmp_file = open(os.path.join(rospack.get_path('motors'), 'scripts', 'steppers', 'config', 'stepper_config.yaml'))
		stepper_config = yaml.safe_load(tmp_file)

		# create controller instances for each for each of the motor bases from the config file
		self.stepper_controller = StepperController(stepper_config['serial'], stepper_config['steps'])

		#self.stepper_controller.initMotors()

		rospy.Subscriber(
			'locomotion',
			SteerAndThrottle,
			self.listener_callback, queue_size=1)

		rospy.Subscriber("telemetry_joy", Joy, self.joyCallback, queue_size=10)

		# intilialize motors
		

		# close the yaml configuration file
		tmp_file.close()

		self.a_server = actionlib.SimpleActionServer("home_motor_manual_as", WashTheDishesAction, execute_cb=self.start_manual_home_cb, auto_start=False)
		self.a_server.start()
		rospy.on_shutdown(self.shutdown)

	def listener_callback(self, msg):
		# first check that the controllers are ready
		# TODO: incorperate the state machince variables to decide if motors should be running or not

		# if motors are ready, set the new speed to each controller
		if self.stepper_controller != None:
			self.stepper_controller.alignMotors(
							int(msg.angles[0]),
							int(msg.angles[1]),
							int(msg.angles[2]),
							int(msg.angles[3])
							)
	def joyCallback(self, joy):
		if joy.axes[1]: #stop manual home if y is pressed
			self.stepper_controller.StopManualHome()
			result = HomeMotorManualResult()
			result.success = True;
			self.a_server.set_succeeded(result)

	def shutdown(self):
		self.stepper_controller.cancel()

	def start_manual_home_cb(self, motorToHome):
		feedback = HomeMotorManualFeedback()
		rate = rospy.Rate(10)

		self.stepper_controller.StartManualHome(motorToHome)

		while(true):
			if self.a_server.is_preempt_requested():
				self.a_server.set_preempted()
				break
			feedback.isRunning = True
			self.a_server.publish_feedback(feedback)
			rate.sleep()


class StepperController():
	"""
		This class holds the imformation relevant for controller a stepper motor contorller onboard the teensy device
	"""

	def __init__(self, serial_number, steps):
		self.serial = serial_number  # the serial number for responding to the device
		self.steps = steps
		# teh micro controller serial instance
		self._mc = serial.Serial(serial_number, 115200, timeout=.1)
		self.buff = ""
		time.sleep(1)  # give the connection a second to settle

		read_thread = threading.Thread(target=self.readSerial)
		read_thread.setDaemon(True) 
		read_thread.start()

	def cancel(self):
		self._mc.write(self._encodeCancel())

	def alignMotors(self, fl, fr, bl, br):
		"""
			Send comm to the motor cointroller to align the front left (fl), front right (fr), back left (bl), and back right (br) motors
			Inputs:
				fl -> the degrees to align the front left motor
				fr -> the degrees to align the front right motor
				bl -> the degrees to align the back left motor
				br -> the degrees to align the back right motor
			Reutrn:
				None
		"""
		# convert from degrees to steps (TODO: verify the right direction and whatnot)

		# write the command to the stepper controller
		self._mc.write(self._encodeAlignCommand(int(fl),int(fr),int(bl),int(br)))

	def initMotors(self):
		self._mc.write(self._encodeInit())

	def StartManualHome(self, port):
		self._mc.write(self._encodeManualHome(port))

	def StopManualHome(self):
		self._mc.write(self._encodeStopManualHome())

	def blink(self, num_blinks):
		self._mc.write(self._encodeBlink(num_blinks=num_blinks))

	def readSerial(self):
		while True:
			data = self._mc.read()
			if data:
				self.buff += data.decode()
				if data == '\n' or data == '\0' or data=='':
					rospy.logwarn(self.buff)

	def _encodeAlignCommand(self, fl, fr, bl, br):
		# cmd = motor<<6 | dir<<5 | steps;
		# return cmd
		fl1, fl2, fl3, fl4 = self.int_to_four_bytes(fl & 0xFFFFFFFF)
		fr1, fr2, fr3, fr4 = self.int_to_four_bytes(fr & 0xFFFFFFFF)
		bl1, bl2, bl3, bl4 = self.int_to_four_bytes(bl & 0xFFFFFFFF)
		br1, br2, br3, br4 = self.int_to_four_bytes(br & 0xFFFFFFFF)
		return bytearray([Command.align_all.value, int(fl1), int(fl2), int(fl3), int(fl4), int(fr1), int(fr2), int(fr3), int(fr4), int(bl1), int(bl2), int(bl3), int(bl4), int(br1), int(br2), int(br3), int(br4)])

	def _encodeBlink(self, num_blinks):
		return bytearray([Command.blink_led.value, num_blinks])

	def _encodeHome(port):
		return bytearray([Command.home_port.value, port])

	def encodeManualHome(port):
		return bytearray([Command.start_manual_home.value, port & 0xFF])

	def encodeStopManualHome():
		return bytearray([Command.stop_manual_home.value])

	def _encodeInit(self):
		return bytearray([Command.init_all.value])

	def _encodeCancel(self):
		return bytearray([Command.cancel.value])

	def _deg2steps(self, deg):
		"""
			convert degrees to the stepper motor steps
			Inputs:
				deg -> the requested degrees
			Return:
				steps -> the resultand steps
		"""
		return int(round((deg / 360) * self.steps))

	def int_to_four_bytes(self, x):
		return [int(x >> i & 0xff) for i in (24,16,8,0)]


def main(args=None):
	# inittialize the main drving node
	sub_node = SteeringSubscriber()

	rospy.spin()


if __name__ == '__main__':
	main()

