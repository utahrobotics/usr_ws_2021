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

from locomotion.msg import SteerAndThrottle

int_to_four_bytes = struct.Struct('<I').pack


class Command(Enum):
    # command for the stepper controller
    init_all = 1
    align_all = 2
    align_one = 3
    stop_all = 4
    stop_one = 5
    blink_led = 6
    home_port = 7


class SteeringSubscriber():
    # This class is responsible for driving all of the Maxon motor controllers using published information from the
    # Mobility node
    def __init__(self):
        rospy.init_node('stepper_control_node')
        rospy.Subscriber(
            'locomotion',
            SteerAndThrottle,
            self.listener_callback)

        # create controller instances for each for each of the motor bases from the config file
        rospy.logwarn(os.getcwd())
        tmp_file = open(
            './config/stepper_config.yaml')
        stepper_config = yaml.safe_load(tmp_file)

        self.stepper_controller = StepperController(stepper_config['serial'],
                                                    stepper_config['steps'])

        # intilialize motors
        self.stepper_controller.initMotors()

        # close the yaml configuration file
        tmp_file.close()

    def listener_callback(self, msg):
        # first check that the controllers are ready
        # TODO: incorperate the state machince variables to decide if motors should be running or not

        # if motors are ready, set the new speed to each controller
        self.stepper_controller.alignMotors(msg.angles[0],
                                            msg.angles[1],
                                            msg.angles[2],
                                            msg.angles[3]
                                            )


class StepperController():
    """
        This class holds the imformation relevant for controller a stepper motor contorller onboard the teensy device
    """

    def __init__(self, serial_number, steps):
        self.serial = serial_number  # the serial number for responding to the device
        # teh micro controller serial instance
        self._mc = serial.Serial(serial_number, 115200, timeout=.1)
        time.sleep(1)  # give the connection a second to settle

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
        self._mc.write(self._encodeAlignCommand(self._deg2steps(fl),
                                                self._deg2steps(fr),
                                                self._deg2steps(bl),
                                                self._deg2steps(br)))

    def initMotors(self):
        self._mc.write(self._encodeInit())

    def blink(self, num_blinks):
        self._mc.write(self._encodeBlink(num_blinks=num_blinks))

    def _encodeAlignCommand(self, fl, fr, bl, br):
        # cmd = motor<<6 | dir<<5 | steps;
        # return cmd
        fl1, fl2, fl3, fl4 = int_to_four_bytes(fl & 0xFFFFFFFF)
        fr1, fr2, fr3, fr4 = int_to_four_bytes(fr & 0xFFFFFFFF)
        bl1, bl2, bl3, bl4 = int_to_four_bytes(bl & 0xFFFFFFFF)
        br1, br2, br3, br4 = int_to_four_bytes(br & 0xFFFFFFFF)
        return bytearray([Command.align_all.value, int(fl4), int(fl3), int(fl2), int(fl1), int(fr4), int(fr3), int(fr2), int(fr1), int(bl4), int(bl3), int(bl2), int(bl1), int(br4), int(br3), int(br2), int(br1)])

    def _encodeBlink(self, num_blinks):
        return bytearray([Command.blink_led.value, num_blinks])

    def _encodeHome(port):
        return bytearray([Command.home_port.value, port])

    def _encodeInit(self):
        return bytearray([Command.init_all.value])

    def _deg2steps(self, deg):
        """
            convert degrees to the stepper motor steps
            Inputs:
                deg -> the requested degrees
            Return:
                steps -> the resultand steps
        """
        return round((deg / 360) * self.steps)


def main(args=None):
    # inittialize the main drving node
    sub_node = SteeringSubscriber()

    rospy.spin()

    # Destroy the node explicitly
    sub_node.destroy_node()
    rospy.shutdown()


if __name__ == '__main__':
    main()
