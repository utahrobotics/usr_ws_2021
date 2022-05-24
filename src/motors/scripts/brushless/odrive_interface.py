#!/usr/bin/env python3

from __future__ import print_function

import rospy
import odrive
from odrive.enums import *
import time
import math
import signal

from locomotion.msg import SteerAndThrottle

def listener_callback(msg):
        my_drive.axis0.config.sensorless_ramp.vel = (msg.throttles[2]*1000)/60 * 2 * math.pi * 7
        my_drive.axis1.config.sensorless_ramp.vel = (msg.throttles[3]*1000)/60 * 2 * math.pi * 7
        #my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        exit(1)


if __name__ == "__main__":
    rospy.init_node('odrv_interface')

    # Find a connected ODrive (this will block until you connect one)
    print("finding an odrive...")
    my_drive = odrive.find_any()

    print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")
    print("Calibration current is " + str(my_drive.axis0.motor.config.calibration_current) + "A")

    # Set some hardware parameters temporarily
    print("setting some hardware parameters temporarily")
    my_drive.config.brake_resistance = 0.5
    my_drive.axis0.motor.config.pole_pairs = 8
    my_drive.axis1.motor.config.pole_pairs = 8

    # Calibrate
    print("starting calibration")
    my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    my_drive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    print("waiting for calibration to end...")
    while my_drive.axis0.current_state != AXIS_STATE_IDLE or my_drive.axis1.current_state != AXIS_STATE_IDLE :
        time.sleep(0.1)

    # Closed loop control 
    print("Changing state to closed loop control")
    #my_drive.axis0.config.sensorless_ramp.vel = 380.95/60 * 2 * math.pi * 7
    #my_drive.axis0.config.sensorless_ramp.vel = 300/60 * 2 * math.pi * 7
    my_drive.axis0.config.sensorless_ramp.vel = 0
    my_drive.axis1.config.sensorless_ramp.vel = 0
    my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    while my_drive.axis0.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        print("axis errors are:")
        print(hex(my_drive.axis0.error))	
        print("motor errors are:")
        print(hex(my_drive.axis0.motor.error))
        print("encoder errors are:")
        print(hex(my_drive.axis0.encoder.error))
        time.sleep(0.1)

    print("current state is " + str(my_drive.axis0.current_state))

    subscription = rospy.Subscriber('locomotion', SteerAndThrottle, listener_callback, queue_size=1)

    rospy.spin()