#!/usr/bin/env python

import Jetson.GPIO as GPIO

enable_pin = 7
GPIO.setmode(GPIO.BOARD)
GPIO.setup(channels=enable_pin, direction=GPIO.OUT, initial=GPIO.LOW)
GPIO.output(enable_pin, GPIO.LOW)
