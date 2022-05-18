#!/usr/bin/env python

import Jetson.GPIO as GPIO
import time

enable_pin = 7
GPIO.setmode(GPIO.BOARD)
GPIO.setup(channels=enable_pin, direction=GPIO.OUT, initial=GPIO.LOW)
GPIO.output(enable_pin, GPIO.LOW)
time.sleep(1)
GPIO.output(enable_pin, GPIO.HIGH)
time.sleep(1)
GPIO.output(enable_pin, GPIO.LOW)
time.sleep(1)
GPIO.output(enable_pin, GPIO.HIGH)
time.sleep(1)
GPIO.output(enable_pin, GPIO.LOW)
time.sleep(1)
GPIO.output(enable_pin, GPIO.HIGH)
time.sleep(1)
GPIO.output(enable_pin, GPIO.LOW)
time.sleep(1)
GPIO.output(enable_pin, GPIO.HIGH)
time.sleep(1)
GPIO.output(enable_pin, GPIO.LOW)
time.sleep(1)
