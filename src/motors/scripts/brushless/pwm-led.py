#  MBTechWorks.com 2016
#  Pulse Width Modulation (PWM) demo to cycle brightness of an LED

import Jetson.GPIO as GPIO   # Import the GPIO library.
import time								# Import time library

GPIO.setmode(GPIO.BOARD)  # Set Pi to use pin number when referencing GPIO pins.
                          # Can use GPIO.setmode(GPIO.BCM) instead to use 
                          # Broadcom SOC channel names.

GPIO.setup(33, GPIO.OUT)  # Set GPIO pin 12 to output mode.
pwm = GPIO.PWM(33, 100)   # Initialize PWM on pwmPin 100Hz frequency


# main loop of program
print("\nPress Ctl C to quit \n")      # Print blank line (\n == newline) before and after message.
dc=0                                   # set dc variable to 0 (will start PWM at 0% duty cycle)
pwm.start(dc)                          # Start PWM with 0% duty cycle
while True:                            # Create an infinite loop until Ctl C is pressed to stop program.
  for dc in range(0, 101, 5):          # Loop with dc set from 0 to 100 stepping dc up by 5 each loop
    pwm.ChangeDutyCycle(dc)
    time.sleep(0.05)                   # wait for .05 seconds at current LED brightness level
    print(dc)
  for dc in range(95, 0, -5):          # Loop with dc set from 95 to 5 stepping dc down by 5 each loop
    pwm.ChangeDutyCycle(dc)
    time.sleep(0.05)                   # wait for .05 seconds at current LED brightness level
    print(dc)
 
pwm.stop()
GPIO.cleanup()                         # resets GPIO ports used in this program back to input mode
