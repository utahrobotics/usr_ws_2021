import rclpy
from rclpy.node import Node
import yaml

from motion_controller_msgs.msg import Mobility

import Jetson.GPIO as GPIO

class DrivingSubscriber(Node):
    #T his clas is responsible for driving all of the Maxon motor controllers using published information from the 
    # Mobility node
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Mobility,
            'drive_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        #create controller instances for each for each of the motor bases from the config file
        tmp_file = open('/home/usr/usr_ws_2020/src/motor_controllers/maxon_motor_node/config/motor_config.yaml')
        motor_config = yaml.load(tmp_file, Loader=yaml.FullLoader)
        self.controllers = {'front_left': None, 'front_right': None, 'back_left': None, 'back_right': None}

        self.controllers['front_left'] = MaxonController(speed_pin=motor_config['front_left']['pins']['speed'], 
                                                     dir_pin=motor_config['front_left']['pins']['direction'],
                                                     ready_pin=motor_config['front_left']['pins']['ready'],
                                                     enable_pin=motor_config['front_left']['pins']['enable'],
                                                     inverted=motor_config['front_left']['inverted'])

        self.controllers['front_right'] = MaxonController(speed_pin=motor_config['front_right']['pins']['speed'], 
                                                     dir_pin=motor_config['front_right']['pins']['direction'],
                                                     ready_pin=motor_config['front_right']['pins']['ready'],
                                                     enable_pin=motor_config['front_right']['pins']['enable'],
                                                     inverted=motor_config['front_right']['inverted'])

        self.controllers['back_left'] = MaxonController(speed_pin=motor_config['back_left']['pins']['speed'], 
                                                     dir_pin=motor_config['back_left']['pins']['direction'],
                                                     ready_pin=motor_config['back_left']['pins']['ready'],
                                                     enable_pin=motor_config['back_left']['pins']['enable'],
                                                     inverted=motor_config['back_left']['inverted'])

        self.controllers['back_right'] = MaxonController(speed_pin=motor_config['back_right']['pins']['speed'], 
                                                    dir_pin=motor_config['back_right']['pins']['direction'],
                                                     ready_pin=motor_config['back_right']['pins']['ready'],
                                                     enable_pin=motor_config['back_right']['pins']['enable'],
                                                     inverted=motor_config['back_right']['inverted'])

        tmp_file.close()

    def listener_callback(self, msg):
        # first check that the controllers are ready
        #TODO: incorperate the state machince variables to decide if motors should be running or not
        if not (self.controllers['front_left'].ready()  & self.controllers['front_right'].ready() & self.controllers['back_left'].ready() & self.controllers['back_left'].ready()):
            # at least one of the motors is not ready so log it and don't move
            #TODO: change the functionality for be more redundants and specific for the fault manager system
            self.get_logger().info('one or motors are not ready, so no motor movment is being done')
        
        # if motors are ready, set the new speed to each controller
        self.controllers['front_left'].set_speed(msg.front_left)
        self.controllers['front_right'].set_speed(msg.front_right)
        self.controllers['back_left'].set_speed(msg.rear_left)
        self.controllers['back_right'].set_speed(msg.rear_right)


class MaxonController():
    # This class holds all of the information relavent for controlling a Maxon
    # Maxon 380200 speed DEC controller.

    def __init__(self, speed_pin, dir_pin, ready_pin, enable_pin, inverted=False):
        # all pins are in alignment with the Jetson Xavier NX GPIO pinout, so we sepcify the mode accordingly
        GPIO.setmode(GPIO.BOARD)

        # setup the speed pin, this is controlled using PWM signals
        self.speed_pin = speed_pin
        GPIO.setup(channels=speed_pin, direction=GPIO.OUT, initial=GPIO.LOW)
        self.pwm_sig = GPIO.PWM(self.speed_pin, 10000) #runs at 10KHz as each cycle in 100us long
        self.pwm_sig.start(0)
        #TODO: grab the pwm frequency from a configuration file
        
        #setup the direction pin, 1 for forward 0 for backwards
        self.dir_pin = dir_pin
        GPIO.setup(channels=dir_pin, direction=GPIO.OUT)

        # setup the ready pin, this signals if the controller is ready to operate
        self.ready_pin = ready_pin
        GPIO.setup(channels=ready_pin, direction=GPIO.IN)

        # setup the enable pin, this signlas if the motor is operational, set 1 for enabled 0 for motor diable
        self.enable_pin = enable_pin
        GPIO.setup(channels=enable_pin, direction=GPIO.OUT, initial=GPIO.HIGH)

        # inverted tell us if the controller direction should be inverted or not, mostly used as a quick configuration change after motor installation
        self.inverted = inverted

    def __del__(self):
        self.pwm_sig.stop()
        GPIO.output(self.enable_pin, GPIO.LOW)
        print('disabling controllers')
        GPIO.cleanup([self.speed_pin, self.dir_pin, self.ready_pin, self.enable_pin])

    def set_speed(self, speed):
        # Sets the speed of the motor, the values for the speed 0-1 with 1 being the maximum velocity,

        # first check the dierection of the velocity, + signifies forward, - signifies backwards
        if speed < 0:
            # velocity is negative so set the direction to be reverse
            level = GPIO.LOW
            if self.inverted:
                level = GPIO.HIGH
            GPIO.output(self.dir_pin, level)
        else:
            # velocity is positive so set the motor into forward operation
            level = GPIO.HIGH
            if self.inverted:
                level = GPIO.LOW
            GPIO.output(self.dir_pin, level)

        # calcualte the duty cycle ranging from 0 - 100 for 0%-100%
        duty_cycle = 100 * round(speed, 2)
        print(duty_cycle)
        self.pwm_sig.ChangeDutyCycle(duty_cycle)
        return

    def ready(self):
        #reads the ready state of the controller returns Ture if controller is ready False otherwise
        if GPIO.input(self.ready_pin) == GPIO.HIGH:
            return True
        return False

    def enable(self):
        # Enables the controller
        try:
            GPIO.output(self.enable_pin, GPIO.HIGH)
            return True
        except:
            return False
    
    def disable(self):
        # Disables the Contrller
        try:
            GPIO.output(self.enable_pin, GPIO.LOW)
            return True
        except:
            return False

def main(args=None):
    rclpy.init(args=args)

    # inittialize the main drving node
    sub_node = DrivingSubscriber()

    rclpy.spin(sub_node)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
