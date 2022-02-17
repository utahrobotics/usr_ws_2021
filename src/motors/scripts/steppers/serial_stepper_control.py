"""
This node is the communication layer betweeen the USR Ros subsystem and the stepper motor controllers.
"""
# TODO: add recieving info from the stepper controller

#import rclpy
#from rclpy.node import Node
import rospy
from rospy import Node
import yaml
import serial, time
from enum import Enum

from motion_controller_msgs.msg import Mobility


class Command(Enum):
    # command for the stepper controller
    init_all = 1
    align_all = 2
    align_one = 3
    stop_all = 4
    stop_one = 5
    blink_led = 6


class SteeringSubscriber():
    # T his clas is responsible for driving all of the Maxon motor controllers using published information from the
    # Mobility node
    def __init__(self):
        rospy.init_node('stepper_control_node')
        rospy.Subscriber(
            'steering',
            Mobility,
            self.listener_callback)

        # create controller instances for each for each of the motor bases from the config file
        tmp_file = open('./config/stepper_config.yaml')
        stepper_config = yaml.load(tmp_file, Loader=yaml.FullLoader)

        self.stepper_controller = StepperController(serial=stepper_config['serial'],
                                                    steps=stepper_config['steps'])

        # intilialize motors and blink for conformation
        self.stepper_controller.initMotors()
        time.sleep(.5)
        self.stepper_controller.blink(3)

        # close the yaml configuration file
        tmp_file.close()

    def listener_callback(self, msg):
        # first check that the controllers are ready
        # TODO: incorperate the state machince variables to decide if motors should be running or not

        # if motors are ready, set the new speed to each controller
        self.stepper_controller.alignMotors(msg.front_left,
                                            msg.front_right,
                                            msg.back_left,
                                            msg.back_right)


class StepperController():
    """
        This class holds the imformation relevant for controller a stepper motor contorller onboard the teensy device
    """

    def __init__(self, serial, steps) -> None:
        self.serial = serial  # the serial number for responding to the device
        self._mc = serial.Serial(serial, 115200, timeout=.1)  # teh micro controller serial instance
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
        self._mc.write(self._encodeInit)

    def blink(self, num_blinks):
        self._mc.write(self._encodeBlink(num_blinks=num_blinks))

    def _encodeAlignCommand(self, fl, fr, bl, br):
        # cmd = motor<<6 | dir<<5 | steps;
        # return cmd
        return bytearray([Command.align_all.value, int(fl), int(fr), int(bl), int(br)])

    def _encodeBlink(self, num_blinks):
        return bytearray([Command.blink_led.value, num_blinks])

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


