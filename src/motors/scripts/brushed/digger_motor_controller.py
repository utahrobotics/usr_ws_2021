import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
import os
print(os.getcwd())

help('modules')

from pyvesc import VESC

from angle_sense import AngleSensor

# serial port that VESC is connected to. Something like "COM3" for windows and as below for linux/mac
serial_port1 = '/dev/drum_vesc'
serial_port2 = '/dev/arm_vesc'

class DrivingSubscriber(Node):
    #This class is responsible for driving all of the Maxon motor controllers using published information from the 
    # Mobility node
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.drum_subscriber_ = self.create_subscription(Float32, 'drum_vel', self.drum_drive_callback,10)
        self.drum_subscriber_

        self.arm_subscriber_ = self.create_subscription(Float32, 'arm_vel', self.arm_callback,10)
        self.arm_subscriber_

        self.drum_motor = VESC(serial_port=serial_port1)
        self.arm_motor = VESC(serial_port=serial_port2)

        self.sensor = AngleSensor()
        self.sensor.start_adc(0)

    def __del__(self):
         with VESC(serial_port=serial_port1) as drum_motor:
             self.drum_motor.set_duty_cycle(0)
         with VESC(serial_port=serial_port2) as arm_motor:
             self.arm_motor.set_duty_cycle(0)

    def drum_drive_callback(self, msg):
         #with VESC(serial_port=serial_port1) as drum_motor:
             drum_vel = msg.data
             if drum_vel > 1:
                drum_vel = 1
             elif drum_vel < -1:
                drum_vel = -1
             print(drum_vel)
             self.drum_motor.set_duty_cycle(drum_vel)

    def arm_callback(self, msg):
        #with VESC(serial_port=serial_port2) as arm_motor:
             arm_vel = msg.data
             if arm_vel > 1:
                 if(self.getAngle > 175):
                     return
                 arm_vel = 1
             elif arm_vel < -1:
                 if(self.getAngle < 60):
                     return
                 arm_vel = -1

             self.arm_motor.set_duty_cycle(arm_vel)

    def getAngle():
        return self.sensor.computeDegrees(5, self.sensor.computeVolts(self.sensor.get_last_result()))


def main(args=None):
    rclpy.init(args=args)

    # inittialize the main drving node
    sub_node = DrivingSubscriber()

    rclpy.spin(sub_node)

    # Destroy the node explicitly
    sub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
