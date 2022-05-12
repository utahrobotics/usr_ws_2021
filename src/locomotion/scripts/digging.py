#!/usr/bin/env python
# defines a LocomotionController object which
#!/usr/bin/env python
# interfaces with user input to control the robot.

from tokenize import Double
import vector
import math
import numpy as np
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
import actionlib
from locomotion.msg import SetSpeedAction, SetSpeedFeedback, SetSpeedResult


class DigCtlr:
# take range from -1 to 1 and traslate that to the angle we need to move the wheels.
    def __init__(self):
        self.armPub = rospy.Publisher('arm_vel', Float32, queue_size=1)
        self.drumPub = rospy.Publisher('drum_vel', Float32, queue_size=1)

        self.setDrumSpeedServer = actionlib.SimpleActionServer(
            "set_drum_speed_as", SetSpeedAction, execute_cb=self.setDrumSpeed_cb, auto_start=False)
        self.setDrumSpeedServer.start()
        self.setArmSpeedServer = actionlib.SimpleActionServer(
            "set_arm_speed_as", SetSpeedAction, execute_cb=self.setArmSpeed_cb, auto_start=False)
        self.setArmSpeedServer.start()

        rospy.Subscriber("telemetry_joy", Joy, self.joyCallback, queue_size=1)

    def setDrumSpeed_cb(self, goal):
        self.drumPub.publish(Float32(goal.speed))
        result = SetSpeedResult()
        result.speed = goal.speed
        self.a_server.set_succeeded(goal.speed)

    def setArmSpeed_cb(self, goal):
        self.armPub.publish(Float32(goal.speed))
        result = SetSpeedResult()
        result.speed = goal.speed
        self.a_server.set_succeeded(goal.speed)

    def Dig(self,_leftTrigger, _rightTrigger, _leftBumper, _rightBumper):
        armVel = 0
        drumVel = (_leftTrigger - _rightTrigger)/2
        if(_leftBumper ^ _rightBumper):
            armVel = 1 if _rightBumper else -1
            self.armPub.publish(Float32(armVel))
            self.drumPub.publish(Float32(drumVel))

        return (armVel, drumVel)
        
    def joyCallback(self, joy):
        if not rospy.get_param("/isAutonomous"):
            self.Dig(joy.axes[3],joy.axes[4], joy.buttons[4], joy.buttons[5])

#    def autonomyCallback(self,twist):
#        global locController
#        if rospy.get_param("/isAutonomous"):
#            print("autonomy twist recieved")
#
#            linear_x = twist.linear.x
#            angular_z = twist.angular.z
#
#            left_speed = linear_x + angular_z
#            right_speed = linear_x - angular_z
#
#            locController.tankSteer(left_speed, right_speed)

if __name__ == "__main__":
    rospy.init_node('locomotion')
    locController = DigCtlr()
    rospy.spin()
	#i = 0
	#while True:
		#i += 0.5
		#print(controller.tankSteer(i,5))