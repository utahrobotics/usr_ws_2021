#!/usr/bin/env python
# defines a LocomotionController object which
#!/usr/bin/env python
# interfaces with user input to control the robot.

from tokenize import Double
import vector
import math
import numpy as np
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy


class DigCtlr:
# take range from -1 to 1 and traslate that to the angle we need to move the wheels.
    def __init__(self):
	self.CamPulse = 375 #150-600
	self.ArmPulse = 150 #150-600
	self.LastJoy = None
        self.camAnglePub = rospy.Publisher('cam_angle', Int32, queue_size=1)
	self.sensorArmAnglePub = rospy.Publisher('sensor_arm_angle', Int32, queue_size=1)
	rospy.Subscriber("telemetry_joy", Joy, self.joyCallback, queue_size=1)
	rospy.rate(100)
	while not rospy.is_shutdown():
		if not rospy.get_param("/isAutonomous") && not lastJoy == None:
		camControl(lastJoy.axes[6], lastJoy.axes[7])
		rospy.sleep()

    def CamControl(self,_dPadX, _dPadY):
        if _dPadX<0:
		self.CamPulse -= 10
	elif _dPadX>0:
		self.CamPulse += 10
	
	if _dPadY<0:
		self.ArmPulse -= 10
	elif _dPadY>0:
		self.ArmPulse += 10

	self.camAnglePub.publish(Int32(CamPulse))
	self.sensorArmAnglePub.publish(Int32(ArmAnglePulse))

        return (armVel, drumVel)
        
    def joyCallback(self, joy):
            lastJoy = joy;

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
    locController = DigCtlr()
    rospy.init_node('locomotion')
    rospy.spin()
	#i = 0
	#while True:
		#i += 0.5
		#print(controller.tankSteer(i,5))
