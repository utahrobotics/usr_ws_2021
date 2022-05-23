#!/usr/bin/env python
# defines a LocomotionController object which
# interfaces with user input to control the robot.

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from locomotion.msg import SteerAndThrottle
from sensor_msgs.msg import Joy
from std_msgs.msg import Header


class LocCtlr:
    # take range from -1 to 1 and translate that to the angle we need to move the wheels.
    scale = 0
    throttle = 0.0
    frontAngle = 0.0
    backAngle = 0.0
    rightMotion = 0.0
    leftMotion = 0.0
    wheelDist = 1.0
    currentStartButtonState = False
    previousStartButtonState = False
    steeringType = 0
    inputScaleFactor = 1
    Lfront = 0.25
    Lback = 0.3
    Wleft = 0.2
    Wright = 0.2
    
    def __init__(self, loc_pub):
        self.loc_pub = loc_pub
        self.angle = 0.0
        self.rightInput = 0.0
    
    def tankSteer(self, left_joystick, right_joystick):
        self.leftMotion = left_joystick
        self.rightMotion = right_joystick
        angles = [90, 90, 90, 90]
        velocities = [self.leftMotion, self.leftMotion, self.rightMotion, self.rightMotion]
        H = Header()
        H.stamp = rospy.Time.now()
        msg = SteerAndThrottle()
        msg.header = H
        msg.angles = angles
        msg.throttles = velocities
        self.loc_pub.publish(msg)
        return (angles, velocities)
    
    def translationControl(self, left_joystickY, right_joystickX):
        self.wheelCenter = [0, 0.5]
        angle = (right_joystickX * 90)
        angles = [90 + angle, 90 + angle, 90 + angle, 90 + angle]
        
        vel = left_joystickY
        velocities = [vel, vel, vel, vel]
        H = Header()
        H.stamp = rospy.Time.now()
        msg = SteerAndThrottle()
        msg.header = H
        msg.angles = angles
        msg.throttles = velocities
        self.loc_pub.publish(msg)
        return (angles, velocities)
    
    def radialSteer(self, left_joystickY, right_joystickX):
        angle = (right_joystickX * 90)
        angles = [90 - angle, 90 + angle, 90 - angle, 90 + angle]
        
        vel = left_joystickY
        velocities = [vel, vel, vel, vel]
        H = Header()
        H.stamp = rospy.Time.now()
        msg = SteerAndThrottle()
        msg.header = H
        msg.angles = angles
        msg.throttles = velocities
        self.loc_pub.publish(msg)
        return (angles, velocities)
    
    def ackermanSteer(self, left_joystickY, right_joystickx):
        # if (right_joystickx == 0): # avoid divide by zero error
        #     angle1 = 90
        #     angle2 = 90
        #     angle3 = 90
        #     angle4 = 90
        # else:
        if (right_joystickx == -1):  # avoid a discontinuity that occurs when input = -1
            right_joystickx = -0.99999
        
        R = self.inputScaleFactor * (1 / right_joystickx) - (
                    (right_joystickx / abs(right_joystickx)) * self.inputScaleFactor)
        angleAckFront = np.atan(self.Lfront / R)
        angleAckBack = np.atan(-self.Lback / R)
        
        angle1 = 90 + np.degrees(np.arctan2(self.Lfront * np.sin(angleAckFront),
                                            (self.Lfront * np.cos(angleAckFront) - self.Wleft * np.sin(angleAckFront))))
        angle2 = 90 + np.degrees(np.arctan2(self.Lfront * np.sin(angleAckFront), (
                    self.Lfront * np.cos(angleAckFront) + self.Wright * np.sin(angleAckFront))))
        angle3 = 90 + np.degrees(np.arctan2(self.Lback * np.sin(angleAckBack),
                                            (self.Lfront * np.cos(angleAckBack) - self.Wleft * np.sin(angleAckBack))))
        angle4 = 90 + np.degrees(np.arctan2(self.Lback * np.sin(angleAckBack),
                                            (self.Lfront * np.cos(angleAckBack) + self.Wright * np.sin(angleAckBack))))
        
        angles = [angle1, angle2, angle3, angle4]
        
        vel = left_joystickY
        velocities = [vel, vel, vel, vel]
        H = Header()
        H.stamp = rospy.Time.now()
        msg = SteerAndThrottle()
        msg.header = H
        msg.angles = angles
        msg.throttles = velocities
        self.loc_pub.publish(msg)
        return (angles, velocities)
    
    def joyCallback(self, joy):
        if not rospy.get_param("/isAutonomous"):
            
            self.currentStartButtonState = joy.buttons[9]
            if self.currentStartButtonState and not self.previousStartButtonState:
                self.steeringType = (self.steeringType + 1) % 4
                rospy.logwarn("switched to drive mode " + str(self.steeringType))
            if self.steeringType == 0:
                # error source
                self.tankSteer(joy.axes[1], joy.axes[5])
            if self.steeringType == 1:
                self.radialSteer(joy.axes[1], joy.axes[2])
            if self.steeringType == 2:
                self.translationControl(joy.axes[1], joy.axes[2])
            if self.steeringType == 3:
                self.ackermanSteer(joy.axes[1], joy.axes[2])
            self.previousStartButtonState = self.currentStartButtonState
    
    def autonomy_callback(self, twist):
        if rospy.get_param("/isAutonomous"):
            linear_x = twist.linear.x
            angular_z = twist.angular.z
            
            left_speed = linear_x + angular_z
            right_speed = linear_x - angular_z
            
            self.tankSteer(left_speed, right_speed)


if __name__ == "__main__":
    pub = rospy.Publisher('locomotion', SteerAndThrottle, queue_size=1)
    locController = LocCtlr(pub)
    rospy.init_node('locomotion')
    rospy.Subscriber("telemetry_joy", Joy, locController.joyCallback, queue_size=1)
    rospy.Subscriber("cmd_vel", Twist, locController.autonomy_callback, queue_size=1)
    rospy.spin()
