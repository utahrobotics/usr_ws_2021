#!/usr/bin/env python
# defines a LocomotionController object which 
# interfaces with user input to control the robot.

from tokenize import Double


class LocCtlr:
# take range from -1 to 1 and traslate that to the angle we need to move the wheels. 
    scale=0
    throttle=0.0
    angle=0.0
    rightMotion =0.0
    leftMotion= 0.0
    wheetDist = 1.0
    def __init__(self,Scale):
        self.scale=Scale
        self.angle=0.0
        self.rightInput=0.0

    def tankSteer(self,_leftJoystick:Double, _rightJoystick:Double): 
        self.leftMotion= _leftJoystick*self.scale
        self.rightMotion= _rightJoystick*self.scale
        return (self.leftMotion,self.rightMotion)

    def translationControl(self,_leftJoystick:Double, _rightTrigger:Double):
        pass

    def radialSteer(self,_leftJoystick:Double, _rightTrigger:Double):
        pass

if __name__ == "__main__":
    controller = LocCtlr(10)
    print (controller.tankSteer(1,5))