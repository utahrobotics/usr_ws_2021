#! /usr/bin/env python

import rospy

from initializer import Initialize
from digger import MoveArm


if __name__ == "__main__":
    rospy.init_node("actions")
    init = Initialize()
    dig = MoveArm()
    init.start()
    dig.start()
    rospy.logwarn("Action servers started")
    rospy.spin()
