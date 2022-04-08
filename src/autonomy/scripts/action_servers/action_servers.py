#! /usr/bin/env python

import rospy

from initialize_server import Initialize
from arm_server import MoveArm


if __name__ == "__main__":
    rospy.init_node("actions")
    init = Initialize()
    dig = MoveArm()
    init.start()
    dig.start()
    rospy.logwarn("Action servers started")
    rospy.spin()
