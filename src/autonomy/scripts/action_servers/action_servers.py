#! /usr/bin/env python

import rospy

from initializer import Initialize
from digger import MoveArm

init = Initialize()
dig = MoveArm()


def start_servers():
    init.start()
    dig.start()


if __name__ == "__main__":
    rospy.init_node("actions")
    start_servers()
    rospy.logwarn("Action servers started")
    rospy.spin()
