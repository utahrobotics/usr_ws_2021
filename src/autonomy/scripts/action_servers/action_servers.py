#! /usr/bin/env python

import rospy

from initialize_server import InitializeServer
from arm_server import MoveArmServer, MoveDiggerServer


if __name__ == "__main__":
    rospy.init_node("actions")
    init = InitializeServer()
    move_arm = MoveArmServer()
    move_digger = MoveDiggerServer()
    init.start()
    move_arm.start()
    move_digger.start()
    rospy.logwarn("Action servers started")
    init.execute(None)
    rospy.spin()
