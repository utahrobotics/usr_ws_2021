#! /usr/bin/env python

import rospy

from initialize_server import InitializeServer
from arm_server import MoveArmServer, MoveDiggerServer, DumpServer


if __name__ == "__main__":
    rospy.init_node("actions")
    init = InitializeServer()
    dump_server = DumpServer()
    #init.start()
    dump_server.start()
    rospy.logwarn("Action servers started")
    #init.execute(None)
    rospy.spin()
