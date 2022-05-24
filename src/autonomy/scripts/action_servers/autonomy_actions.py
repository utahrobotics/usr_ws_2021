#! /usr/bin/env python

import rospy

# from initialize_server import InitializeServer
from dump_server import DumpServer
from arm_server import DigServer


if __name__ == "__main__":
    rospy.init_node("autonomy_actions")
    # init = InitializeServer()
    dig_server = DigServer()
    # init.start()
    dig_server.start()
    rospy.logwarn("Action servers started")
    rospy.spin()
