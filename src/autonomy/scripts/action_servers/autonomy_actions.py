#! /usr/bin/env python

import rospy

from initialize_server import InitializeServer
from arm_server import MoveArm, DigServer, DumpServer
from dump_server import DumpServer


if __name__ == "__main__":
	rospy.init_node("autonomy_actions")
	init = InitializeServer()
	dump_server = DumpServer()
	dig_server = DigServer()
	init.start()
	dump_server.start()
	dig_server.start()
	rospy.logwarn("Action servers started")
	rospy.spin()
