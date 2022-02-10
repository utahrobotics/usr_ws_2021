#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
import tf

class State():

	def __init__(self):
		rospy.init_node('robot_tf_listener')
		self.listener = tf.TransformListener()
		rospy.Timer(rospy.Duration(1.0), self.TransformPoint)
		rospy.spin()

	def TransformPoint(self, e):
		laser_point = PointStamped()
		laser_point.header.frame_id="base_cam1"
		laser_point.header.stamp = rospy.Time()
		laser_point.point.x = 1.0
		laser_point.point.y = 0.2
		laser_point.point.z = 0.0

		try:
			base_point = self.listener.transformPoint("base_link", laser_point)
			lp = laser_point.point
			bp = base_point.point
			#rospy.logwarn('base_cam1: (%d, %d, %d) -----> base_link: (%d, %d, %d) at time %d'%(lp.x, lp.y, lp.z, bp.x, bp.y, bp.z, base_point.header.stamp.to_sec()))

			rospy.logwarn(str(lp.x) + " " + str(lp.y) + " " + str(lp.z) + " -> " + str(bp.x) + " " + str(bp.y) + " " + str(bp.z) + " ")
		except e:
			rospy.logerr("transform err")

if __name__ == "__main__":
	s = State()
