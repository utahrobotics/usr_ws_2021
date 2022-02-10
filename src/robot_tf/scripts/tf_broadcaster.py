#!/usr/bin/env python

import rospy
import tf

def main():
	rospy.init_node('robot_tf_broadcaster')
	r = rospy.Rate(100)

	br = tf.TransformBroadcaster()

	while not rospy.is_shutdown():
		br.sendTransform((0.2, 0.1, 0.3), tf.transformations.quaternion_from_euler(0,0,0),
			rospy.Time.now(), "base_cam1", "base_link")
		br.sendTransform((0.2, -0.1, 0.3), tf.transformations.quaternion_from_euler(0,0,0),
					rospy.Time.now(), "base_cam2", "base_link")
		r.sleep()
	

if __name__ == "__main__":
	main()
