#!/usr/bin/env python

import rospy
import tf

def main():
	rospy.init_node('depth_cam_tf_broadcaster')
	r = rospy.Rate(100)

	br = tf.TransformBroadcaster()

	while not rospy.is_shutdown():
		br.sendTransform((0.05, 0.0, 0.05), tf.transformations.quaternion_from_euler(0,0,0),
			rospy.Time.now(), "camera_link", "servo_link")
		br.sendTransform((0.05, 0.0, 0.1), tf.transformations.quaternion_from_euler(0,0,0),
			rospy.Time.now(), "realsense_link", "servo_link")
		br.sendTransform((0.0, 0.0, 0.05), tf.transformations.quaternion_from_euler(0,0,0),
			rospy.Time.now(), "pozyx_link", "base_link")
		br.sendTransform((0.3, 0.25, 0.25), tf.transformations.quaternion_from_euler(0,0,0),
			rospy.Time.now(), "aruco1", "map")
		br.sendTransform((0.3, -0.25, 0.25), tf.transformations.quaternion_from_euler(0,0,0),
					rospy.Time.now(), "aruco3", "map")
		r.sleep()
	

if __name__ == "__main__":
	main()
