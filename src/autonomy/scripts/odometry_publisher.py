#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, TwistStamped
import math

odom_broadcaster = tf.TransformBroadcaster()
odom_pub = rospy.Publisher('nav_msgs/Odometry', Odometry, queue_size=50)
x = 0.0
y = 0.0
th = 0.0

current_time = rospy.get_rostime()
last_time = rospy.get_rostime()

def odomPubCallback(twistStamped ):
	vx = twistStamped.twist.linear.x
	vy = twistStamped.twist.linear.y
	vth = twistStamped.twist.angular.z

	current_time = rospy.get_rostime()

	dt = (current_time - last_time).secs
	dx = (vx * math.cos(th) - vy * math.sin(th)) * dt
	dy = (vy * math.cos(th) - vy * math.sin(th)) * dt
	dth = vth * dt

	x += dx
	y += dy
	th += dth

	odom_quat = tf.createQuaternionMsgFromYaw(th)

	#first, we'll publish the transform over tf
	odom_trans = TransformStamped()
	odom_trans.header.stamp = current_time
	odom_trans.header.frame_id = "odom"
	odom_trans.child_frame_id = "base_link"

	odom_trans.transform.translation.x = x
	odom_trans.transform.translation.y = y
	odom_trans.transform.translation.z = 0.0
	odom_trans.transform.rotation = odom_quat

	odom_broadcaster.sendTransform(odom_trans)

	#next, we'll publish the odometry message over ROS
	odom = Odometry()
	odom.header.stamp = current_time
	odom.header.frame_id = "odom"

	#set the position
	odom.pose.pose.position.x = x
	odom.pose.pose.position.y = y
	odom.pose.pose.position.z = 0.0
	odom.pose.pose.orientation = odom_quat

	#set the velocity
	odom.child_frame_id = "base_link"
	odom.twist.twist.linear.x = vx
	odom.twist.twist.linear.y = vy
	odom.twist.twist.angular.z = vth

	#publish the message
	odom_pub.publish(odom)

	last_time = current_time

def main():
	rospy.init_node('robot_pose_tf_broadcaster')
	rospy.Subscriber("imu/vel", TwistStamped, odomPubCallback)
	rospy.spin()
