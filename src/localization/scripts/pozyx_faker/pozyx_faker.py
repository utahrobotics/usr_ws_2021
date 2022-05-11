#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    rospy.init_node('fake_pozyx_tf_broadcaster')
    pub = rospy.Publisher('sensors/pozyx/pose', PoseWithCovarianceStamped, queue_size=10)
    r = rospy.Rate(100)

    br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        t = rospy.Time.now()
        # br.sendTransform((0.5*math.cos(t.secs) + 2, 0.5*math.sin(t.secs), 0), tf.transformations.quaternion_from_euler(0, 0, 0),
        #                 rospy.Time.now(), "odom", "map")
        
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = 0.5*math.cos(t.secs) + 2
        pose.pose.pose.position.y = 0.5*math.sin(t.secs)
        #quaternion = tf.transformations.quaternion_from_euler(0,0,0)

        # pose.pose.pose.orientation.w = 1
        # pose.pose.pose.orientation.x = 0
        # pose.pose.pose.orientation.y = 0
        # pose.pose.pose.orientation.z = 0
        pub.publish(pose)
        r.sleep()


if __name__ == "__main__":
    main()
