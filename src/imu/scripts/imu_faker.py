#!/usr/bin/env python
import rospy
import tf
import math


def main():
    rospy.init_node('fake_imu_tf_broadcaster')
    r = rospy.Rate(100)

    br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        t = rospy.Time.now()
        br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(), "base_link", "odom")
        r.sleep()


if __name__ == "__main__":
    main()
