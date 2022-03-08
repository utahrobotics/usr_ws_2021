#!/usr/bin/env python
import rospy
import tf
import math


def main():
    rospy.init_node('fake_pozyx_tf_broadcaster')
    r = rospy.Rate(100)

    br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        t = rospy.Time.now()
        br.sendTransform((0.5*math.cos(t.secs) + 2, 0.5*math.sin(t.secs), 0), tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(), "odom", "map")
        r.sleep()


if __name__ == "__main__":
    main()
