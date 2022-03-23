#! /usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

MoveBaseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)

def main():
    rospy.init_node("simple_goal_test")
    MoveBaseClient.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 6.0
    goal.target_pose.pose.position.y = 0.5
    goal.target_pose.pose.orientation.w = 1.0
    
    MoveBaseClient.send_goal(goal)
    wait = MoveBaseClient.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
    else:
        rospy.logwarn("goal sent")


if __name__ == "__main__":
    main()
