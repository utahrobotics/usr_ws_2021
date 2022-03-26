#! /usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib


_MoveBaseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)


def wait_for_server():
    _MoveBaseClient.wait_for_server()


def move_to(x, y):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    _MoveBaseClient.send_goal(goal)
