#! /usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from autonomy.msg import MoveArmAction, MoveArmGoal, InitializeAction, InitializeGoal


class ActionResult:
    def __init__(self, server):
        self._server = server

    def wait_for_result(self):
        self._server.wait_for_result()
        return self._server.get_result()


class ActionClient(object):
    def __init__(self):
        self._move_base = SimpleActionClient('move_base', MoveBaseAction)
        self._retract_arm = SimpleActionClient('retract_arm', MoveArmAction)
        self._initialize = SimpleActionClient('initialize', InitializeAction)

    def wait_for_servers(self):
        # self._move_base.wait_for_server()     # Un comment when tf stops making warnings
        self._retract_arm.wait_for_server()
        self._initialize.wait_for_server()

    def move_to(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        self._move_base.send_goal(goal)
        return ActionResult(self._move_base)

    def retract_arm(self, fully_retract=False):
        goal = MoveArmGoal()
        goal.fully_retract = fully_retract
        self._retract_arm.send_goal(goal)
        return ActionResult(self._retract_arm)

    def initialize(self):
        self._initialize.send_goal(InitializeGoal())
        return ActionResult(self._initialize)
