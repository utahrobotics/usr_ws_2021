#! /usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from autonomy.msg import MoveArmAction, MoveArmGoal, InitializeAction, InitializeGoal


class ActionResult:
    """
    Returned from ActionClient's methods
    Represents future results from ActionServers
    """
    def __init__(self, server):
        self._server = server

    def wait_for_result(self):
        """
        Blocks until the goal is completed
        @return:
        """
        self._server.wait_for_result()
        return self._server.get_result()


class ActionClient(object):
    """
    This class is a wrapper for the ActionClients to send goals easily
    """
    def __init__(self):
        self._move_base = SimpleActionClient('move_base', MoveBaseAction)
        self._retract_arm = SimpleActionClient('retract_arm', MoveArmAction)
        self._initialize = SimpleActionClient('initialize', InitializeAction)

    def wait_for_servers(self, timeout=1):
        """
        Wait for the Action Server to startup
        I've faced issues with timeout being 0 (infinite), but 1 sec seems to work
        @param timeout: How long to wait for the ActionServer to spin up
        """
        # self._move_base.wait_for_server()     # Un comment when tf stops making warnings
        delay = rospy.Duration(timeout)
        self._retract_arm.wait_for_server(delay)
        self._initialize.wait_for_server(delay)

    def move_to(self, x, y, feedback_cb=None):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        self._move_base.send_goal(goal, feedback_cb=feedback_cb)
        return ActionResult(self._move_base)

    def retract_arm(self, fully_retract=False):
        goal = MoveArmGoal()
        goal.fully_retract = fully_retract
        self._retract_arm.send_goal(goal)
        return ActionResult(self._retract_arm)

    def initialize(self):
        self._initialize.send_goal(InitializeGoal())
        return ActionResult(self._initialize)
