#! /usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from autonomy.msg import MoveArmAction, MoveArmGoal, InitializeAction, InitializeGoal, MoveDiggerGoal, MoveDiggerAction


class ActionResult:
    """
    Returned from ActionClient's methods
    Represents future results from ActionServers
    """
    def __init__(self, server):
        self._server = server

    def get_state(self):
        """
        Returns the state of action server
        https://docs.ros.org/en/kinetic/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html#a1496dbc011f48451f4ea98e1ad2f8cd9

        Uses enums from this message object
        from actionlib_msgs.msg import GoalStatus
        eg. GoalStatus.PENDING

        @return: Status of the action server
        """
        return self._server.get_state()

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
        self._move_arm = SimpleActionClient('move_arm', MoveArmAction)
        self._move_digger = SimpleActionClient('move_digger', MoveDiggerAction)
        self._initialize = SimpleActionClient('initialize', InitializeAction)

    def wait_for_servers(self, timeout=3):
        """
        Wait for the Action Server to startup
        I've faced issues with timeout being 0 (infinite), but 3 sec seems to work
        @param timeout: How long to wait for the ActionServer to spin up
        """
        delay = rospy.Duration(timeout)
        # self._move_base.wait_for_server(delay)     # Un comment when tf stops making warnings
        self._move_arm.wait_for_server(delay)
        self._initialize.wait_for_server(delay)

    def move_to(self, x, y, feedback_cb=None):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        self._move_base.send_goal(goal, feedback_cb=feedback_cb)
        return ActionResult(self._move_base)

    def move_arm(self, extend=False):
        goal = MoveArmGoal()
        goal.extend = extend
        self._move_arm.send_goal(goal)
        return ActionResult(self._move_arm)

    def move_digger(self, digging=False, duration=15.0):
        goal = MoveDiggerGoal()
        goal.digging = digging
        goal.duration = duration
        self._move_digger.send_goal(goal)
        return ActionResult(self._move_digger)

    def initialize(self):
        self._initialize.send_goal(InitializeGoal())
        return ActionResult(self._initialize)
