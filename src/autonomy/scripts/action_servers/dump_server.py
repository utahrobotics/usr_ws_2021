import rospy
from abstract_server import AbstractActionServer
from actionlib import SimpleActionClient
from locomotion.msg import SetSpeedGoal, SetSpeedAction, SteerAndThrottle
from math import pi
from time import sleep


class DumpServer(AbstractActionServer):
    def __init__(self):
        self._move_arm = SimpleActionClient('set_arm_speed_as', SetSpeedAction)
        self._move_drum = SimpleActionClient('set_drum_speed_as', SetSpeedAction)
        timeout = rospy.Duration(3)
        self._move_arm.wait_for_server(timeout)
        self._move_drum.wait_for_server(timeout)

        self._drive_pub = rospy.Publisher('locomotion', SteerAndThrottle, queue_size=5)

        AbstractActionServer.__init__(self, "Dump", DumpAction)
    
    def execute(self, goal):
        # lift arm
        goal = SetSpeedGoal()
        goal.speed = -1.0
        for _ in range(6):
            self._move_arm.send_goal(goal)
            sleep(0.5)
        goal = SetSpeedGoal()       # Could just edit speed without reinstancing but be safe
        goal.speed = 0.0
        self._move_arm.send_goal(goal)
        
        # drive forward
        goal = SteerAndThrottle()
        goal.angles = [90, 90, 90, 90]
        goal.throttles = [1, 1, 1, 1]
        self._drive_pub.publish(goal)
        sleep(2)
        goal = SteerAndThrottle()
        goal.angles = [90, 90, 90, 90]
        goal.throttles = [0, 0, 0, 0]
        self._drive_pub.publish(goal)

        # unload
        goal = SetSpeedGoal()
        goal.speed = -1.0
        self._move_drum.send_goal(goal)
        sleep(5)
        goal = SetSpeedGoal()
        goal.speed = 0
        
        self._move_drum.send_goal(goal)
        
        # drive back
        goal = SteerAndThrottle()
        goal.angles = [90, 90, 90, 90]
        goal.throttles = [-1, -1, -1, -1]
        self._drive_pub.publish(goal)
        sleep(2)
        goal = SteerAndThrottle()
        goal.angles = [90, 90, 90, 90]
        goal.throttles = [0, 0, 0, 0]
        self._drive_pub.publish(goal)

        # lower arm
        goal = SetSpeedGoal()
        goal.speed = 1.0
        for _ in range(6):
            self._move_arm.send_goal(goal)
            sleep(0.5)
        goal = SetSpeedGoal()
        goal.speed = 0.0
        self._move_arm.send_goal(goal)
