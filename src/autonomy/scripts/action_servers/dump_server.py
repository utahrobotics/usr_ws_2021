import rospy
from abstract_server import AbstractActionServer
from locomotion.msg import SetSpeedGoal, SetSpeedAction, SteerAndThrottle
from autonomy.msg import DumpAction
from std_msgs.msg import Float32
from math import pi
from time import sleep


class DumpServer(AbstractActionServer):
    def __init__(self):
        self._arm_pub = rospy.Publisher('arm_vel', Float32, queue_size=1)
        self._drum_pub = rospy.Publisher('drum_vel', Float32, queue_size=1)
        self._drive_pub = rospy.Publisher('locomotion', SteerAndThrottle, queue_size=5)

        AbstractActionServer.__init__(self, "Dump", DumpAction)
    
    def execute(self, goal):
        # lift arm
        msg = Float32(-1)
        for _ in range(6):
            self._arm_pub.publish(msg)
            sleep(0.5)
        self._arm_pub.publish(Float32(0))
        
        sleep(0.5)
        
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
        
        sleep(0.5)

        # unload
        self._drum_pub.publish(Float32(-1))
        sleep(5)
        self._drum_pub.publish(Float32(0))
        
        sleep(0.5)
        
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
        
        sleep(0.5)

        # lower arm
        msg = Float32(1)
        for _ in range(6):
            self._arm_pub.publish(msg)
            sleep(0.5)
        self._arm_pub.publish(Float32(0))
