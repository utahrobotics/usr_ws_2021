import rospy
from abstract_server import AbstractActionServer
from autonomy.msg import DumpAction, DigAction
from std_msgs.msg import Float32, Twist
from locomotion.msg import SteerAndThrottle
from rospy import sleep


ARM_SPEED_PUB = rospy.Publisher('arm_vel', Float32, queue_size=1)
DRUM_SPEED_PUB = rospy.Publisher('drum_vel', Float32, queue_size=1)


class DumpServer(AbstractActionServer):
    def __init__(self):
        self._drive_pub = rospy.Publisher('locomotion', SteerAndThrottle, queue_size=5)
        
        AbstractActionServer.__init__(self, "Dump", DumpAction)
    
    def execute(self, goal):
        # TODO Alignment with fiducials
        # lift arm
        msg = Float32(-1)
        for _ in range(6):
            ARM_SPEED_PUB.publish(msg)
            sleep(0.5)
        ARM_SPEED_PUB.publish(Float32(0))
        
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
        DRUM_SPEED_PUB.publish(Float32(-1))
        sleep(5)
        DRUM_SPEED_PUB.publish(Float32(0))
        
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
            ARM_SPEED_PUB.publish(msg)
            sleep(0.5)
        ARM_SPEED_PUB.publish(Float32(0))


class DigServer(AbstractActionServer):
    def __init__(self):
        AbstractActionServer.__init__(self, "Dig", DigAction)
    
    def execute(self, goal):
        # TODO lower arm to correct a angle
        # lower arm
        msg = Float32(1)
        for _ in range(6):
            ARM_SPEED_PUB.publish(msg)
            sleep(0.5)
        ARM_SPEED_PUB.publish(Float32(0))
        
        sleep(0.5)
        
        # dig
        DRUM_SPEED_PUB.publish(Float32(1))
        sleep(7.5)
        DRUM_SPEED_PUB.publish(Float32(0))
        
        sleep(0.5)
        
        # raise arm higher
        msg = Float32(-1)
        for _ in range(12):
            ARM_SPEED_PUB.publish(msg)
            sleep(0.5)
        ARM_SPEED_PUB.publish(Float32(0))
