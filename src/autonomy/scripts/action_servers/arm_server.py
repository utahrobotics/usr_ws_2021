import rospy
from abstract_server import AbstractActionServer
from autonomy.msg import DumpAction, DigAction
from std_msgs.msg import Float32, Twist
from locomotion.msg import SteerAndThrottle
from time import time as get_time


ARM_SPEED_PUB = rospy.Publisher('arm_vel', Float32, queue_size=1)
DRUM_SPEED_PUB = rospy.Publisher('drum_vel', Float32, queue_size=1)
DRIVE_PUB = rospy.Publisher('locomotion', SteerAndThrottle, queue_size=5)
POLLING_DELAY = 0.1
POLLING_RATE = rospy.Duration(POLLING_DELAY)

ARM_DEPTH = 0.0


def update_arm_depth(msg):
    global ARM_DEPTH
    ARM_DEPTH = msg.data


rospy.Subscriber('/sensors/angleSensor/depth', Float32, update_arm_depth)


def drive_forward_for(duration, speed):
    goal = SteerAndThrottle()
    goal.angles = [90, 90, 90, 90]
    goal.throttles = [speed, speed, speed, speed]
    DRIVE_PUB.publish(goal)
    rospy.sleep(duration)
    goal = SteerAndThrottle()
    goal.angles = [90, 90, 90, 90]
    goal.throttles = [0, 0, 0, 0]
    DRIVE_PUB.publish(goal)


def lower_arm_to_depth(depth, timeout=0.0):
    handle_timeout = timeout > 0.0
    current_timeout = 0.0
    msg = Float32(1)
    while ARM_DEPTH > depth:
        ARM_SPEED_PUB.publish(msg)
        POLLING_RATE.sleep()
        if handle_timeout:
            current_timeout += POLLING_DELAY
            if current_timeout >= timeout:
                break
    ARM_SPEED_PUB.publish(Float32(0))


def raise_arm_to_depth(depth, timeout=0.0):
    handle_timeout = timeout > 0.0
    current_timeout = 0.0
    msg = Float32(-1)
    while ARM_DEPTH < depth:
        ARM_SPEED_PUB.publish(msg)
        POLLING_RATE.sleep()
        if handle_timeout:
            current_timeout += POLLING_DELAY
            if current_timeout >= timeout:
                break
    ARM_SPEED_PUB.publish(Float32(0))


def spin_drum_for(duration, speed):
    DRUM_SPEED_PUB.publish(Float32(speed))
    rospy.sleep(duration)
    DRUM_SPEED_PUB.publish(Float32(0))


class DumpServer(AbstractActionServer):
    def __init__(self):
        AbstractActionServer.__init__(self, "Dump", DumpAction)
    
    def execute(self, goal):
        # TODO Alignment with fiducials
        raise_arm_to_depth(0.6)     # raise arm
        drive_forward_for(4, 1)     # drive forward
        rospy.sleep(0.3)            # wait to stop shakes
        spin_drum_for(5, -1)        # unload for 5 secs
        drive_forward_for(4, -1)    # drive back
        lower_arm_to_depth(0.3)     # return to neutral


class DigServer(AbstractActionServer):
    def __init__(self):
        # sensors/angles
        AbstractActionServer.__init__(self, "Dig", DigAction)
        self._digging_timeout = 5.0
    
    def execute(self, goal):
        DEPTH_STEP = 0.2
        for i in range(1, 3):
            DRUM_SPEED_PUB.publish(Float32(1))      # spin up drum
            lower_arm_to_depth(- i * DEPTH_STEP, self._digging_timeout)     # lower arm to depth
            DRUM_SPEED_PUB.publish(Float32(0))      # stop drum
            raise_arm_to_depth(0.2)     # lift arm
            drive_forward_for(2, 1)     # drive forward for 2 secs
            spin_drum_for(5, -1)        # unload for 5 secs
            drive_forward_for(2, -1)    # drive back for 2 secs
