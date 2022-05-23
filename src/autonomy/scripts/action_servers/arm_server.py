import rospy
from abstract_server import AbstractActionServer
from autonomy.msg import DumpAction, DigAction
from std_msgs.msg import Float32, Twist
from locomotion.msg import SteerAndThrottle


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


def safe_sleep(duration, rate=10):
    """
    Returns true if autonomy changed during the sleep
    """
    autonomy = rospy.get_param("/isAutonomous")
    polling_delay = 1.0 / rate
    ros_rate = rospy.Rate(rate)
    
    while duration > 0:
        ros_rate.sleep()
        duration -= polling_delay
        
        if rospy.get_param("/isAutonomous") != autonomy:
            return True
    
    return False


def drive_forward_for(duration, speed):
    goal = SteerAndThrottle()
    goal.angles = [90, 90, 90, 90]
    goal.throttles = [speed, speed, speed, speed]
    DRIVE_PUB.publish(goal)
    goal = SteerAndThrottle()
    goal.angles = [90, 90, 90, 90]
    goal.throttles = [0, 0, 0, 0]
    if safe_sleep(duration):
        DRIVE_PUB.publish(goal)
        return True
    DRIVE_PUB.publish(goal)
    return False


def lower_arm_to_depth(depth, timeout=0.0):
    handle_timeout = timeout > 0.0
    current_timeout = 0.0
    msg = Float32(1)
    autonomy = rospy.get_param("/isAutonomous")
    
    while ARM_DEPTH > depth:
        ARM_SPEED_PUB.publish(msg)
        POLLING_RATE.sleep()
        
        if autonomy != rospy.get_param("/isAutonomous"):
            break
        
        if handle_timeout:
            current_timeout += POLLING_DELAY
            if current_timeout >= timeout:
                break
    ARM_SPEED_PUB.publish(Float32(0))
    return autonomy != rospy.get_param("/isAutonomous")


def raise_arm_to_depth(depth, timeout=0.0):
    handle_timeout = timeout > 0.0
    current_timeout = 0.0
    msg = Float32(-1)
    autonomy = rospy.get_param("/isAutonomous")
    
    while ARM_DEPTH < depth:
        ARM_SPEED_PUB.publish(msg)
        POLLING_RATE.sleep()
        
        if autonomy != rospy.get_param("/isAutonomous"):
            break
        
        if handle_timeout:
            current_timeout += POLLING_DELAY
            if current_timeout >= timeout:
                break
    ARM_SPEED_PUB.publish(Float32(0))
    return autonomy != rospy.get_param("/isAutonomous")


def spin_drum_for(duration, speed):
    DRUM_SPEED_PUB.publish(Float32(speed))
    if safe_sleep(duration):
        DRUM_SPEED_PUB.publish(Float32(0))
        return True
    DRUM_SPEED_PUB.publish(Float32(0))
    return False


class DumpServer(AbstractActionServer):
    def __init__(self):
        AbstractActionServer.__init__(self, "Dump", DumpAction)
    
    def execute(self, goal):
        # TODO Alignment with fiducials
        if raise_arm_to_depth(0.6): return      # raise arm
        if drive_forward_for(4, 1): return      # drive forward
        if safe_sleep(0.3): return              # wait to stop shakes
        if spin_drum_for(5, -1): return          # unload for 5 secs
        if drive_forward_for(4, -1): return      # drive back
        if lower_arm_to_depth(0.3): return       # return to neutral


class DigServer(AbstractActionServer):
    def __init__(self):
        AbstractActionServer.__init__(self, "Dig", DigAction)
        self._digging_timeout = 5.0
    
    def execute(self, goal):
        DEPTH_STEP = 0.2
        for i in range(1, 3):
            DRUM_SPEED_PUB.publish(Float32(1))      # spin up drum
            if lower_arm_to_depth(- i * DEPTH_STEP, self._digging_timeout):
                DRUM_SPEED_PUB.publish(Float32(0))      # stop drum
                return     # lower arm to depth
            DRUM_SPEED_PUB.publish(Float32(0))      # stop drum
            if raise_arm_to_depth(0.2): return      # lift arm
            if drive_forward_for(2, 1): return      # drive forward for 2 secs
            if spin_drum_for(5, -1): return         # unload for 5 secs
            if drive_forward_for(2, -1): return     # drive back for 2 secs
