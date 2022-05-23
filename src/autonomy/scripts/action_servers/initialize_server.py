import rospy
from abstract_server import AbstractActionServer
from autonomy.msg import InitializeAction
from motors.msg import InitMotors
from actionlib import SimpleActionClient


class InitializeServer(AbstractActionServer):
    def __init__(self):
        AbstractActionServer.__init__(self, 'initialize', InitializeAction)
        
        self.init_motors_pub = rospy.Publisher(
            'init_motors',
            InitMotors,
            queue_size=1
        )


    def execute(self, _goal):
        """
        Should contain ALL the logic to startup and calibrate the bot
        @return: Warning codes (ie. non-fatal errors, fatal ones should be exceptions)
        """
        self.init_motors_pub.publish(InitMotors())

        goal = MoveArmGoal()
        goal.extend = True
        self.move_arm.send_goal(goal)
        
        self.bypass_vel_pub.publish(BypassVelocity(-1, -1))

        rospy.sleep(4)
        
        self.bypass_vel_pub.publish(BypassVelocity(0, 0))
        
        rospy.sleep(16)
        
        rospy.logwarn("Initialized")
