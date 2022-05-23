import rospy
from abstract_server import AbstractActionServer
from autonomy.msg import InitializeAction
from motors.msg import InitMotors, FakeInitAction, FakeInitGoal
from actionlib import SimpleActionClient
from std_msgs.msg import Int32


class InitializeServer(AbstractActionServer):
    def __init__(self):
        AbstractActionServer.__init__(self, 'initialize', InitializeAction)
        
        #self.init_motors_pub = rospy.Publisher(
            'init_motors',
            InitMotors,
            queue_size=1
        )
        self.fake_init = SimpleActionClient('fake_init_as', FakeInitAction)
        self.cam_angle_pub = rospy.Publisher('cam_angle', Int32, queue_size=1)


    def execute(self, _goal):
        """
        Should contain ALL the logic to startup and calibrate the bot
        @return: Warning codes (ie. non-fatal errors, fatal ones should be exceptions)
        """
        
        self.cam_angle_pub.publish(Int32(650))      # lift arm
        rospy.sleep(10)
        self.fake_init.send_goal(FakeInitGoal())
        self.fake_init.wait_for_result()
                
        rospy.logwarn("Initialized")

