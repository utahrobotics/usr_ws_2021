import rospy
from abstract_server import AbstractActionServer
from autonomy.msg import InitializeAction


class InitializeServer(AbstractActionServer):
    def __init__(self):
        AbstractActionServer.__init__(self, 'initialize', InitializeAction)

    def execute(self, _goal):
        """
        Should contain ALL the logic to startup and calibrate the bot
        @return: Warning codes (ie. non-fatal errors, fatal ones should be exceptions)
        """
        rospy.sleep(1)
        rospy.loginfo("Initialized")
