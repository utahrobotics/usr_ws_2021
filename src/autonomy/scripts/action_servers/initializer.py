import rospy
from abstract_server import AbstractActionServer
from autonomy.msg import InitializeAction


class Initialize(AbstractActionServer):
    def __init__(self):
        AbstractActionServer.__init__(self, 'initialize', InitializeAction)

    def execute(self, goal):
        rospy.loginfo("Initialized")
