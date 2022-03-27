import rospy
from abstract_server import AbstractActionServer
from autonomy.msg import RetractArmAction


class RetractArm(AbstractActionServer):
    def __init__(self):
        AbstractActionServer.__init__(self, 'retract_arm', RetractArmAction)

    def execute(self, goal):
        rospy.loginfo("Executed")
