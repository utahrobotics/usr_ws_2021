import rospy
from abstract_service import AbstractActionServer
from autonomy.msg import RetractArmAction


class RetractArm(AbstractActionServer):
    def __init__(self):
        super(RetractArm, self).__init__('retract_arm', RetractArmAction)

    def execute(self, goal):
        rospy.loginfo("Executed")
