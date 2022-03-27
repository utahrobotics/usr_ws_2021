import rospy
from abstract_server import AbstractActionServer
from autonomy.msg import MoveArmAction


class MoveArm(AbstractActionServer):
    def __init__(self):
        AbstractActionServer.__init__(self, 'move_arm', MoveArmAction)

    def execute(self, goal):
        rospy.loginfo("Executed")
