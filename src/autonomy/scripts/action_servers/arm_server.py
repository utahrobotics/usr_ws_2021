import rospy
from abstract_server import AbstractActionServer
from autonomy.msg import MoveArmAction, MoveDiggerAction


class MoveArmServer(AbstractActionServer):
    def __init__(self):
        AbstractActionServer.__init__(self, 'move_arm', MoveArmAction)

    def execute(self, goal):
        rospy.sleep(1)
        if goal.extend:
            rospy.logwarn("Extended Arm")
        else:
            rospy.logwarn("Retracted Arm")


class MoveDiggerServer(AbstractActionServer):
    def __init__(self):
        AbstractActionServer.__init__(self, 'move_digger', MoveDiggerAction)

    def execute(self, goal):
        rospy.sleep(1)
        if goal.digging:
            rospy.logwarn("Dug")
        else:
            rospy.logwarn("Unloaded")
