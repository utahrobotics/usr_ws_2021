import rospy
from abstract_server import AbstractActionServer
from autonomy.msg import MoveArmAction, MoveDiggerAction
from std_msgs.msg import Float32


class MoveArmServer(AbstractActionServer):
    def __init__(self):
        AbstractActionServer.__init__(self, 'move_arm', MoveArmAction)
	self.arm_vel_pub = rospy.Publisher('arm_vel', Float32, queue_size=1)

    def execute(self, goal):
        rospy.sleep(1)
        if goal.extend:
            rospy.logwarn("Extended Arm")
        else:
            rospy.logwarn("Retracted Arm")


class MoveDiggerServer(AbstractActionServer):
    def __init__(self):
	self.drum_vel_pub = rospy.Publisher('drum_vel', Float32, queue_size=1)
        AbstractActionServer.__init__(self, 'move_digger', MoveDiggerAction)

    def execute(self, goal):
	if goal.digging:
		self.drum_vel_pub.publish(Float32(1))		# 1 radian per second?
	else:
		self.drum_vel_pub.publish(Float32(-1))		# reverse 1 radian per second?

        rospy.sleep(goal.duration)
	self.drum_vel_pub.publish(Float32(0))
        if goal.digging:
            rospy.logwarn("Dug")
        else:
            rospy.logwarn("Unloaded")
