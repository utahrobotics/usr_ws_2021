import smach
import rospy
from std_msgs.msg import Twist
import actionlib

from autonomy.msg import DriveToGoal, DriveToAction


__all__ = []		# TODO


class Reorientation(smach.State):
	def __init__(self):
		smach.State.__init__(
			self,
			outcomes=['oriented']
		)
	
	def execute(self, _userdata):
		pass


class Driving(smach.State):
	def __init__(self):
		smach.State.__init__(
			self,
			outcomes=['oriented'],
			input_keys=["drive_to_client", "location"]
		)

	def execute(self, userdata):
		goal = DriveToGoal()
		goal.location = userdata.location
		userdata.drive_to_client.send_goal(goal)
		userdata.wait_for_result()

		# TODO Handle error
		error = userdata.drive_to_client.get_result()

		return 'oriented'


class Digging(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=["dug"])
	
	def execute(self, _userdata):
		pass


class Unloading(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=["empty"])
	
	def execute(self, _userdata):
		pass


def initialize_DriveTo_client():
	client = actionlib.SimpleActionClient('drive_to', DriveToAction)
	client.wait_for_server()
	return client


def initialize_autonomy_topic():
	return rospy.Publisher('autonomy', Twist, queue_size=10)
	

