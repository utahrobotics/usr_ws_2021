import rospy
from extended_state import ExtendedState
from autonomy.msgs import InitializeAction, InitializeGoal
from actionlib import SimpleActionClient


class InitializeState(ExtendedState):
	def __init__(self):
		self.init_client = SimpleActionClient("initialize", InitializeAction)
		ExtendedState.__init__(
			self,
			outcomes=['finished', 'manual'],
			input_keys=["action_client"],
			output_keys=["action_client"]
		)
	
	def execute(self, userdata):
		rospy.logwarn("Initing")
		self.init_client.send_goal(InitializeGoal())
		if self.wait_for_action_result(self.init_client):
			return 'manual'
		rospy.logwarn('Initialized')
		return 'finished'
