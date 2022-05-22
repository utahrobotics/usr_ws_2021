import rospy
from extended_state import ExtendedState
from actionlib import SimpleActionClient
from autonomy.msgs import DigAction, DigGoal


class DigState(ExtendedState):
	def __init__(self):
		self._dig_client = SimpleActionClient("Dig", DigAction)
		ExtendedState.__init__(
			self,
			outcomes=['finished', 'manual'],
			input_keys=["driving_to_site"],
			output_keys=["driving_to_site"]
		)
	
	def execute(self, userdata):
		userdata.current_state = 'Dig'
		self._dig_client.send_goal(DigGoal())
		self.wait_for_action_result(self._dig_client)
		userdata.driving_to_site = False
		return 'finished'
