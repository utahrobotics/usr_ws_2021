import rospy
from extended_state import ExtendedState


class DriveState(ExtendedState):
	def __init__(self):
		ExtendedState.__init__(
			self,
			outcomes=['reached_dig_site', 'reached_bin', 'manual'],
			input_keys=["driving_to_site", "action_client"],
			output_keys=["action_client"]
		)
	
	def execute(self, userdata):
		userdata.current_state = 'Drive'
		# TODO: Use move_base
		rospy.logwarn('Drove to bin')
		return 'reached_bin'
