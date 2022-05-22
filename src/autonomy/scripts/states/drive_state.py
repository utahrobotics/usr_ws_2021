import rospy
from extended_state import ExtendedState


class DriveState(ExtendedState):
	def __init__(self):
		ExtendedState.__init__(
			self,
			outcomes=['reached_dig_site', 'reached_bin', 'manual'],
			input_keys=["driving_to_site"],
		)
	
	def execute(self, userdata):
		userdata.current_state = 'Drive'
		# TODO: Use move_base
		if userdata.driving_to_site:
			pass  # TODO: drive to site
		else:
			pass  # TODO: drive to bin
		rospy.logwarn('Drove to bin')
		return 'reached_bin'
