import rospy
from extended_state import ExtendedState


class ManualState(ExtendedState):
	def __init__(self):
		ExtendedState.__init__(
			self,
			outcomes=['drive', 'dig', 'unload'],
			input_keys=['current_state'],
			output_keys=['current_state']
		)
	
	def execute(self, userdata):
		if rospy.get_param("/isAutonomous"):
			rospy.logwarn("Entered Manual State while autonomous! Ignoring...")
			return userdata.current_state.lower()
		
		rospy.logwarn("ENTERING MANUAL MODE")
		self.sleep(0)       # wait until autonomous
		rospy.logwarn("EXITING MANUAL MODE")
		return userdata.current_state.lower()
