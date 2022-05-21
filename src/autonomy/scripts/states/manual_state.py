import rospy
from extended_state import ExtendedState


class ManualState(ExtendedState):
    def __init__(self):
        ExtendedState.__init__(
            self,
            outcomes=['drive', 'dig', 'unload'],
            input_keys=["action_client", 'current_state'],
            output_keys=["action_client", 'current_state']
        )

    def autonomy_callback(self, msg):
        ExtendedState.autonomy_callback(self, msg)
        rospy.logwarn("EXITING MANUAL MODE")

    def execute(self, userdata):
        rospy.logwarn("ENTERING MANUAL MODE")
        self.sleep(0)       # wait until autonomous
        return userdata.current_state.lower()
