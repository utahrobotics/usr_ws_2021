import rospy
from extended_state import ExtendedState


class InitializeState(ExtendedState):
    def __init__(self):
        ExtendedState.__init__(
            self,
            outcomes=['finished', 'manual'],
            input_keys=["action_client"],
            output_keys=["action_client"]
        )

    def execute(self, userdata):
        userdata.current_state = 'Init'
        rospy.logwarn("Initializing")
        userdata.action_client.initialize().wait_for_result()
        rospy.logwarn('Initialized')
        return 'finished'
