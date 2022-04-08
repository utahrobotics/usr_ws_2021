from smach import State
import rospy


class InitializeState(State):
    def __init__(self):
        State.__init__(self, outcomes=['finished'], input_keys=["action_client"], output_keys=["action_client"])

    def execute(self, userdata):
        rospy.logwarn("Initializing")
        userdata.action_client.initialize().wait_for_result()
        rospy.logwarn('Initialized')
        return 'finished'
